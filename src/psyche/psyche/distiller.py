import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String

from langchain_core.prompts import PromptTemplate

from psyche_interfaces.action import PlainTextInference

import yaml

class Distiller(Node):
    """
    A node that listens to one or more topics, transforms them via an LPU and publishes the topic
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameters('', [
            ('narrative', ''),
            ('prompt', """Compress, summarize, emphasize parts of, and/or otherwise transform the input into a coherent output (topic name={output_topic}) in terms of the narrative. Phrase it in terms of the narrative as it understands itself; don't reference the narrative itself. Do not invent data that is not here. Only use the data that is presented here.
                Narrative: {narrative}
                Input topics:
                {input_topics}
            
            Interpretation:    
            """),
            ('input_topics', ['sense_of_self', 'context']),
            ('output_topic', ''),
            ('update_interval', 60.0),
            ('action_server_name', 'instruct')
        ])
        
        self.narrative = self.get_parameter('narrative').get_parameter_value().string_value
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value
        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        self.action_server_name = self.get_parameter('action_server_name').get_parameter_value().string_value
        
        self.action_client = ActionClient(self, PlainTextInference, self.action_server_name)
        
        self.output_pub = self.create_publisher(String, self.output_topic, 4)
        self.input_queue = {}
        self.input_subs = []
        for topic in self.input_topics:
            self.input_queue[topic] = []
            self.input_subs.append(self.create_subscription(String, topic, self.queue_message_callback(topic), 4))            
        self.get_logger().info(f'Listening to {self.input_topics}')
        self.timer = self.create_timer(self.update_interval, self.update)
        self.get_logger().debug(f'Timer set to {self.update_interval} seconds')
        self.update()

    def queue_message_callback(self, topic):
        def callback(msg):
            self.get_logger().debug(f'Got message on {topic}: {msg.data}')
            self.queue_message(msg, topic)
        return callback
                
    # This should be overridden by the subclass
    def transform_topic(self, topic_name: str, msg):
        """Render the message from the specified topic into a string"""
        self.get_logger().debug(f'transforming topic {topic_name}')
        if type(msg) == String:
            return msg.data
        
        if hasattr(msg, 'data'):
            return str(msg.data)
        
        return str(msg)
    
    def queue_message(self, msg, topic):
        self.get_logger().debug(f'Queueing message on {topic}: {msg.data}')
        if topic not in self.input_queue:
            self.input_queue[topic] = []
        self.input_queue[topic].append(msg)
    
    def transform_inputs(self, inputs):
        '''A hook to transform the inputs before they are passed to the prompt'''
        self.get_logger().debug('Transforming inputs')
        dumped = yaml.dump(inputs, default_flow_style=False)
        self.get_logger().debug(f'Dumped inputs: {dumped}')
        return dumped
    
    def update(self):
        self.get_logger().debug('Distilling...')
        if not self.prompt:
            self.get_logger().error('No prompt set')
            raise ValueError('No prompt set')
        
        inputs = {}
        for topic, messages in self.input_queue.items():
            self.get_logger().debug(f'Processing {len(messages)} messages on {topic}')
            inputs[topic] = [self.transform_topic(topic, msg) for msg in messages]
    
        self.input_queue = {}
        self.get_logger().debug(f"Inputs: {inputs}")
        inputs = (self.transform_inputs(inputs)).strip()
        if inputs == '' or inputs == '{}' or inputs == '' or inputs == '[]' or inputs == {} or inputs == []:
            self.get_logger().debug('No inputs--skipping prompt')
            return
        self.get_logger().debug(f'Prompting with inputs: {inputs}')
        prompt=self.prompt.format(
            narrative=self.narrative,
            output_topic=self.output_topic,
            input_topics=inputs
        )
        self.get_logger().debug(f'Prompt: {prompt}; awaiting action server {self.action_server_name}')
        goal = PlainTextInference.Goal(prompt=prompt)
        self.action_client.wait_for_server()
        self.get_logger().debug(f"Action server {self.action_server_name} found")
        future = self.action_client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_inferred)
        
    def on_chunk(self, chunk: str):
        '''A hook for raw chunks sent back from the inference server'''
        pass
    
    def on_word(self, word: str):
        '''A hook for units of text _on par_ with "words". In the last sentence ["_", "on", "par_", "with", '"', "words"""] is more like it.'''
        pass
    
    # This is helpful if you are sending this to a TTS node
    def on_sentence(self, sentence: str):
        '''A hook for full sentences. This is the most useful for most applications, like streaming to /voice. (/voice will get its own sentence splitting, but this allows asynchronous speech and yields to other nodes in the meantime.)'''
        pass
        
    def on_result(self, result: str):
        '''A hook for the final result. Use the on_sentence hook if you can possibly use the results in a streaming fashion. This is the last hook called.'''
        self.output_pub.publish(String(data=result))
        
    def goal_response_callback(self, future) -> None:
        """
        Callback for handling the response after sending the goal.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().debug('Goal rejected :(')
                return

            self.get_logger().debug('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.on_response)
        except Exception as e:
            self.get_logger().error(f'Error in goal response callback: {e}')

    def on_inferred(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return
            
            self.get_logger().debug('Goal accepted')
            future = goal_handle.get_result_async()
            future.add_done_callback(self.on_response)
        except Exception as e:
            self.get_logger().error(f'Error in inference done: {e}')
    
    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        self.get_logger().debug(f'Feedback: {feedback.chunk.chunk}')
        
        if feedback.chunk.level == 0:
            self.on_chunk(feedback.chunk.chunk)
        elif feedback.chunk.level == 1:
            self.on_word(feedback.chunk.chunk)
        elif feedback.chunk.level == 2:
            self.on_sentence(feedback.chunk.chunk)
        else:
            self.get_logger().error('Unknown feedback level')
    
    def on_response(self, future):
        try:
            result = future.result().result
            self.on_result(result.response)
        except Exception as e:
            self.get_logger().error(f'Error in processing response: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    # TODO: Get a better name in here for this
    node = Distiller('distiller')
    rclpy.spin(node)

if __name__ == '__main__':
    main()