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
            ('prompt', """Compress, summarize, emphasize parts of, and/or otherwise transform the input into a coherent output (topic name={output_topic}) in terms of the narrative. Phrase it in terms of an inhabitant in the narrative. Don't "break character".
                Narrative: {narrative}
                Input topics:
                {input_topics}
            
            Interpretation:    
            """),
            ('input_topics', []),
            ('output_topic', ''),
            ('update_interval', 60.0)
        ])
        
        self.narrative = self.get_parameter('narrative').get_parameter_value().string_value
        self.input_topic_list = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value
        self.prompt = self.get_parameter('prompt').get_parameter_value().string_value
        
        self.action_client = ActionClient(self, PlainTextInference, "/infer")
        
        self.output_pub = self.create_publisher(String, self.output_topic, 10)
        self.input_subs = []
        self.input_queue = {}
        self.input_subs = [self.create_subscription(String, topic, self.queue_message_callback(topic), 10) for topic in self.input_topic_list]
        
        self.timer = self.create_timer(self.update_interval, self.update)
        self.update()

    def queue_message_callback(self, topic):
        def callback(msg):
            self.queue_message(msg, topic)
        return callback
                
    # This should be overridden by the subclass
    def transform_topic(self, topic_name: str, msg):
        """Render the message from the specified topic into a string"""
        if type(msg) == String:
            return msg.data
        
        if hasattr(msg, 'data'):
            return str(msg.data)
        
        return str(msg)
    
    def queue_message(self, msg, topic):
        if topic not in self.input_queue:
            self.input_queue[topic] = []
        self.input_queue[topic].append = msg
    
    def transform_inputs(self, inputs):
        '''A hook to transform the inputs before they are passed to the prompt'''
        return yaml.dump(inputs, default_flow_style=False)
    
    def update(self):
        if not self.prompt:
            self.get_logger().error('No prompt set')
            raise ValueError('No prompt set')
        
        inputs = {}
        for topic, messages in self.input_queue.items():
            inputs[topic] = [self.transform_topic(topic, msg) for msg in messages]
    
        inputs = self.transform_inputs(inputs)
        self.action_client.wait_for_server()
        goal = PlainTextInference.Goal(prompt=self.prompt.format(
            narrative=self.narrative,
            output_topic=self.output_topic,
            input_topics=inputs
        ))
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
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')
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
            
            self.get_logger().info('Goal accepted')
            future = goal_handle.get_result_async()
            future.add_done_callback(self.on_response)
        except Exception as e:
            self.get_logger().error(f'Error in inference done: {e}')
    
    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
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
            self.on_result(result)
        except Exception as e:
            self.get_logger().error(f'Error in processing response: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    # TODO: Get a better name in here for this
    node = Distiller('distiller')
    rclpy.spin(node)

if __name__ == '__main__':
    main()