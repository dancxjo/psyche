import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String

from psyche_interfaces.action import PlainTextInference
from psyche_interfaces.message import StreamedChunk

class Distiller(Node):
    """
    A node that listens to one or more topics, transforms them via an LPU and publishes the topic
    """
    def __init__(self):
        self.declare_parameters('distiller', [
            ('node_name', ''),
            ('action_server_name', '/infer'),
            ('prompt_topic', ''),
            ('input_topics', []),
            ('output_topic', ''),
        ])
        
        self.node_name = self.get_parameter('node_name').string_value
        self.action_server_name = self.get_parameter('action_server_name').string_value
        self.prompt_topic = self.get_parameter('prompt_topic').string_value
        self.input_topics = self.get_parameter('input_topics').string_array_value
        self.output_topic = self.get_parameter('output_topic').string_value
        self.update_interval = self.get_parameter('update_interval').double_value
        
        if not (self.node_name and self.action_server_name and self.prompt_topic and self.input_topics and self.output_topic):
            self.get_logger().error('Missing parameters')
            raise ValueError('Missing parameters')
        
        super().__init__(self.node_name)
        self.action_client = ActionClient(self, PlainTextInference, self.action_server_name)
        
        self.prompt_sub = self.create_subscription(String, self.prompt_topic, self.update_prompt, 10)
        
        self.output_pub = self.create_publisher(String, self.output_topic, 10)
        self.input_subs = []
        self.input_queue = {}
        for topic in self.input_topics:
            self.input_subs.append(self.create_subscription(String, topic, lambda msg, topic=topic: self.queue_message(msg, topic), 10))
        
        self.timer = self.create_timer(self.update_interval, self.update)
        
    # This should be overridden by the subclass
    def transform_topic(self, topic_name: str, msg):
        """Render the message from the specified topic into a string"""
        if type(msg) == String:
            return msg.data
        
        if hasattr(msg, 'data'):
            return str(msg.data)
        
        return str(msg)

    def update_prompt(self, msg):
        self.prompt = msg.data
    
    def queue_message(self, msg, topic):
        if topic not in self.input_queue:
            self.input_queue[topic] = []
        self.input_queue[topic].append = msg
    
    def update(self):
        if not self.prompt:
            self.get_logger().error('No prompt set')
            raise ValueError('No prompt set')
        
        inputs = {}
        for topic, messages in self.input_queue.items():
            inputs[topic] = [self.transform_topic(topic, msg) for msg in messages]
    
        prompt = self.prompt.format(**inputs)
        self.get_logger().info(f'Prompt: {prompt}')
        self.action_client.wait_for_server()
        goal = PlainTextInference.Goal(prompt=prompt)
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.on_inferred)
        
    def on_chunk(self, chunk: str):
        pass
    
    def on_word(self, word: str):
        pass
    
    # This is helpful if you are sending this to a TTS node
    def on_sentence(self, sentence: str):
        pass
        
    def on_result(self, result: str):
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
            future.add_done_callback(self.on_response, feedback_callback=self.on_feedback)
        except Exception as e:
            self.get_logger().error(f'Error in inference done: {e}')
    
    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.level == 0:
            self.on_chunk(feedback.chunk)
        elif feedback.level == 1:
            self.on_word(feedback.word)
        elif feedback.level == 2:
            self.on_sentence(feedback.sentence)
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
    node = Distiller()
    rclpy.spin(node)

if __name__ == '__main__':
    main()