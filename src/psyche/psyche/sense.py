import rclpy
from rclpy.node import Node
from psyche_interfaces.msg import Sensation
from std_msgs.msg import String
from rclpy.action import ActionClient
from psyche_interfaces.action import PlainTextInference

class Sense(Node):
    """
    A sense subscribes to one or more sensors and publishes processed sensations on a timer and/or when a change is detected.
    """
    def __init__(self):
        super().__init__('generic_sensor')
        self.declare_parameters('', [
            ('sensor_id', 'default_sensor'),
            ('reliability', 'unrated'),
            ('processing_notes', ''),
            ('input_topics', ['thought'],),
            ('input_types', ['std_msgs.msg.String']),
            ('output_topic', 'sensation'),
            ('update_interval', 0.0), # 0 means changes report immediately,
            ('accumulation_method', 'latest'),
        ])

        # Retrieve parameters
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.input_types = self.get_parameter('input_types').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value

        # Setup communication
        self.last_readings = {}
        self.accumulated_readings = {}
        self.queued_readings = {}
        self.subscribers = []
        self.setup_subscribers()

        self.publisher = self.create_publisher(Sensation, self.output_topic, 10)
    
        # The plain distiller uses the uninformed instruct llm
        self.distill = ActionClient(self, PlainTextInference, 'instruct')
        # while the considered distiller uses the rag assisted "informed" llm
        self.consider = ActionClient(self, PlainTextInference, 'recall')
    
        # An opt out for the timer if the update interval is set to 0    
        if self.update_interval > 0:
            self.timer = self.create_timer(self.update_interval, self.publish_accumulation)

    def setup_subscribers(self):
        for topic, msg_type in zip(self.input_topics, self.input_types):
            msg_class = self.get_message_class(msg_type)
            sub = self.create_subscription(msg_class, topic, self.create_callback(topic), 10)
            self.subscribers.append(sub)

    def get_message_class(self, msg_type):
        module_name, class_name = msg_type.rsplit('.', 1)
        module = __import__(module_name, fromlist=[class_name])
        return getattr(module, class_name)

    def create_callback(self, topic):
        def callback(msg):
            self.process_message(msg, topic)
        return callback

    def format_message(self, msg, topic) -> Sensation:
        idx = self.input_topics.index(topic)
        mt = self.input_types[idx]
        data = str(msg)
        s = Sensation()
        s.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        s.for_topic = topic
        s.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        s.data = data
        s.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        return s

    def queue_reading(self, msg, topic):
        if topic not in self.queued_readings:
            self.queued_readings[topic] = []
        self.queued_readings[topic].append(msg)

    def average_reading(self, msg, topic):
        self.queue_reading(msg, topic)
        self.accumulated_readings[topic] = sum(self.queued_readings[topic]) / len(self.queued_readings[topic])
    
    def latest_reading(self, msg, topic):
        self.accumulated_readings[topic] = msg
        
    def first_reading(self, msg, topic):
        if topic not in self.accumulated_readings:
            self.accumulated_readings[topic] = msg

    def accumulate(self, msg, topic):        
        method = self.get_parameter('accumulation_method').get_parameter_value().string_value
        
        if method == 'queue':
            self.queue_reading(msg, topic)
        elif method == 'average':
            self.average_reading(msg, topic)
        elif method == 'latest':
            self.latest_reading(msg, topic)
        elif method == 'first':
            self.first_reading(msg, topic)
        elif method == 'distilled':
            self.queue_reading(msg, topic)
        elif method == 'considered':
            self.queue_reading(msg, topic)
        else:
            self.get_logger().error(f"Unknown accumulation method: {method}")
        
    def process_message(self, msg, topic):
        if self.update_interval == 0.0:
            # Report immediately
            self.get_logger().info(f"Reporting immediately: {msg}")
            self.publish_sensation(topic, msg)
            return
        
        self.last_readings[topic] = msg
        self.accumulate(msg, topic)
        
    def summarize(self, distiller, topic, accumulation):
        goal_msg = PlainTextInference.Goal()
        goal_msg.prompt = f"""These are a set of recent sensor readings from {topic}.
        
        {accumulation}
        
        Can you make sense of them?
        """
        future = distiller.send_goal_async(goal_msg)
        future.add_done_callback(self.make_callback(topic))

    def make_callback(self, topic): 
        def callback(future):
            return self.on_summary(topic, future.result().response)
        return callback
 
    def on_summary(self, topic, summary):
        sensation = Sensation() # I wish python used the new keyword :(
        sensation.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        sensation.for_topic = topic
        sensation.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        sensation.data = summary
        sensation.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        
        self.publish_sensation(topic, sensation)

    def process_accumulation(self, topic, accumulation):
        method = self.get_parameter('accumulation_method').get_parameter_value().string_value
        if method == 'distilled' or method == 'considered':
            distiller = self.distill if method == 'distilled' else self.consider
            self.summarize(distiller, topic, accumulation)        
            distiller.wait_for_server()
        else:
            self.publish_sensation(topic, accumulation)

    def clear_accumulations(self):
        '''Hook to maybe cache the last accumulation for a while before clearing it.'''
        self.accumulated_readings = {}

    def publish_accumulation(self):
        self.get_logger().info(f"Publishing accumulated sensations")
        for topic, accumulation in self.accumulated_readings.items():
            self.process_accumulation(topic, accumulation)
        self.clear_accumulations()

    def publish_sensation(self, topic, reading):
        self.get_logger().info(f"Publishing sensation: {reading}")
        # Format the message as a string or another type depending on your application
        msg = self.format_message(reading, topic)
        self.get_logger().info(f"Publishing sensation: {msg}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Sense()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
