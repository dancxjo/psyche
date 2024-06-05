import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from psyche_interfaces.msg import Sensation
from std_msgs.msg import String

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
            ('input_topics', []),
            ('input_types', []),
            ('output_topic', '/sensation'),
            ('update_interval', 0),
            ('threshold', 1.0)
        ])

        # Retrieve parameters
        self.input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.input_types = self.get_parameter('input_types').get_parameter_value().string_array_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        # Setup communication
        self.last_readings = {}
        self.subscribers = []
        self.setup_subscribers()

        self.publisher = self.create_publisher(Sensation, self.output_topic, 10)
    
        # An opt out for the timer if the update interval is set to 0    
        if self.update_interval > 0:
            self.timer = self.create_timer(self.update_interval, self.publish_sensations)

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

    def detect_changes(self, current_reading, topic):
        '''Compare the current reading to the last one to detect if the change is significant enough to publish outside the timer loop'''
        if topic not in self.last_readings:
            return True
        return abs(current_reading - self.last_readings[topic]) > self.threshold

    def process_message(self, msg, topic):
        pass

    def publish_sensations(self):
        for topic, reading in self.last_readings.items():
            self.publish_sensation(topic, reading)

    def publish_sensation(self, topic, reading):
        # Format the message as a string or another type depending on your application
        msg = String(data=f"{self.get_parameter('sensor_id').get_parameter_value().string_value}: {reading}")
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Sense()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
