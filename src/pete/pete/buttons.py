import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from psyche_interfaces.msg import Sensation
from psyche import Sense

class ButtonSense(Sense):
    """
    ButtonSense detects and processes button press events for 'clean' and 'dock' buttons on a robotic platform.
    It uses the base class Sense to manage ROS 2 node initialization and message handling but focuses
    on empty messages that signify button presses.
    """
    def __init__(self):
        super().__init__()

        # Override default sensor ID and setup specific input topics and message types
        self.set_parameters([
            rclpy.node.Parameter('sensor_id', rclpy.Parameter.Type.STRING, 'button_sensor'),
            rclpy.node.Parameter('input_topics', rclpy.Parameter.Type.STRING_ARRAY, ['/clean_button', '/dock_button']),
            rclpy.node.Parameter('input_types', rclpy.Parameter.Type.STRING_ARRAY, ['std_msgs.msg.Empty', 'std_msgs.msg.Empty']),
            rclpy.node.Parameter('reliability', rclpy.Parameter.Type.STRING, 'high'),
            rclpy.node.Parameter('processing_notes', rclpy.Parameter.Type.STRING, 'No data payload, triggered by physical button press')
        ])
        self.setup_subscribers()

    def process_message(self, msg, topic):
        """
        Process the received message based on the topic to identify the action triggered by the button press.
        """
        if topic == '/clean_button':
            self.publish_sensation('clean')
        elif topic == '/dock_button':
            self.publish_sensation('dock')

    def publish_sensation(self, action):
        """
        Publish a formatted sensation indicating which button was pressed.
        """
        sensation_msg = Sensation()
        sensation_msg.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        sensation_msg.for_topic = ", ".join(self.input_topics)  # The button press topic
        sensation_msg.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        sensation_msg.data = f"{action} button pressed"
        sensation_msg.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        self.publisher.publish(sensation_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    button_sense_node = ButtonSense()
    rclpy.spin(button_sense_node)
    button_sense_node.destroy_node()
    rclpy.shutdown()
