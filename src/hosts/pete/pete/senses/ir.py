import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from psyche_interfaces.msg import Sensation
from sense import Sense

class IRSense(Sense):
    """
    IRSense detects and processes IR characters from an omnidirectional IR receiver.
    It uses the base class Sense to manage ROS 2 node initialization and message handling,
    focusing on UInt16 messages that signify IR characters.
    """
    def __init__(self):
        super().__init__()

        # Override default sensor ID and set up specific input topic and message type
        self.set_parameters([
            rclpy.node.Parameter('sensor_id', rclpy.Parameter.Type.STRING, 'ir_omni_sensor'),
            rclpy.node.Parameter('input_topics', rclpy.Parameter.Type.STRING_ARRAY, ['/ir_omni']),
            rclpy.node.Parameter('input_types', rclpy.Parameter.Type.STRING_ARRAY, ['std_msgs.msg.UInt16']),
            rclpy.node.Parameter('reliability', rclpy.Parameter.Type.STRING, 'medium'),
            rclpy.node.Parameter('processing_notes', rclpy.Parameter.Type.STRING, 'IR character data; 0 means no character received')
        ])
        self.setup_subscribers()

    def process_message(self, msg, topic):
        """
        Process the received UInt16 message to extract the IR character.
        """
        ir_character = msg.data
        if ir_character != 0:
            self.publish_sensation(f"IR character {ir_character} received")
        else:
            self.publish_sensation("No IR character received")

    def publish_sensation(self, description):
        """
        Publish a formatted sensation indicating what IR character was received.
        """
        sensation_msg = Sensation()
        sensation_msg.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        sensation_msg.for_topic = ", ".join(self.input_topics)  # The IR character topic
        sensation_msg.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        sensation_msg.data = description
        sensation_msg.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        self.publisher.publish(sensation_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    ir_sense_node = IRSense()
    rclpy.spin(ir_sense_node)
    ir_sense_node.destroy_node()
    rclpy.shutdown()
