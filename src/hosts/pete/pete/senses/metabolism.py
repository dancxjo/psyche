import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from create_msgs.msg import ChargingState  # Adjust this import based on the actual package name
from psyche_interfaces.msg import Sensation
from sense import Sense

class MetabolismSense(Sense):
    """
    MetabolismSense handles monitoring and reporting the status of the robot's battery system,
    including capacity, charge, charge ratio, charging state, current, temperature, and voltage.
    """
    def __init__(self):
        super().__init__()

        # Override default sensor ID and set up specific input topics and message types
        self.set_parameters([
            rclpy.node.Parameter('sensor_id', rclpy.Parameter.Type.STRING, 'battery_sensor'),
            rclpy.node.Parameter('input_topics', rclpy.Parameter.Type.STRING_ARRAY, [
                '/battery/capacity', '/battery/charge', '/battery/charge_ratio',
                '/battery/charging_state', '/battery/current', '/battery/temperature', '/battery/voltage']),
            rclpy.node.Parameter('input_types', rclpy.Parameter.Type.STRING_ARRAY, [
                'std_msgs.msg.Float32', 'std_msgs.msg.Float32', 'std_msgs.msg.Float32',
                'create_msgs.msg.ChargingState', 'std_msgs.msg.Float32', 'std_msgs.msg.Int16', 'std_msgs.msg.Float32']),
            rclpy.node.Parameter('reliability', rclpy.Parameter.Type.STRING, 'high'),
            rclpy.node.Parameter('processing_notes', rclpy.Parameter.Type.STRING, 'Monitor battery status and health metrics')
        ])
        self.setup_subscribers()

    def process_message(self, msg, topic):
        """
        Extract data from messages and generate descriptive sensations based on the type of battery information received.
        """
        description = f"{topic.split('/')[-1].replace('_', ' ').capitalize()}: {msg.data}"
        self.publish_sensation(description)

    def publish_sensation(self, description):
        """
        Publish a formatted sensation including detailed battery metrics.
        """
        sensation_msg = Sensation()
        sensation_msg.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        sensation_msg.for_topic = ", ".join(self.input_topics)
        sensation_msg.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        sensation_msg.data = description
        sensation_msg.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        self.publisher.publish(sensation_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    metabolism_sense_node = MetabolismSense()
    rclpy.spin(metabolism_sense_node)
    metabolism_sense_node.destroy_node()
    rclpy.shutdown()
