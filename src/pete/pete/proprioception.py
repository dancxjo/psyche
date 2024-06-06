import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from create_msgs.msg import Bumper, Cliff
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from psyche_interfaces.msg import Sensation
from sense import Sense

class ProprioceptionSense(Sense):
    """
    ProprioceptionSense handles and reports the internal state of the robot related to its
    physical interaction with the environment and its own body status.
    """
    def __init__(self):
        super().__init__()

        # Override default sensor ID and set up specific input topics and message types
        self.set_parameters([
            rclpy.node.Parameter('sensor_id', rclpy.Parameter.Type.STRING, 'proprioception_sensor'),
            rclpy.node.Parameter('input_topics', rclpy.Parameter.Type.STRING_ARRAY, [
                'bumper', 'cliff', 'joint_states', 'odom', 'wheeldrop', 'tf']),
            rclpy.node.Parameter('input_types', rclpy.Parameter.Type.STRING_ARRAY, [
                'create_msgs.msg.Bumper', 'create_msgs.msg.Cliff', 'sensor_msgs.msg.JointState',
                'nav_msgs.msg.Odometry', 'std_msgs.msg.Empty', 'tf2_msgs.msg.TFMessage']),
            rclpy.node.Parameter('reliability', rclpy.Parameter.Type.STRING, 'high'),
            rclpy.node.Parameter('processing_notes', rclpy.Parameter.Type.STRING, 'Handles proprioceptive data for self-awareness')
        ])
        self.setup_subscribers()

    def process_message(self, msg, topic):
        """
        Handle messages from various sensors and formats them into sensations.
        """
        if isinstance(msg, Empty) and topic == 'wheeldrop':
            description = "Wheel drop detected"
        else:
            description = f"{topic.split('')[-1].replace('_', ' ').capitalize()} data received"
        
        self.publish_sensation(description)

    def publish_sensation(self, description):
        """
        Publish a formatted sensation based on the sensor data received.
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
    proprioception_sense_node = ProprioceptionSense()
    rclpy.spin(proprioception_sense_node)
    proprioception_sense_node.destroy_node()
    rclpy.shutdown()
