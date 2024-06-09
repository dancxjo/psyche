import rclpy
from .sense import Sense
from psyche_interfaces.msg import Sensation
import datetime

class Heartbeat(Sense):
    '''The heartbeat is the canonical sense. It reports the current time on a regular interval'''
    def __init__(self):
        super().__init__(
            'heartbeat',
            sensor_id='heartbeat',
            reliability='high',
            processing_notes='',
            update_interval=60.0,
            input_topics=[],
            input_types=[],
        )
        
    def publish_accumulation(self):
        new_sensation = Sensation() # call inxs
        new_sensation.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        new_sensation.for_topic = ''
        new_sensation.reliability = self.get_parameter('reliability').get_parameter_value().string_value
        new_sensation.data = f"It is {datetime.datetime.now().isoformat()} and you are alive."
        new_sensation.processing_notes = self.get_parameter('processing_notes').get_parameter_value().string_value
        new_sensation.timestamp = datetime.datetime.now().isoformat()
        self.get_logger().info(f"Publishing heartbeat: {new_sensation.data}")
        self.publisher.publish(new_sensation)

def main(args=None):
    rclpy.init(args=args)
    node = Sense()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
