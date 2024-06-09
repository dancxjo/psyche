import rclpy
from .sense import Sense
from std_msgs.msg import String
import datetime

class Heartbeat(Sense):
    '''The heartbeat is the canonical sense. It reports the current time on a regular interval'''
    def __init__(self):
        super().__init__(
            'heartbeat',
            update_interval=60.0,
            input_topics=[],
            input_types=[],
        )
        
    def publish_accumulation(self):
        new_sensation = String(data=f"It is {datetime.datetime.now().isoformat()} and you are alive.") # call inxs
        self.get_logger().debug(f"Publishing heartbeat: {new_sensation.data}")
        self.publisher.publish(new_sensation)

def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
