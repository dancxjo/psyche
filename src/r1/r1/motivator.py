import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist

class Motivator(Node):
    def __init__(self):
        super().__init__('motivator')
        self.subscription = self.create_subscription(
            String,
            'twists',
            self.twists_callback,
            10
        )
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def twists_callback(self, msg):
        self.get_logger().debug('Received twists: %s' % msg.data)
        # Process the received script here
        def process_twists(self, script):
            script_array = json.loads(script)
            ## TODO: Allow for a delay between each twist
            for twist in script_array:
                twist_msg = Twist()
                try:
                    twist_msg.linear.x = float(twist['linear']['x'])
                    twist_msg.linear.y = float(twist['linear']['y'])
                    twist_msg.linear.z = float(twist['linear']['z'])
                    twist_msg.angular.x = float(twist['angular']['x'])
                    twist_msg.angular.y = float(twist['angular']['y'])
                    twist_msg.angular.z = float(twist['angular']['z'])
                    self.publisher_cmd_vel.publish(twist_msg)
                except KeyError as e:
                    self.get_logger().error(f'Missing key in twist data: {e}')
                    continue
                except Exception as e:
                    self.get_logger().error(f'Failed to publish twist: {e}')
                    continue
                
        # Call the process_script function in script_callback
        process_twists(self, msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Motivator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()