import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        self.say(String(data='Hello, I am R1! Just a moment while I wake up.'))
        self.timer = self.create_timer(30.0, self.timer_callback)
        self.times = 1
        
    def say(self, msg):
        self.get_logger().debug('Saying: %s' % msg.data)
        self.voice.publish(msg)
        
    def timer_callback(self):
        self.get_logger().info('Waking up...')
        self.say(String(data=f'I\'ve already announced this {self.times} times!'))
        self.times += 1

def main(args=None):
    rclpy.init(args=args)
    boot_announcer = BootAnnouncer()
    rclpy.spin(boot_announcer)
    boot_announcer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()