import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from create_msgs.msg import DefineSong, PlaySong

class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        self.say(String(data='Hello, I am R1! Just a moment while I wake up.'))
        
    def say(self, msg):
        self.get_logger().debug('Saying: %s' % msg.data)
        self.voice.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    boot_announcer = BootAnnouncer()
    rclpy.spin(boot_announcer)
    boot_announcer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()