import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from psyche_interfaces.action import PlainTextInference, InferenceWithImages


class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        self.say('Hello, I am R1! Just a moment while I wake up.')
        self.timer = self.create_timer(30.0, self.timer_callback)
        self.times = 1
        
        self.act1 = ActionClient(self, PlainTextInference, 'instruct')
        self.act2 = ActionClient(self, InferenceWithImages, 'inspect')
        
        self.say('Waiting for instruct and inspect action servers...')
        self.act1.wait_for_server()
        self.say('Instruct action server is ready!')
        self.act2.wait_for_server()
        self.say('Inspect action server is ready!')
        
    def say(self, msg):
        self.get_logger().info('Saying: %s' % msg)
        self.voice.publish(String(data=msg))
        
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