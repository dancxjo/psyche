import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from psyche_interfaces.action import PlainTextInference, InferenceWithImages


class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        
        # Wait for all the services to complete initialization
        self.act1 = ActionClient(self, PlainTextInference, 'instruct')
        self.act2 = ActionClient(self, InferenceWithImages, 'inspect')
        
        self.say('Waiting for instruct and inspect action servers...')
        self.act1.wait_for_server(timeout_sec=0)
        self.say('Instruct action server is ready!')
        self.act2.wait_for_server()
        self.say('Inspect action server is ready!')

        self.topics = {}
    
        for t in ['bumper', 'sensation', 'proprioception', 'twists', 'instant', 'identity', 'situation', 'intent', 'autobiography', 'feeling', 'song', 'imu']:
            self.topics[t] = self.create_subscription(String, t, self.make_callback(t), 10)

    def make_callback(self, topic):
        def callback(msg):
            #self.say(f'The {topic} topic has been initialized and is receiving data.')
            self.destroy_subscription(self.topics[topic])
            del self.topics[topic]
            if len(self.topics) == 0:
                self.finish()
        return callback

    def finish(self):
        self.say('Robot 1 initialization complete.')

    def say(self, msg):
        self.get_logger().info('Saying: %s' % msg)
        self.voice.publish(String(data=msg))
        
def main(args=None):
    rclpy.init(args=args)
    boot_announcer = BootAnnouncer()
    rclpy.spin(boot_announcer)
    boot_announcer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
