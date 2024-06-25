import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from psyche_interfaces.action import PlainTextInference, InferenceWithImages

topics = {
    "voice": "Voice initialized.",
    # "bumper": "Platform initialized.",
    "sensation": "Sensation initialized.",
    # "proprioception": "Proprioception initialized.",
    # "twists": "Navigation initialized.",
    # "instant": "Sensory integration initialized.",
    # "identity": "Identity initialized.",
    "situation": "Combobulation initialized.",
    # "shell_output": "Receiving standard output.",
    # "autobiography": "Autobiography initialized.",
    # "feeling": "Emotions initialized.",
    # "song": "Cantation initialized.",
    # "imu": "Inertial measurement initialized.",
}

class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        self.say('Robot one initializing...')
        # Wait for all the services to complete initialization
        self.act1 = ActionClient(self, PlainTextInference, 'instruct')
        self.act2 = ActionClient(self, InferenceWithImages, 'inspect')
        
        self.say('Initializing ell emm yooz...')
        self.act1.wait_for_server(timeout_sec=0)
        self.say('Ell emm one initialized.')
        self.act2.wait_for_server()
        self.say('Ell emm two initialized.')

        self.topics = {}
    
        for t, msg in topics.items():
            self.topics[t] = self.create_subscription(String, t, self.make_callback(t, msg), 10)

    def make_callback(self, topic, m):
        def callback(msg):
            self.say(m)
            self.destroy_subscription(self.topics[topic])
            del self.topics[topic]
            if len(self.topics) == 0:
                self.finish()
        return callback

    def finish(self):
        self.say('Alright! Let\'s go!')

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
