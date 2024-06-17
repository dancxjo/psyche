import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from psyche_interfaces.action import PlainTextInference, InferenceWithImages

topics = {
    "voice": "Ahem.",
    "bumper": "Ouch!",
    "sensation": "Hmm.",
    "proprioception": "Whoo!",
    "twists": "Here we go!",
    "instant": "Aha!",
    "identity": "Oh!",
    "situation": "I see.",
    "shell_output": "Receiving standard output.",
    "autobiography": "That's it.",
    "feeling": "Ah!",
    "song": "Doe, a deer, a female deer.",
    "imu": "Whippee!"
}

class BootAnnouncer(Node):
    def __init__(self):
        super().__init__('boot_announcer')
        self.voice = self.create_publisher(String, 'voice', 10)
        self.say('Robot 1 initializing...')
        # Wait for all the services to complete initialization
        self.act1 = ActionClient(self, PlainTextInference, 'instruct')
        self.act2 = ActionClient(self, InferenceWithImages, 'inspect')
        
        self.say('Ready?')
        self.act1.wait_for_server(timeout_sec=0)
        self.say('Get set.')
        self.act2.wait_for_server()
        self.say('Go!')

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
