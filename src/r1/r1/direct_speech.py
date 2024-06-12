import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import json

class DirectSpeech(Node):
    def __init__(self):
        super().__init__('direct_speaker')
        self.audio_stream = self.create_subscription(ByteMultiArray, 'streamed_voice', self.play, 2)
        
    def play(self, msg):
        self.get_logger().debug('Playing mp3 chunk')
        chunk = msg.data
        
        ## TODO: Use pydub to play the audio chunk

def main(args=None):
    rclpy.init(args=args)
    boot_announcer = BootAnnouncer()
    rclpy.spin(boot_announcer)
    boot_announcer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()