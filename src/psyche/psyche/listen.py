import rclpy
from rclpy.node import Node
import speech_recognition as sr
from psyche_interfaces.msg import Sensation
from std_msgs.msg import ByteMultiArray
from datetime import datetime
from speech_recognition import AudioData

class AudioSegmenter(Node):
    def __init__(self):
        super().__init__('audio_segmenter')
        self.segment_publisher = self.create_publisher(ByteMultiArray, 'audio/segmented', 10)
        self.recognizer = sr.Recognizer()

        self.declare_parameters(namespace='',
                                parameters=[
                                    ('device_index', 3),
                                ])

        device_index = self.get_parameter('device_index').get_parameter_value().integer_value
        self.microphone = sr.Microphone(device_index=device_index)
        self.start_listening()

    def start_listening(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)  # Adjusts for ambient noise to reduce false positives
        self.listen_continuous()

    def listen_continuous(self):
        with self.microphone as source:
            while rclpy.ok():
                try:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=10.0)  # Listen for the first phrase and extract it into audio data
                    self.publish_segment(audio)
                except sr.WaitTimeoutError:
                    continue  # No speech was detected

    def publish_segment(self, audio):
        self.get_logger().debug('Publishing segmented audio')
        wav_data = audio.get_wav_data()

        byte_array_msg = ByteMultiArray()
        byte_array_msg.data = wav_data

        self.segment_publisher.publish(byte_array_msg)
        self.get_logger().debug(f'Published {len(byte_array_msg.data)} bytes of WAV audio data')

def main(args=None):
    rclpy.init(args=args)
    node = AudioSegmenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()