import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import requests
import io
from pydub import AudioSegment
import subprocess
import tempfile
import os

class TTSNode(Node):
    """
    ROS Node for text-to-speech processing and streaming the audio over TCP.
    """
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('voice_id', 'p317'),
                                    ('coqui_base_url', "http://192.168.0.7:5002")
                                ])
        self.subscription = self.create_subscription(String, 'voice', self.voice_callback, 10)
        self.publisher = self.create_publisher(ByteMultiArray, 'streamed_voice', 10)
        self.voice_id = self.get_parameter('voice_id').get_parameter_value().string_value
        self.coqui_base_url = self.get_parameter('coqui_base_url').get_parameter_value().string_value
        self.playlist = []
        self.timer = self.create_timer(1, self.handle_queue)
        
    def voice_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received text: {text}")
        wav = self.get_wav(text)
        self.playlist.append(wav)
        # self.publisher.publish(ByteMultiArray(data=wav.))
        self.handle_queue()
    
    def handle_queue(self):
        if len(self.playlist) == 0:
            return
        wav = self.playlist.pop(0)
        padded_wav = AudioSegment.silent(duration=100) + wav + AudioSegment.silent(duration=100)
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            padded_wav.export(temp_file.name, format="wav")
            temp_file_path = temp_file.name

        # Use ffplay to play the temporary file
        subprocess.run(["aplay", temp_file_path])

        # Delete the temporary file
        os.unlink(temp_file_path)
        
    def get_wav(self, text):
        # "style_wav": "", # TODO: Record some "emoticons" so robot can express feelings
        # "language_id": "" # TODO: Let robot change languages
        response = requests.get(f"{self.coqui_base_url}/api/tts", params={
                                    "text": text,
                                    "speaker_id": self.voice_id,
                                    "style_wav": "",
                                    "language_id": ""
                                }
                            )

        if response.status_code == 200:
            # Replace the silence with the received audio
            received_audio = AudioSegment.from_file(io.BytesIO(response.content), format="wav")
            return received_audio
        else:
            self.get_logger().error("Failed to fetch audio")
            return None


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    try:
        rclpy.spin(tts_node)
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
