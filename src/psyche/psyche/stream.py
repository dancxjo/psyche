import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
from flask import Flask, Response, request
import requests
from pydub import AudioSegment
from pydub.generators import Sine
import io

app = Flask(__name__)

chunk_size = 1024

# Generate 10 minutes of silent audio at 440 Hz (adjust as needed)
silence = Sine(440).to_audio_segment(duration=1000*60*10).fade_in(50).fade_out(100)

def generate_stream(audio_segment):
    """ Generator function to continuously yield audio data """
    while True:
        # Simulate a continuous stream by looping the audio segment
        for chunk in audio_segment[::chunk_size]:  # Adjust chunk size as needed
            yield chunk.raw_data

@app.route('/stream')
def stream():
    """ Route to handle streaming audio """
    return Response(generate_stream(silence), mimetype="audio/wav")

def speak(text, publisher):
    # "style_wav": "", # TODO: Record some "emoticons" so robot can express feelings
    # "language_id": "" # TODO: Let robot change languages

    response = requests.get("http://192.168.0.7:5002/api/tts", params={
                                "text": text,
                                "speaker_id": voice_id,
                                "style_wav": "",
                                "language_id": ""
                            }
                        )

    if response.status_code == 200:
        # Replace the silence with the received audio
        received_audio = AudioSegment.from_file(io.BytesIO(response.content), format="wav")
        publisher.publish(ByteMultiArray(data=received_audio.raw_data))
        global silence
        silence = received_in_silence_overlay(received_audio, position=0)
        return "Audio updated", 200
    else:
        return "Failed to fetch audio", 500

def received_in_silence_overlay(received_audio, position=0):
    """ Overlay received audio on silence at specified position """
    return silence.overlay(received_audio, position=position)


class TTSNode(Node):
    """
    ROS Node for text-to-speech processing and streaming the audio over TCP.
    """
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('voice_id', 'p336'),
                                    ('chunk_size', 1024),
                                    ('port', 8000)
                                ])
        self.subscription = self.create_subscription(String, 'voice', self.voice_callback, 10)
        self.publisher = self.create_publisher(ByteMultiArray, 'streamed_voice', 10)
        self.voice_id = self.get_parameter('voice_id').get_parameter_value().string_value
        self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        global voice_id
        global chunk_size
        voice_id = self.voice_id
        chunk_size = self.chunk_size
        
        app.run(debug=True, threaded=True, host='0.0.0.0', port=self.port)
        
    def voice_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received text: {text}")
        speak(text, self.publisher)
        self.get_logger().info("Audio updated")
        self.get_logger().info("Streaming audio...")

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    try:
        rclpy.spin(tts_node)
    finally:
        tts_node.audio_server.close()
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
