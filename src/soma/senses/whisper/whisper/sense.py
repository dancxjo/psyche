import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
from pete_interfaces.msg import Sensation
import uuid
import datetime

class AuditoryComprehension(Node):
    def __init__(self):
        super().__init__('auditory_comprehension')        
        self.transcript_sub = self.create_subscription(
            String, '/audio/transcription', self.publish_transcription, 10
        )
        self.segment_sub = self.create_subscription(
            ByteMultiArray, '/audio/segmented', self.publish_segment, 10
        )
        self.sensation_pub = self.create_publisher(
            Sensation, '/sensation', 10
        )
        self.current_audio_id = uuid.uuid4()  # Generate a unique identifier for the audio event

    def publish_transcription(self, transcription):
        iso_now = datetime.datetime.now().isoformat()
        sensation = Sensation()
        sensation.sensor_type = "STT Transcription"
        sensation.model = "OpenAI Whisper"
        sensation.timestamp = iso_now
        sensation.data_source = "/audio/segmented"
        sensation.capabilities = "Transcription capability"
        sensation.data = transcription.data
        sensation.data_description = "Transcribed spoken words"
        sensation.reliability = "Low"
        sensation.processing_notes = "Matches segment with segment id = " + str(self.current_audio_id)
        self.sensation_pub.publish(sensation)

    def publish_segment(self, segment):
        iso_now = datetime.datetime.now().isoformat()        
        sensation = Sensation()
        sensation.sensor_type = "Audio segmenter"
        sensation.model = "OpenAI Whisper"
        sensation.data_source = "onboard microphone"
        sensation.capabilities = "Audio processing"
        sensation.data = "(audio data)"  # Placeholder for actual binary data
        sensation.data_format = "wav"
        sensation.timestamp = iso_now
        sensation.data_description = "Segment of audio"
        sensation.processing_notes = "To be transcribed with segment id = " + str(self.current_audio_id)
        self.sensation_pub.publish(sensation)

    def update_audio_id(self):
        self.current_audio_id = uuid.uuid4()  # Update ID for a new audio event

def main(args=None):
    rclpy.init(args=args)
    node = AuditoryComprehension()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
