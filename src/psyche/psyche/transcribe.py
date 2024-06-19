import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import ByteMultiArray, String
from speech_recognition import AudioData
import io
import threading

class Transcriber(Node):
    def __init__(self):
        super().__init__('audio_transcriber')
        
        self.declare_parameters(namespace='',
                        parameters=[
                            ('model', 'small'),
                            ('language', 'english')
                        ])

        self.segments_to_transcribe = []
        self.segment_sub = self.create_subscription(ByteMultiArray, 'audio/segmented', self.queue_segment, 10)
        self.transcription_pub = self.create_publisher(String, 'sensation', 10)
        self.recognizer = sr.Recognizer()
        self.get_logger().debug('Audio transcriber node started')
        
        # Set up a timer to process segments every second (or other suitable interval)
        self.timer = self.create_timer(1.0, self.start_transcribing)

    def start_transcribing(self):
        if self.segments_to_transcribe:
            self.get_logger().debug('Transcribing segment')
            segment = self.segments_to_transcribe.pop(0)
            self.get_logger().debug(f'Processing audio')
            self.process_audio(segment)
            self.get_logger().debug('Finished transcribing segment')
    
    def queue_segment(self, msg):
        self.get_logger().debug('Queuing segment')
        self.segments_to_transcribe.append(msg.data)
        self.get_logger().debug(f'Segment queued {len(self.segments_to_transcribe)}')

    def transcribe_in_all_the_ways(self, audio):
        recognizer = [self.recognizer.recognize_google, self.recognizer.recognize_whisper]
        for rec in recognizer:
            def threaded_recognition(rec, audio):
                try:
                    transcription = rec(audio)
                    self.get_logger().debug(f"Transcription: {transcription}")
                    self.publish_transcription(transcription)
                except Exception as e:
                    self.get_logger().error(f"Failed to transcribe audio: {str(e)}")
            #threading.Thread(target=threaded_recognition, args=(rec, audio)).start()
            threaded_recognition(rec, audio)

    def transcribe_audio_whisper(self, audio, language, model):
        try:
            transcription = self.recognizer.recognize_whisper(audio, language=language, model=model)
            self.get_logger().debug(f"Transcription: {transcription}")
            if transcription.strip() != "":
                self.publish_transcription(transcription)
        except Exception as e:
            self.get_logger().error(f"Failed to transcribe audio: {str(e)}")

    def process_audio(self, audio_data_list):
        self.get_logger().debug(f'Processing audio {type(audio_data_list)}')

        language = self.get_parameter('language').get_parameter_value().string_value
        model = self.get_parameter('model').get_parameter_value().string_value
        
        # Convert list of byte tuples to a flat list of bytes
        flat_list = [b for byte_tuple in audio_data_list for b in byte_tuple]
        
        # Convert flat list of bytes to a single bytes object
        byte_buffer = bytes(flat_list)
        
        # Now create a BytesIO object from the bytes
        audio_buffer = io.BytesIO(byte_buffer)
        audio_buffer.seek(0)  # Rewind the buffer to the start

        # Use the buffer with speech_recognition
        try:
            with sr.AudioFile(audio_buffer) as source:
                audio = self.recognizer.record(source)
                threading.Thread(target=self.transcribe_audio_whisper, args=(audio, language, model)).start()
                # self.transcribe_in_all_the_ways(audio)
        except Exception as e:
            self.get_logger().error(f"Failed to transcribe audio: {str(e)}")
        
    def publish_transcription(self, transcription):
        self.get_logger().debug('Publishing transcription')
        self.transcription_pub.publish(String(data=f"I could be mistaken, but I think I heard someone say earlier: '{transcription}'"))

def main(args=None):
    rclpy.init(args=args)
    node = Transcriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
