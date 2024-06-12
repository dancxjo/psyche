import os
import io
import tempfile
import socket
import threading
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
# from TTS.api import TTS
import torch
from gtts import gTTS
import subprocess

class AudioStreamServer:
    """
    This class manages a TCP server for streaming vocalized speech from /voice to clients.
    """
    def __init__(self, host='0.0.0.0', port=8000):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.clients = []
        self.running = True
        threading.Thread(target=self.accept_connections, daemon=True).start()
        # TODO: Assign to an actual logger
        print(f"Streaming server listening on {self.host}:{self.port}")

    def accept_connections(self):
        """
        Accepts new connections and handles them with a separate thread.
        """
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                self.clients.append(client_socket)
                # TODO: Assign to an actual logger
                print('Connected by', addr)
                threading.Thread(target=self.handle_client, args=(client_socket,), daemon=True).start()
            except OSError as e:
                if self.running:
                    # TODO: Assign to an actual logger
                    print(f"Error accepting connections: {e}")
                break

    def handle_client(self, client_socket):
        """
        Handles the client by reading data and managing the stream.
        """
        while True:
            try:
                data = client_socket.recv(1024)
                if data:
                    self.stream_audio(data) 
                else:
                    raise ConnectionResetError("Client disconnected")
            except Exception as e:
                print(f"Connection lost with {client_socket}: {e}")
                break
        self.clients.remove(client_socket)
        client_socket.close()

    def stream_audio(self, data):
        """
        Streams audio data to all connected clients.
        """
        for client in self.clients[:]:  # Make a copy of the list
            try:
                client.sendall(data)
            except Exception as e:
                
                print(f"Failed to send data to {client}: {e}")
                self.clients.remove(client)
                client.close()
                        
    def close(self):
        """
        Closes the server and all client connections cleanly.
        """
        print("Shutting down server...")
        self.running = False
        self.server_socket.close()
        for client in self.clients:
            client.close()
        self.clients.clear()
        print("Server shut down successfully.")

class TTSNode(Node):
    """
    ROS Node for text-to-speech processing and streaming the audio over TCP.
    """
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('voice_file', '/usr/local/share/tts/voices/pete.wav'),
                                    ('language', 'en'),
                                    ('model_path', '/usr/local/share/tts/models/xtts_v2'),
                                    ('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
                                    ('chunk_size', 1024),
                                ])
        self.subscription = self.create_subscription(String, 'voice', self.voice_callback, 10)
        self.publisher = self.create_publisher(ByteMultiArray, 'streamed_voice', 10)
        self.model = self.load_model()
        self.audio_server = AudioStreamServer()
        threading.Thread(target=self.audio_server.accept_connections, daemon=True).start()

    def load_model(self):
        """
        Loads the TTS model from the specified path.
        """
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        device = self.get_parameter('device').get_parameter_value().string_value
        return "One day we might be able to get coqui-tts running"
        # return TTS(model_path).to(device)

    def voice_callback(self, msg):
        """
        Callback function for voice commands. Processes the text and streams the resulting audio.
        """
        self.get_logger().debug('Received voice command: "%s"' % msg.data)
        self.process_speech(msg.data)

    def process_speech(self, text):
        """
        Converts text to speech, saves it to a temporary file, and streams the audio.
        """
        voice_file = self.get_parameter('voice_file').get_parameter_value().string_value
        language = self.get_parameter('language').get_parameter_value().string_value
        file_path = tempfile.mktemp(suffix='.mp3')
        try:
            tts = gTTS(text)
            tts.save(file_path)
            wav_file_path = file_path.replace('.mp3', '.wav')
            subprocess.run(['ffmpeg', '-i', file_path, wav_file_path], check=True)

            # self.model.tts_to_file(text, file_path=file_path, speaker_wav=[voice_file], language=language)
            self.publish_audio(wav_file_path)
        finally:
            if os.path.exists(file_path):
                os.remove(file_path)  # Ensure cleanup regardless of errors

    def publish_audio(self, file_path):
        try:
            self.publisher.publish(ByteMultiArray(data=b'\x00' * chunk_size))
            self.audio_server.stream_audio(b'\x00' * chunk_size)
            with open(file_path, 'rb') as file:
                wav_data = file.read()  # Read the entire WAV file
            chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
            # Stream the entire WAV data in chunks
            for i in range(0, len(wav_data), chunk_size):
                chunk = wav_data[i:i+chunk_size]
                if not chunk:
                    break
                self.publisher.publish(ByteMultiArray(data=chunk))
                self.audio_server.stream_audio(chunk)

        except Exception as e:
            self.get_logger().error(f"Error during audio publishing: {e}")

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    signal.signal(signal.SIGINT, lambda sig, frame: tts_node.audio_server.close())
    signal.signal(signal.SIGTERM, lambda sig, frame: tts_node.audio_server.close())
    try:
        rclpy.spin(tts_node)
    finally:
        tts_node.audio_server.close()
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
