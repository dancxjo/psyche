import os
import tempfile
import socket
import threading
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray

class TextStreamServer:
    """
    This class manages a TCP server for streaming the voice as text to clients who will then handle tts themselves.
    """
    def __init__(self, host='0.0.0.0', port=8200):
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
                    self.stream_text(data) 
                else:
                    raise ConnectionResetError("Client disconnected")
            except Exception as e:
                print(f"Connection lost with {client_socket}: {e}")
                break
        self.clients.remove(client_socket)
        client_socket.close()

    def stream_text(self, data):
        """
        Streams audio data to all connected clients.
        """
        for client in self.clients[:]:  # Make a copy of the list
            try:
                client.sendall(f"{data}\n")
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

class VoiceNode(Node):
    """
    ROS Node for streaming the voice as text over TCP.
    """
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameters(namespace='',
                                parameters=[
                                ])
        self.subscription = self.create_subscription(String, 'voice', self.voice_callback, 10)
        self.server = TextStreamServer()
        threading.Thread(target=self.server.accept_connections, daemon=True).start()

    def voice_callback(self, msg):
        try:
            self.server.stream_text(msg.data)

        except Exception as e:
            self.get_logger().error(f"Error during voice publishing: {e}")

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()
    signal.signal(signal.SIGINT, lambda sig, frame: voice_node.server.close())
    signal.signal(signal.SIGTERM, lambda sig, frame: voice_node.server.close())
    try:
        rclpy.spin(voice_node)
    finally:
        voice_node.server.close()
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
