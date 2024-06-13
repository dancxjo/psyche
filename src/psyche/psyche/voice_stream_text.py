import os
import socket
import threading
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceNode(Node):
    """
    ROS Node for streaming voice as text over TCP.
    """
    def __init__(self, host='0.0.0.0', port=8200):
        super().__init__('tts_node')
        self.host = host
        self.port = port
        self.tcp_clients = []
        self.running = True

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

        self.subscription = self.create_subscription(String, 'voice', self.voice_callback, 10)

        threading.Thread(target=self.accept_connections, daemon=True).start()
        print(f"Streaming server listening on {self.host}:{self.port}")

    def accept_connections(self):
        while self.running:
            try:
                client_socket, addr = self.server_socket.accept()
                self.tcp_clients.append(client_socket)
                print('Connected by', addr)
            except OSError as e:
                if self.running:
                    print(f"Error accepting connections: {e}")
                break

    def voice_callback(self, msg):
        data = str(msg.data).strip()
        if data:
            for client in self.tcp_clients[:]:  # Make a copy of the list to avoid modification during iteration
                try:
                    client.sendall(data.strip())
                except Exception as e:
                    print(f"Failed to send data to {client}: {e}")
                    self.tcp_clients.remove(client)
                    client.close()

    def close(self):
        print("Shutting down server...")
        self.running = False
        self.server_socket.close()
        for client in self.tcp_clients:
            client.close()
        self.tcp_clients.clear()
        print("Server shut down successfully.")

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceNode()
    signal.signal(signal.SIGINT, lambda sig, frame: voice_node.close())
    signal.signal(signal.SIGTERM, lambda sig, aframe: voice_node.close())
    try:
        rclpy.spin(voice_node)
    finally:
        voice_node.close()
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
