import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import json
import asyncio
import websockets
import threading

form_html = b'''
<!DOCTYPE html>
<html>
<head>
    <style>
        .container {
            display: grid;
            grid-template-columns: 1fr auto;
            gap: 10px;
            align-items: center;
        }
        .history {
            font-family: monospace;
            white-space: pre-wrap;
            overflow: auto;
            height: 200px;
            border: 1px solid black;
            padding: 10px;
        }
    </style>
    <script>
        document.addEventListener('DOMContentLoaded', (event) => {
            // TODO: Unhardcode
            const socket = new WebSocket('ws://192.168.0.13:8101');
            
            socket.onmessage = function(event) {
                const history = document.querySelector('.history');
                history.textContent += JSON.stringify(event.data) + '\\n';
            };

            const form = document.querySelector('form');
            form.addEventListener('submit', function(event) {
                event.preventDefault();
                const input = form.querySelector('input[name="text"]');
                const message = input.value;
                socket.send(message);
                input.value = ''; // Clear the input field after sending
            });
        });
    </script>
</head>
<body>
    <div class="history"></div>
    <form class="container">
        <input type="text" name="text" autofocus/>
        <button type="submit">Send</button>
    </form>
</body>
</html>
'''

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(form_html)

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        post_data = parse_qs(post_data.decode('utf-8'))
        text = post_data['text'][0]
        self.send_response(200)
        self.end_headers()
        self.wfile.write(form_html)
        self.server.node.publish_sensation(text)

class SimpleHTTPServer(HTTPServer):
    def __init__(self, server_address, RequestHandlerClass, node):
        super().__init__(server_address, RequestHandlerClass)
        self.node = node
        self.topic_queue = {}
        self.websocket_task = None

    async def handle_websocket(self, websocket, path):
        while True:
            message = await websocket.recv()
            self.node.publish_sensation(message)
            for topic, messages in self.topic_queue.items():
                for message in messages:
                    await websocket.send(message)

    async def start_websocket_server(self):
        host = self.node.get_parameter('host').get_parameter_value().string_value
        port = self.node.get_parameter('port').get_parameter_value().integer_value + 1
        async with websockets.serve(self.handle_websocket, host, port):
            self.node.get_logger().info(f'Started websocket server at {host}:{port}')
            await asyncio.Future()  # Run forever

    def run_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.websocket_task = loop.run_until_complete(self.start_websocket_server())
        loop.run_forever()

    def publish_event(self, topic, message):
        self.topic_queue[topic] = [] if not self.topic_queue[topic] else self.topic_queue[topic]
        self.topic_queue[topic].append(message)

class SimpleHttpInputServer(Node):
    def __init__(self):
        super().__init__('simple_http_input_server')
        self.sensation_publisher = self.create_publisher(String, 'sensation', 10)
        self.declare_parameters(namespace='', parameters=[
            ('port', 8100),
            ('host', '0.0.0.0')
        ])
        self.create_http_server()
        self.get_logger().info('Ready to accept input')
        self.subs = []
        for topic in ['voice', 'shell_commands', 'sensation']:
            self.subs.append(self.create_subscription(String, topic, lambda m: self.server.publish_event(topic, m), 10))

    def create_http_server(self):
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.server = SimpleHTTPServer((host, port), SimpleHTTPRequestHandler, self)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        threading.Thread(target=self.server.run_websocket_server, daemon=True).start()

    def publish_sensation(self, text):
        msg = String(data=text)
        self.sensation_publisher.publish(String(data=f"You hear something on your special developer message channel: {msg.data}"))
        self.get_logger().info(f'Published sensation: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    server = SimpleHttpInputServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
