### A simple ROS2 node that reads in text from the developer and outputs it to /sensation, the topic that the psyche listens to.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import json

form_html = b'''
            <html>
                <body>
                    <form method="POST">
                        <input type="text" name="text" autofocus />
                        <button type="submit">Send.</button>
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
        self.publish_sensation(text)
    
    def publish_sensation(self, text):
        msg = String(data=text)
        self.server.node.sensation_publisher.publish(String(data=f"You hear something on your special developer message channel: {msg}"))
        self.server.node.get_logger().info(f'Published sensation: {msg.data}')

class SimpleHTTPServer(HTTPServer):
    def __init__(self, server_address, RequestHandlerClass, node):
        super().__init__(server_address, RequestHandlerClass)
        self.node = node

class SimpleHttpInputServer(Node):
    '''Creates and manages an http server that serves up /say, a POST endpoint that accepts a string and publishes it to /sensation. The corresponding GET endpoint serves up a simple form for testing.'''
    def __init__(self):
        super().__init__('simple_http_input_server')
        self.sensation_publisher = self.create_publisher(String, 'sensation', 10)
        self.declare_parameters(namespace='', parameters=[
            ('port', 8100),
            ('host', '0.0.0.0')
        ])
        self.create_http_server()
        self.get_logger().info('Ready to accept input')
    
    def create_http_server(self):
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.server = SimpleHTTPServer((host, port), SimpleHTTPRequestHandler, self)
        
    def run(self):
        self.get_logger().info('Starting server...')
        self.server.serve_forever()
        
def main(args=None):
    rclpy.init(args=args)
    server = SimpleHttpInputServer()
    server.run()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    