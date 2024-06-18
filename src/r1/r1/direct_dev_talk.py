import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs
import threading

form_html = b'''
            <html>
                <head>
                    <style>
                        .container {
                            display: grid;
                            grid-template-columns: 1fr auto;
                            gap: 10px;
                            align-items: center;
                        }
                    </style>
                </head>
                <body>
                    <form method="POST" class="container">
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
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(form_html)
        self.server.node.publish_sensation(text)
    
class SimpleHTTPServer(HTTPServer):
    def __init__(self, server_address, RequestHandlerClass, node):
        super().__init__(server_address, RequestHandlerClass)
        self.node = node

class SimpleHttpInputServer(Node):
    def __init__(self):
        super().__init__('simple_http_input_server')
        self.sensation_publisher = self.create_publisher(String, 'sensation', 10)
        self.declare_parameters(namespace='', parameters=[
            ('port', 8100),
            ('host', '0.0.0.0')
        ])
        self.get_logger().info('Ready to accept input')
        self.create_http_server()

    def publish_sensation(self, text):
        msg = String(data=f"You hear something on your special developer message channel: {text}")
        self.sensation_publisher.publish(msg)
        self.get_logger().info(f'Published sensation: {msg.data}')
        
    def create_http_server(self):
        host = self.get_parameter('host').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        server_address = (host, port)
        self.http_server = SimpleHTTPServer(server_address, SimpleHTTPRequestHandler, self)
        self.server_thread = threading.Thread(target=self.http_server.serve_forever)
        self.server_thread.start()
        self.get_logger().info(f'Starting server at {host}:{port}')
        
def main(args=None):
    rclpy.init(args=args)
    server = SimpleHttpInputServer()
    rclpy.spin(server)
    server.http_server.shutdown()
    server.server_thread.join()
    server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
