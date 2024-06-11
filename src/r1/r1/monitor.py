import rclpy
from rclpy.node import Node
import aiohttp
from aiohttp import web
import asyncio
from rclpy.qos import qos_profile_sensor_data

class WebSocketROS2Node(Node):
    def __init__(self):
        super().__init__('dynamic_web_socket_ros2_node')
        self.subs = {}
        self.app = web.Application()
        self.get_logger().info("Creating web server")
        self.app.add_routes([web.get('/{topic}', self.topic_handler), web.get('/ws/{topic}', self.ws_handler)])
        self.get_logger().info("Launching web server")
        self.runner = web.AppRunner(self.app)

    async def topic_handler(self, request):
        self.get_logger().info(f"Handling topic {request.match_info['topic']}")
        topic_name = request.match_info['topic']

        html = f"""<html>
<head>
    <title>ROS Topic Stream: {topic_name}</title>
</head>
<body>
    <h1>Streaming ROS Topic: {topic_name}</h1>
    <div id="data"></div>
    <script>
        var ws = new WebSocket('ws://' + window.location.host + '/ws/{topic_name}');
        ws.onmessage = function(event) {{
            document.getElementById('data').innerText += event.data + '\\n';
        }};
    </script>
</body>
</html>"""
        return web.Response(text=html, content_type='text/html')

    async def ws_handler(self, request):
        self.get_logger().info(f"Handling websocket connection")
        topic_name = request.match_info['topic']
        ws = web.WebSocketNotFoundError.Response()
        await ws.prepare(request)

        if topic_name not in self.subs:
            self.subs[topic_name] = []
            self.create_subscription(topic_name)

        self.subs[topic_name].append(ws)

        try:
            while True:
                await asyncio.sleep(100)
        finally:
            self.subs[topic_name].remove(ws)
            await ws.close()

        return ws

    def create_subscription(self, topic_name):
        topic_type = self.get_topic_type(topic_name)
        if topic_type:
            callback = lambda msg, topic_name=topic_name: self.listener_callback(msg, topic_name)
            self.create_subscription(topic_type, topic_name, callback, qos_profile_sensor_data)

    def listener_callback(self, msg, topic_name):
        coros = [ws.send_str(str(msg)) for ws in self.subs.get(topic_name, [])]
        asyncio.gather(*coros)

    def get_topic_type(self, topic_name):
        topic_list_and_types = self.get_topic_names_and_types()
        for topic, types in topic_list_and_types:
            if topic == topic_name:
                return rclpy.serialization.get_message_class_from_type_name(types[0])
        return None

    async def start_server(self):
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()
        self.get_logger().info("Server running on http://localhost:8080/")

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketROS2Node()
    loop = asyncio.get_event_loop()
    loop.create_task(node.start_server())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
