import rclpy
from rclpy.node import Node
from aiohttp import web
import asyncio
from rclpy.qos import qos_profile_sensor_data
import asyncio
from rclpy.executors import MultiThreadedExecutor
import threading

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
        self.get_logger().info(f"Handling websocket connection for topic {request.match_info['topic']}")
        topic_name = request.match_info['topic']
        ws = web.WebSocketResponse()
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
        self.get_logger().info(f"Attempting to create subscription for {topic_name} with type {topic_type}")
        if topic_type:
            callback = lambda msg, tn=topic_name: self.listener_callback(msg, tn)
            self.create_subscription(topic_type, topic_name, callback, qos_profile_sensor_data)
        else:
            self.get_logger().error(f"Failed to find topic type for {topic_name}")

    def listener_callback(self, msg, topic_name):
        self.get_logger().info(f"Message received on {topic_name}: {msg}")
        coros = [ws.send_str(str(msg)) for ws in self.subs.get(topic_name, []) if ws is not None]
        asyncio.gather(*coros)
        
    def get_topic_type(self, topic_name):
        topic_list_and_types = self.get_topic_names_and_types()
        for topic, types in topic_list_and_types:
            if topic == topic_name:
                return rclpy.serialization.get_message_class_from_type_name(types[0])
        return None

    async def start_server(self):
        self.get_logger().info("Starting server")
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()
        self.get_logger().info("Server running on http://localhost:8080/")

def start_asyncio_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketROS2Node()
    executor = MultiThreadedExecutor()

    # Setup asyncio loop and tasks
    asyncio_loop = asyncio.new_event_loop()
    asyncio_loop.create_task(node.start_server())

    # Start asyncio loop in a separate thread
    thread = threading.Thread(target=start_asyncio_loop, args=(asyncio_loop,))
    thread.start()

    try:
        # Use the executor to handle ROS2 callbacks
        executor.add_node(node)
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        asyncio_loop.call_soon_threadsafe(asyncio_loop.stop)
        thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()