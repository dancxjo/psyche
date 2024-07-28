import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class ThoughtWatcher(Node):
    def __init__(self):
        super().__init__('thought_watcher')
        self.subscription = self.create_subscription(
            String,
            'thought',
            self.extract_commands,
            10
        )

        self.pubs = {
            'voice': self.create_publisher(
                String,
                'voice',
                10
            ),

            'shell_commands': self.create_publisher(
                String,
                'shell_commands',
                10
            )    
        }
        

    def handle_command(self, command):
        topic, body, tail = command
        
        if topic not in self.pubs:
            self.get_logger().error(f'Invalid topic: {topic}')
            return
        
        message = String()
        message.data = body
        self.get_logger().info(f'Publishing message: {message.data} to topic: {topic}')
        self.pubs[topic].publish(message)
        

    def extract_shell_commands(self, thought):
        commands = []
        commands = re.findall(r'```(\w+)\s+(.*?)\s*(```|$)', thought, re.DOTALL | re.MULTILINE)
        
        return commands

    def extract_commands(self, msg):
        thought = msg.data
        self.get_logger().info(f'Received thought: {thought}')
        commands = self.extract_shell_commands(thought)
        for command in commands:
            self.handle_command(command)

def main(args=None):
    rclpy.init(args=args)
    node = ThoughtWatcher()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()