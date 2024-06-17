import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class ShellSession(Node):
    def __init__(self):
        super().__init__('shell_session')
        self.shell_output = self.create_publisher(String, 'shell_output', 10)
        self.shell_input = self.create_subscription(String, 'shell_commands', self.handle_command, 10)

    def handle_command(self, msg):
        self.get_logger().info(f'Received command on topic: {msg.data}')
        command_output = self.execute_command(msg.data)
        self.publish_command_output(command_output)

    def execute_command(self, command):
        try:
            self.get_logger().info(f'Executing command: {command}')
            result = subprocess.run(command, shell=True, text=True, capture_output=True)
            self.get_logger().info(f'Command executed with return code: {result.returncode}')
            return result.stdout if result.returncode == 0 else result.stderr
        except Exception as e:
            self.get_logger().error(f'Error executing command: {str(e)}')
            return str(e)

    def publish_command_output(self, output):
        message = String()
        message.data = output
        self.get_logger().info(f'Publishing command output: {message.data}')
        self.shell_output.publish(message)
        self.get_logger().info('Published command output')

def main(args=None):
    rclpy.init(args=args)
    node = ShellSession()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
