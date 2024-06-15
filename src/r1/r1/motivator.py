import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from geometry_msgs.msg import Twist

class Motivator(Node):
    def __init__(self):
        super().__init__('motivator')
        self.subscription = self.create_subscription(
            String,
            'twists',
            self.twists_callback,
            10
        )
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.publisher_sensation = self.create_publisher(
            String,
            'sensation',
            10
        )

    def twists_callback(self, msg):
        self.get_logger().debug(f'Received twists: {msg.data}')
        self.process_twists(msg.data)

    def process_twists(self, script):
        try:
            self.publisher_sensation.publish(String(data=f"""You have sent these commands to your body: {script}"""))
            script_array = json.loads(script)
            if not isinstance(script_array, list):
                raise ValueError("Parsed JSON is not a list")
        except json.JSONDecodeError as e:
            self.publisher_sensation.publish(String(data=f"""Your body says: Failed to decode JSON: {e} {script}"""))

            self.get_logger().error(f'Failed to decode JSON: {e}')
            return
        except ValueError as e:
            self.publisher_sensation.publish(String(data=f"""Your body says: Failed to decode JSON: {e} {script}"""))
            self.get_logger().error(f'Error in script data: {e}')
            return

        for twist_command in script_array:
            if not isinstance(twist_command, dict):
                self.get_logger().error(f'Invalid twist command: {twist_command}')
                self.publisher_sensation.publish(String(data=f"""Your body says: Invalid twist command: {twist_command}"""))
                continue

            if 'reasoning' in twist_command:
                self.publisher_sensation.publish(String(data=f"""You issued this command ostensibly for reason: {twist_command['reasoning']}"""))

            if 'pause_time' in twist_command:
                try:
                    pause_time = float(twist_command['pause_time'])
                    self.get_logger().info(f'Pausing for {pause_time} seconds')
                    self.publisher_sensation.publish(String(data=f"""Pausing for {pause_time} seconds"""))
                    self.create_timer(pause_time, lambda: None).cancel()  # Use ROS timer to sleep without blocking
                except (ValueError, TypeError) as e:
                    self.publisher_sensation.publish(String(data=f"""Invalid pause time {pause_time} seconds"""))
                    self.get_logger().error(f'Invalid pause_time value: {e}')
                continue

            duration = twist_command.get('duration', 5.0)
            rate = twist_command.get('rate', 20)
            if rate == 0:
                self.get_logger().error('Rate cannot be zero, using default rate of 20 Hz')
                self.publisher_sensation.publish(String(data=f"""Rate cannot be zero, using default rate of 20 Hz"""))
                rate = 20
                
            interval = 1.0 / rate

            twist_msg = Twist()
            if 'linear' in twist_command:
                twist_msg.linear.x = float(twist_command['linear'].get('x', 0.0))
                twist_msg.linear.y = float(twist_command['linear'].get('y', 0.0))
                twist_msg.linear.z = float(twist_command['linear'].get('z', 0.0))
            if 'angular' in twist_command:
                twist_msg.angular.x = float(twist_command['angular'].get('x', 0.0))
                twist_msg.angular.y = float(twist_command['angular'].get('y', 0.0))
                twist_msg.angular.z = float(twist_command['angular'].get('z', 0.0))

            self.execute_twist(twist_msg, duration, interval)

    def execute_twist(self, twist_msg, duration, interval):
        if type(duration) is not int and type(duration) is not float:
            self.get_logger().error(f'Invalid duration: {duration}')
            self.publisher_sensation.publish(String(data=f"""Invalid duration: {duration}"""))
            return
        
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)
        self.publisher_sensation.publish(String(data=f"""Executing {twist_msg} for {duration}s at {interval} until {end_time}"""))

        def timer_callback():
            if self.get_clock().now() < end_time:
                # self.publisher_sensation.publish(String(data=f"""publishing {twist_msg}"""))
                self.publisher_cmd_vel.publish(twist_msg)
            else:
                self.publisher_sensation.publish(String(data=f"""time's run out on this maneuvre; stopping body"""))
                self.publisher_cmd_vel.publish(Twist())  # Stop the robot
                timer.cancel()

        self.publisher_sensation.publish(String(data=f"""Setting a timer to keep moving until {end_time} at {interval} Hz"""))
        timer = self.create_timer(interval, timer_callback)

def main(args=None):
    rclpy.init(args=args)
    node = Motivator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
