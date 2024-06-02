import time
import gpiod
from gpiod.line import Direction, Value
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Define GPIO lines for trigger and echo
TRIG_LINE_OFFSET = 28  # GPIO2_D4 corresponds to line offset 28 on gpiochip2
ECHO_LINE_OFFSET = 20  # GPIO1_C4 corresponds to line offset 20 on gpiochip1

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # Parameters for GPIO lines
        self.declare_parameter('trigger_pin', TRIG_LINE_OFFSET)
        self.declare_parameter('echo_pin', ECHO_LINE_OFFSET)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('field_of_view_deg', 15.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('echo_chip', "/dev/gpiochip1")
        self.declare_parameter('trigger_chip', "/dev/gpiochip2")

        self.trigger_pin = self.get_parameter('trigger_pin').value
        self.echo_pin = self.get_parameter('echo_pin').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.fov_deg = self.get_parameter('field_of_view_deg').value
        self.frame_id = self.get_parameter('frame_id').value
        self.echo_chip = self.get_parameter('echo_chip').value
        self.trigger_chip = self.get_parameter('trigger_chip').value

        # GPIO setup
        self.trig_chip = gpiod.request_lines(
            self.trigger_chip,
            consumer="Ultrasonic Sensor",
            config={
                self.trigger_pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            },
        )
        self.echo_chip = gpiod.request_lines(
            self.echo_chip,
            consumer="Ultrasonic Sensor",
            config={
                self.echo_pin: gpiod.LineSettings(direction=Direction.INPUT)
            },
        )

        # Publisher
        self.publisher = self.create_publisher(Range, 'ultrasonic_sensor/distance', 10)
        self.timer = self.create_timer(0.1, self.publish_distance)

    def publish_distance(self):
        distance = self.measure_distance()
        if distance is not None:
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.fov_deg * (3.14159 / 180.0)
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = distance
            self.publisher.publish(msg)

    def measure_distance(self):
        # Trigger the sensor
        self.trig_chip.set_value(self.trigger_pin, Value.ACTIVE)
        time.sleep(0.00001)
        self.trig_chip.set_value(self.trigger_pin, Value.INACTIVE)

        # Wait for the echo to start
        start_time = time.time()
        while self.echo_chip.get_value(self.echo_pin) == Value.INACTIVE:
            start_time = time.time()

        # Wait for the echo to end
        end_time = time.time()
        while self.echo_chip.get_value(self.echo_pin) == Value.ACTIVE:
            end_time = time.time()

        # Calculate the distance
        elapsed_time = end_time - start_time
        distance = (elapsed_time * 34300) / 2  # Speed of sound is 34300 cm/s
        return distance

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()