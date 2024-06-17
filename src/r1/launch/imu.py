from launch_ros.actions import Node

imu1 = Node(
        package="mpu6050driver",
        executable="mpu6050driver",
        name="mpu6050driver",
        output="screen",
    )
