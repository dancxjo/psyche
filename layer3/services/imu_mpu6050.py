#!/usr/bin/env python3
"""IMU service for MPU-6050 sensor publishing to zenoh/ros2.

This service reads IMU data from an MPU-6050 sensor and publishes
it on appropriate zenoh and ROS2 topics for the ear device.

Example:
    $ python imu_mpu6050.py
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Dict, Any
import logging

try:
    import smbus2
except ImportError:
    smbus2 = None

try:
    import zenoh
except ImportError:
    zenoh = None


@dataclass
class IMUData:
    """Represents IMU sensor data from MPU-6050."""
    
    timestamp: float
    accel_x: float
    accel_y: float  
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    temperature: float


class MPU6050:
    """Driver for MPU-6050 IMU sensor."""
    
    # MPU-6050 I2C address and register definitions
    MPU6050_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40
    TEMP_OUT_H = 0x41
    TEMP_OUT_L = 0x42
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48
    
    def __init__(self, bus_id: int = 1):
        """Initialize MPU-6050.
        
        Parameters
        ----------
        bus_id : int
            I2C bus ID (default 1 for Raspberry Pi)
        """
        self.bus_id = bus_id
        self.bus = None
        
    def connect(self) -> bool:
        """Connect to the MPU-6050 sensor.
        
        Returns
        -------
        bool
            True if connection successful, False otherwise
        """
        if smbus2 is None:
            logging.warning("smbus2 not available - using mock data")
            return False
            
        try:
            self.bus = smbus2.SMBus(self.bus_id)
            # Wake up the MPU-6050 (it starts in sleep mode)
            self.bus.write_byte_data(self.MPU6050_ADDR, self.PWR_MGMT_1, 0)
            logging.info("MPU-6050 connected successfully")
            return True
        except Exception as e:
            logging.error(f"Failed to connect to MPU-6050: {e}")
            return False
    
    def read_word_2c(self, addr: int) -> int:
        """Read a 16-bit signed word from the sensor."""
        if self.bus is None:
            return 0
        high = self.bus.read_byte_data(self.MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(self.MPU6050_ADDR, addr + 1)
        val = (high << 8) + low
        return val - 65536 if val >= 0x8000 else val
    
    def read_imu_data(self) -> IMUData:
        """Read IMU data from the sensor.
        
        Returns
        -------
        IMUData
            Current IMU readings
        """
        if self.bus is None:
            # Return mock data if no hardware connection
            return IMUData(
                timestamp=time.time(),
                accel_x=0.1,
                accel_y=0.2, 
                accel_z=9.8,
                gyro_x=0.01,
                gyro_y=0.02,
                gyro_z=0.03,
                temperature=25.0
            )
        
        # Read raw sensor data
        accel_x = self.read_word_2c(self.ACCEL_XOUT_H) / 16384.0  # Convert to g
        accel_y = self.read_word_2c(self.ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(self.ACCEL_ZOUT_H) / 16384.0
        
        temp = self.read_word_2c(self.TEMP_OUT_H) / 340.0 + 36.53  # Convert to Celsius
        
        gyro_x = self.read_word_2c(self.GYRO_XOUT_H) / 131.0  # Convert to degrees/sec
        gyro_y = self.read_word_2c(self.GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(self.GYRO_ZOUT_H) / 131.0
        
        return IMUData(
            timestamp=time.time(),
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            temperature=temp
        )


class IMUPublisher:
    """Publisher for IMU data to zenoh and ROS2 topics."""
    
    def __init__(self, device_id: str = "ear"):
        """Initialize the IMU publisher.
        
        Parameters
        ----------
        device_id : str
            Device identifier for topic naming
        """
        self.device_id = device_id
        self.zenoh_session = None
        self.imu_topic = f"/{device_id}/imu/data"
        
    def connect_zenoh(self) -> bool:
        """Connect to zenoh session.
        
        Returns
        -------
        bool
            True if connection successful, False otherwise
        """
        if zenoh is None:
            logging.warning("zenoh-python not available")
            return False
            
        try:
            config = zenoh.Config()
            self.zenoh_session = zenoh.open(config)
            logging.info("Connected to zenoh session")
            return True
        except Exception as e:
            logging.error(f"Failed to connect to zenoh: {e}")
            return False
    
    def publish_imu_data(self, data: IMUData) -> None:
        """Publish IMU data to zenoh topic.
        
        Parameters
        ----------
        data : IMUData
            IMU data to publish
        """
        # Create ROS2 sensor_msgs/Imu compatible message
        msg = {
            "header": {
                "stamp": {
                    "sec": int(data.timestamp),
                    "nanosec": int((data.timestamp % 1) * 1e9)
                },
                "frame_id": f"{self.device_id}_imu_link"
            },
            "linear_acceleration": {
                "x": data.accel_x * 9.80665,  # Convert to m/s²
                "y": data.accel_y * 9.80665,
                "z": data.accel_z * 9.80665
            },
            "angular_velocity": {
                "x": data.gyro_x * 0.017453293,  # Convert to rad/s
                "y": data.gyro_y * 0.017453293,
                "z": data.gyro_z * 0.017453293
            },
            "orientation": {
                "x": 0.0,  # Not calculated from raw MPU-6050
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            },
            "orientation_covariance": [-1.0] + [0.0] * 8,  # -1 indicates unknown
            "angular_velocity_covariance": [0.0] * 9,
            "linear_acceleration_covariance": [0.0] * 9
        }
        
        if self.zenoh_session:
            try:
                self.zenoh_session.put(self.imu_topic, json.dumps(msg))
                logging.debug(f"Published IMU data to {self.imu_topic}")
            except Exception as e:
                logging.error(f"Failed to publish to zenoh: {e}")
        else:
            logging.info(f"Would publish to {self.imu_topic}: {msg}")


def main() -> None:
    """Main service loop for IMU data publishing."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Initialize components
    sensor = MPU6050()
    publisher = IMUPublisher()
    
    # Connect to hardware and services
    sensor_connected = sensor.connect()
    zenoh_connected = publisher.connect_zenoh()
    
    if not sensor_connected:
        logging.warning("Running with mock IMU data")
    if not zenoh_connected:
        logging.warning("Running without zenoh publishing")
    
    # Main publish loop
    rate_hz = 50  # 50Hz publish rate
    sleep_time = 1.0 / rate_hz
    
    logging.info(f"Starting IMU publisher at {rate_hz}Hz")
    
    try:
        while True:
            # Read and publish IMU data
            imu_data = sensor.read_imu_data()
            publisher.publish_imu_data(imu_data)
            
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        logging.info("Shutting down IMU service")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
    finally:
        if publisher.zenoh_session:
            publisher.zenoh_session.close()


if __name__ == "__main__":
    main()