"""
Zenoh IMU Publisher for Ear Device (MPU-6050)

- Reads IMU data from MPU-6050 via I2C
- Publishes data to zenoh topic 'imu/data'

Requires: smbus2, zenoh-python

Example:
    python zenoh_imu_pub.py
"""

import time
from dataclasses import dataclass, asdict
from typing import Dict
import json

import smbus2
import zenoh

@dataclass
class ImuData:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    timestamp: float

class MPU6050:
    def __init__(self, bus: int = 1, address: int = 0x68):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        # Wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0)

    def read(self) -> ImuData:
        def read_word(reg):
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg+1)
            val = (high << 8) + low
            if val >= 0x8000:
                val = -((65535 - val) + 1)
            return val
        # Read accelerometer
        accel_x = read_word(0x3B) / 16384.0
        accel_y = read_word(0x3D) / 16384.0
        accel_z = read_word(0x3F) / 16384.0
        # Read gyroscope
        gyro_x = read_word(0x43) / 131.0
        gyro_y = read_word(0x45) / 131.0
        gyro_z = read_word(0x47) / 131.0
        return ImuData(
            accel_x=accel_x,
            accel_y=accel_y,
            accel_z=accel_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            timestamp=time.time()
        )

def main():
    mpu = MPU6050()
    session = zenoh.open()
    key = 'imu/data'
    print("Publishing IMU data to zenoh topic 'imu/data'")
    while True:
        data = mpu.read()
        session.put(key, json.dumps(asdict(data)))
        time.sleep(0.05)  # 20Hz

if __name__ == "__main__":
    main()
