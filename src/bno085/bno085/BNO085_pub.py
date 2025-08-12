import time
from math import atan2, sqrt
from math import pi as PI
import traceback

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

orientation_quat = [0, 0, 0, 0]  # x, y, z, w (real)
linear_accel = [0, 0, 0]         # x, y, z (m/s^2)
gyro = [0, 0, 0]                  # x, y, z (deg/s)


class BNO085_Publisher(Node):
    def __init__(self):
        super().__init__('BNO085_Publisher')
        # Publisher for IMU data
        self.imu_data_publisher = self.create_publisher(Imu, 'IMU_Data', 10)

        # Initialize IMU sensor
        self.imu = None
        self.init_sensor()

        # Timer for 30Hz publishing
        self.read_send_timer = self.create_timer(1.0 / 30.0, self.read_and_send_imu_data)

    def init_sensor(self):
        i2c = I2C(7)
        try:
            self.imu = BNO08X_I2C(i2c)
        except Exception:
            self.get_logger().error('Failed to connect to BNO085 via I2C...')
            raise Exception('Failed to connect to BNO085 via I2C')

        # Enable BNO085 features
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)

        time.sleep(0.5)  # Allow IMU to initialize

    def read_and_send_imu_data(self):
        # Read sensor values
        gyro[0], gyro[1], gyro[2] = self.imu.gyro
        linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.linear_acceleration
        orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3] = self.imu.quaternion

        # Create IMU message
        imu_data_msg = Imu()
        imu_data_msg.header.stamp = self.get_clock().now().to_msg()
        imu_data_msg.header.frame_id = "imu_frame"  # Hardcoded frame

        # Map IMU to ROS coordinate frame
        imu_data_msg.angular_velocity.x = gyro[1]
        imu_data_msg.angular_velocity.y = -gyro[0]
        imu_data_msg.angular_velocity.z = gyro[2]
        imu_data_msg.linear_acceleration.x = linear_accel[1]
        imu_data_msg.linear_acceleration.y = -linear_accel[0]
        imu_data_msg.linear_acceleration.z = linear_accel[2]
        imu_data_msg.orientation.x = orientation_quat[1]
        imu_data_msg.orientation.y = -orientation_quat[0]
        imu_data_msg.orientation.z = orientation_quat[2]
        imu_data_msg.orientation.w = orientation_quat[3]

        self.imu_data_publisher.publish(imu_data_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        bno_publisher = BNO085_Publisher()
        rclpy.spin(bno_publisher)
        bno_publisher.destroy_node()
    except Exception as e:
        print(orientation_quat)
        print("------------------")
        print(traceback.format_exc())
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
