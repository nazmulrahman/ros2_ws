import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

class BNO055Reader(Node):
    def __init__(self):
        super().__init__('bno055_reader')

        # ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(Vector3Stamped, 'imu/mag', 10)

        # Serial Connection
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_imu_data)

    def read_imu_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            data = line.split(',')

            if len(data) == 9:
                acc_x, acc_y, acc_z = float(data[0]), float(data[1]), float(data[2])
                gyro_x, gyro_y, gyro_z = float(data[3]), float(data[4]), float(data[5])
                mag_x, mag_y, mag_z = float(data[6]), float(data[7]), float(data[8])

                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # Accelerometer (m/s²)
                imu_msg.linear_acceleration.x = acc_x
                imu_msg.linear_acceleration.y = acc_y
                imu_msg.linear_acceleration.z = acc_z

                # Gyroscope (rad/s)
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                # Magnetometer (Published separately)
                mag_msg = Vector3Stamped()
                mag_msg.header.stamp = imu_msg.header.stamp
                mag_msg.header.frame_id = "imu_link"
                mag_msg.vector.x = mag_x
                mag_msg.vector.y = mag_y
                mag_msg.vector.z = mag_z

                # Publish messages
                self.imu_pub.publish(imu_msg)
                self.mag_pub.publish(mag_msg)

                self.get_logger().info(f"IMU: ACC=({acc_x}, {acc_y}, {acc_z}), GYRO=({gyro_x}, {gyro_y}, {gyro_z}), MAG=({mag_x}, {mag_y}, {mag_z})")

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
