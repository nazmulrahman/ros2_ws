import rclpy
import serial
import json
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion
import tf_transformations

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # ROS 2 Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        # Connect to Arduino Serial
        self.serial_port = "/dev/ttyACM0"  # Change if needed
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException:
            self.get_logger().error(f"Failed to connect to {self.serial_port}")
            self.ser = None

        # Timer to read serial data
        self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.ser is None:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("{") and line.endswith("}"):
                data = json.loads(line)

                # Publish IMU Data
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # Assign accelerometer values
                imu_msg.linear_acceleration.x = data["ax"]
                imu_msg.linear_acceleration.y = data["ay"]
                imu_msg.linear_acceleration.z = data["az"]

                # No gyroscope, set angular velocity to zero
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0

                # Fake orientation (no gyroscope)
                q = tf_transformations.quaternion_from_euler(0, 0, 0)
                imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                self.imu_pub.publish(imu_msg)

                # Publish Magnetometer Data
                mag_msg = MagneticField()
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_msg.header.frame_id = "imu_link"
                mag_msg.magnetic_field.x = data["mx"]
                mag_msg.magnetic_field.y = data["my"]
                mag_msg.magnetic_field.z = data["mz"]

                self.mag_pub.publish(mag_msg)

                self.get_logger().info(f"Published IMU - ACC: {data['ax']}, {data['ay']}, {data['az']} | MAG: {data['mx']}, {data['my']}, {data['mz']}")

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Failed to parse serial data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
