import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
import math

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if "ACC:" in line:
                try:
                    data = line.replace("ACC:", "").replace("GYRO:", "").split(";")
                    acc = [float(i) for i in data[0].split(",")]
                    gyro = [float(i) for i in data[1].split(",")]

                    imu_msg = Imu()
                    imu_msg.linear_acceleration.x = acc[0]
                    imu_msg.linear_acceleration.y = acc[1]
                    imu_msg.linear_acceleration.z = acc[2]
                    imu_msg.angular_velocity.x = math.radians(gyro[0])
                    imu_msg.angular_velocity.y = math.radians(gyro[1])
                    imu_msg.angular_velocity.z = math.radians(gyro[2])

                    self.publisher_.publish(imu_msg)
                    self.get_logger().info(f"IMU Published: {imu_msg}")

                except Exception as e:
                    self.get_logger().error(f"Error parsing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
