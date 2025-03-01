import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(Int32, 'arduino_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port as needed
        self.timer = self.create_timer(0.1, self.read_serial)  # Read every 100ms

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            try:
                value = int(line)  # Convert serial data to integer
                msg = Int32()
                msg.data = value
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')
            except ValueError:
                self.get_logger().warn(f'Invalid data received: {line}')

def main(args=None):
    rclpy.init(args=args)
    arduino_reader = ArduinoReader()
    rclpy.spin(arduino_reader)
    arduino_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()