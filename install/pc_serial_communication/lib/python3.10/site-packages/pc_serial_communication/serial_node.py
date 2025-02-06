import rclpy
from rclpy.node import Node
import serial

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        self.serial_port = '/dev/ttyACM0'  # Update to match your Arduino port
        self.baud_rate = 9600  # Ensure this matches your Arduino's baud rate
        self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.timer = self.create_timer(1.0, self.send_value)  # Timer to send data every second

    def send_value(self):
        value = 42  # Example value to send
        message = f"{value}\n"  # Add newline to indicate the end of the message
        self.serial_conn.write(message.encode())
        self.get_logger().info(f"Sent: {message.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_conn.close()  # Close serial connection
        rclpy.shutdown()

if __name__ == '__main__':
    main()
