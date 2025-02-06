import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
import time

class SerialCmdVelNode(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_node')
        self.serial_port = '/dev/ttyACM0'  # Update to match your Arduino port
        self.baud_rate = 9600  # Match Arduino's baud rate
        self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Subscribe to the correct topic
        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',  # Updated topic
            self.cmd_vel_callback,
            10
        )

        # Publisher for fake encoder feedback
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.05, self.read_encoder_feedback)  # 20Hz frequency

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Send cmd_vel to Arduino
        message = f"{linear_x},{angular_z}\n"
        self.serial_conn.write(message.encode())
        self.get_logger().info(f"Sent to Arduino: {message.strip()}")

    def read_encoder_feedback(self):
        """Reads fake encoder values from Arduino and publishes them as joint states"""
        if self.serial_conn.in_waiting > 0:
            try:
                feedback = self.serial_conn.readline().decode().strip()
                left_ticks, right_ticks = map(float, feedback.split(','))  # Read from Arduino
                
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
                joint_state_msg.position = [left_ticks, right_ticks]
                self.joint_state_publisher.publish(joint_state_msg)

                self.get_logger().info(f"Published joint states: {left_ticks}, {right_ticks}")
            except Exception as e:
                self.get_logger().warn(f"Error parsing encoder feedback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_conn.close()  # Close the serial connection
        rclpy.shutdown()

if __name__ == '__main__':
    main()
