#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

# If your 'cmd_vel_unstamped' is a geometry_msgs/Twist-like message,
# import the correct message type:
from geometry_msgs.msg import Twist

class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # Params (you can make these ROS2 parameters in real code)
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # or 'COM3' on Windows
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('max_speed_pwm', 255)  # L298N: 0-255

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Attempt to open the serial port
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            time.sleep(2)  # Wait for Arduino reset
            self.get_logger().info(f"Connected to Arduino on {serial_port} at {baud_rate} baud.")
        except serial.SerialException:
            self.get_logger().error(f"Could not open serial port {serial_port}")
            self.ser = None

        # Subscribe to the diff_cont/cmd_vel_unstamped topic
        # Adjust the topic name or message type if needed
        self.subscription = self.create_subscription(
            Twist,
            'diff_cont/cmd_vel_unstamped',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Cache max speed
        self.max_pwm = float(self.get_parameter('max_speed_pwm').value)

        # Robot-specific constants
        self.wheel_separation = 0.20   # meters (distance between wheels)
        self.wheel_radius = 0.05      # meters (if needed for real scaling)

        # Just a placeholder for simple scaling
        # Adjust these to tune your speed mapping
        self.linear_to_pwm = 200.0  
        self.angular_to_pwm = 150.0

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return

        # Extract linear.x and angular.z
        lx = msg.linear.x
        az = msg.angular.z

        # -------------------------------------------------------
        # 1) Compute Left & Right Motor Speed as described:
        #    - If lx>0 and az=0 => both forward
        #    - If lx>0 and az>0 => left < right
        #    - etc.
        #
        # A straightforward approach is the standard differential
        # drive formula:
        #   vLeft  = lx - (az * wheel_separation/2)
        #   vRight = lx + (az * wheel_separation/2)
        #
        # Then convert these to PWM (and sign).
        # -------------------------------------------------------
        vLeft = lx - (az * self.wheel_separation / 2.0)
        vRight = lx + (az * self.wheel_separation / 2.0)

        # Scale them to approximate PWM range
        left_pwm_float = vLeft * self.linear_to_pwm
        right_pwm_float = vRight * self.linear_to_pwm

        # Another approach (less physically “exact”, simpler):
        # left_pwm_float = (lx * self.linear_to_pwm) - (az * self.angular_to_pwm)
        # right_pwm_float = (lx * self.linear_to_pwm) + (az * self.angular_to_pwm)

        # Convert float -> int & clamp
        left_pwm = int(max(min(left_pwm_float, self.max_pwm), -self.max_pwm))
        right_pwm = int(max(min(right_pwm_float, self.max_pwm), -self.max_pwm))

        # Debug info
        self.get_logger().info(f"cmd_vel -> L={left_pwm}, R={right_pwm} (from lx={lx:.2f}, az={az:.2f})")

        # -------------------------------------------------------
        # 2) Send to Arduino via serial in the format: "L,R\n"
        # -------------------------------------------------------
        cmd_str = f"{left_pwm},{right_pwm}\n"
        try:
            self.ser.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
