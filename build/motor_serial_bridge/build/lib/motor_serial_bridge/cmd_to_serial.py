#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class CmdToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_to_serial')

        # Serial Port Configuration
        serial_port = "/dev/ttyACM0"
        baud_rate = 9600

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            time.sleep(2)  # Allow time for Arduino to reset
            self.get_logger().info(f"Connected to Arduino on {serial_port} at {baud_rate} baud.")
        except serial.SerialException:
            self.get_logger().error(f"Could not open serial port {serial_port}")
            self.ser = None

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return

        # Extract linear and angular values
        lx = msg.linear.x
        az = msg.angular.z



        # Key Mapping for Arduino Commands
# ______________________________________________________________________________________
# | Key  | Meaning         | Published `Twist` Message    | Send to Arduino (int value) |
# |------|-----------------|------------------------------|-----------------------------|
# | `i`  | Move Forward    | linear.x > 0, angular.z = 0  |  1                          |
# | `,`  | Move Backward   | linear.x < 0, angular.z = 0  | -1                          |
# | `j`  | Turn Left       | linear.x = 0, angular.z > 0  | -2                          |
# | `l`  | Turn Right      | linear.x = 0, angular.z < 0  |  2                          |
# | `u`  | Forward + Left  | linear.x > 0, angular.z > 0  |  4                          |
# | `o`  | Forward + Right | linear.x > 0, angular.z < 0  |  3                          |
# | `m`  | Backward + Left | linear.x < 0, angular.z > 0  | -3                          |
# | `.`  | Backward + Right| linear.x < 0, angular.z < 0  | -4                          |
# | `k`  | Stop            | linear.x = 0, angular.z = 0  |  0                          |
# |________________________|______________________________|_____________________________|

        # Mapping Twist values to integer commands for Arduino
        command = 0  # Default: Stop

        if lx > 0 and az == 0:
            command = 1  # Move Forward
        elif lx < 0 and az == 0:
            command = -1  # Move Backward
        elif lx == 0 and az > 0:
            command = -2  # Turn Left
        elif lx == 0 and az < 0:
            command = 2  # Turn Right
        elif lx > 0 and az > 0:
            command = 4  # Forward + Left
        elif lx > 0 and az < 0:
            command = 3  # Forward + Right
        elif lx < 0 and az > 0:
            command = -3  # Backward + Left
        elif lx < 0 and az < 0:
            command = -4  # Backward + Right

        # Send command to Arduino
        try:
            self.ser.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdToSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
