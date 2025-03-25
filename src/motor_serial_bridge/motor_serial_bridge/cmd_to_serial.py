#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class CmdToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_to_serial')

        # Connect to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)

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

        lx = msg.linear.x
        az = msg.angular.z

        # Mapping to integer commands
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
        self.ser.write(f"{command}\n".encode('utf-8'))
        self.get_logger().info(f"Sent command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdToSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
