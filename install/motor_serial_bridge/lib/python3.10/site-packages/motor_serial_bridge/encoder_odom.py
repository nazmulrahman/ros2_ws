#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException:
            self.get_logger().error("Could not open serial port /dev/ttyACM0")
            self.ser = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_odom)

    def update_odom(self):
        if self.ser is None:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()

            if not line.endswith(";"):
                return

            parts = line[:-1].split('|')  

            if len(parts) < 4:
                self.get_logger().error(f"Incomplete Serial Data: {line}")
                return

            rpmL = float(parts[0].split(':')[1])
            rpmR = float(parts[1].split(':')[1])
            left_speed = float(parts[2].split(':')[1])
            right_speed = float(parts[3].split(':')[1])

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
