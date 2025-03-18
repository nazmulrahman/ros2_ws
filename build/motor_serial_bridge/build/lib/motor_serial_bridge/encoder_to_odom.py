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

        # Serial Connection to Arduino
        serial_port = "/dev/ttyACM0"  # Change if needed
        baud_rate = 9600
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {serial_port} at {baud_rate} baud.")
        except serial.SerialException:
            self.get_logger().error(f"Could not open serial port {serial_port}")
            self.ser = None

        # Odometry Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_odom)  # 10 Hz update

        # Robot Position Tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Robot rotation angle

        # Robot Parameters
        self.wheel_base = 0.3  # Distance between wheels (meters), adjust based on your robot

    def update_odom(self):
        if self.ser is None:
            return
        
        # Read data from Arduino
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            
            data = line.split('|')  # Expecting format "L_RPM:120|R_RPM:122|L_Speed:0.25|R_Speed:0.24"
            left_speed = float(data[2].split(':')[1])  # Extract left speed (m/s)
            right_speed = float(data[3].split(':')[1])  # Extract right speed (m/s)

            # Calculate velocity
            v = (left_speed + right_speed) / 2  # Linear velocity
            w = (right_speed - left_speed) / self.wheel_base  # Angular velocity
            
            dt = 0.1  # Time interval (matches the timer)
            delta_x = v * math.cos(self.theta) * dt
            delta_y = v * math.sin(self.theta) * dt
            delta_theta = w * dt
            
            # Update position
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.orientation = Quaternion(*tf_transformations.quaternion_from_euler(0, 0, self.theta))
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            # Publish Odometry
            self.odom_pub.publish(odom_msg)

            self.get_logger().info(f"Published Odom: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.1f}°")

        except Exception as e:
            self.get_logger().error(f"Serial Read Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
