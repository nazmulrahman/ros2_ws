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

        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_odom)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_base = 0.3 

    def update_odom(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split('|')
            rpmL = float(parts[0].split(':')[1])
            rpmR = float(parts[1].split(':')[1])
            left_speed = float(parts[2].split(':')[1])
            right_speed = float(parts[3].split(':')[1])

            v = (left_speed + right_speed) / 2  
            w = (right_speed - left_speed) / self.wheel_base  
            dt = 0.1  
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += w * dt

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.odom_pub.publish(odom_msg)
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
