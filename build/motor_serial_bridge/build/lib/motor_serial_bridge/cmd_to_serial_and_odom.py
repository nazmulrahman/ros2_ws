#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math

class CmdToSerialWithOdomNode(Node):
    def __init__(self):
        super().__init__('cmd_to_serial_with_odom')

        # Serial Port Configuration
        serial_port = "/dev/ttyACM0"  # Adjust this if needed
        baud_rate = 115200

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            time.sleep(2)  # Allow Arduino to reset
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

        # Publisher for Odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Timer to Read Serial Data
        self.create_timer(0.1, self.read_encoder_data)

        # Robot Parameters (Adjust Based on Your Robot)
        self.wheel_radius = 0.05  # 5 cm wheel radius
        self.wheel_base = 0.3  # Distance between wheels (30 cm)
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg):
        """Send teleop keyboard or Nav2 commands to Arduino"""
        if self.ser is None:
            return

        # Extract linear and angular values
        lx = msg.linear.x
        az = msg.angular.z

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

    def read_encoder_data(self):
        """Read encoder feedback from Arduino and publish odometry"""
        if self.ser is None:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("ENC,"):
                _, pulse_count, rpm = line.split(',')
                pulse_count = int(pulse_count)
                rpm = float(rpm)

                # Convert RPM to Linear Velocity (V = ω * r)
                angular_velocity = (rpm * 2 * math.pi) / 60  # Convert RPM to rad/s
                linear_velocity = angular_velocity * self.wheel_radius  # v = ω * r

                # Estimate Position (Basic Dead Reckoning)
                dt = 0.1  # Time step (100ms)
                self.x_pos += linear_velocity * math.cos(self.theta) * dt
                self.y_pos += linear_velocity * math.sin(self.theta) * dt
                self.theta += (linear_velocity / self.wheel_base) * dt  # Change in orientation

                # Create Odometry Message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "odom"
                odom_msg.child_frame_id = "base_link"

                # Set Position
                odom_msg.pose.pose.position.x = self.x_pos
                odom_msg.pose.pose.position.y = self.y_pos
                quaternion = quaternion_from_euler(0, 0, self.theta)
                odom_msg.pose.pose.orientation = Quaternion(
                    x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
                )

                # Set Velocity
                odom_msg.twist.twist.linear.x = linear_velocity
                odom_msg.twist.twist.angular.z = angular_velocity

                # Publish Odometry
                self.odom_publisher.publish(odom_msg)
                self.get_logger().info(f"Published Odom: x={self.x_pos:.2f}, y={self.y_pos:.2f}, theta={self.theta:.2f}")

        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdToSerialWithOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
