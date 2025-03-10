#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

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

        # Publish odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Initialize position & time
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

        # Start a timer to read encoder feedback
        self.create_timer(0.1, self.read_encoder_feedback)  # 10Hz

    def cmd_vel_callback(self, msg):
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

    def read_encoder_feedback(self):
        if self.ser is None:
            return
        
        try:
            data = self.ser.readline().decode('utf-8').strip()
            if data.startswith("SpeedL:"):
                parts = data.split(',')
                speedL = float(parts[0].split(':')[1])
                speedR = float(parts[1].split(':')[1])

                # Compute robot movement
                dt = time.time() - self.last_time
                self.last_time = time.time()
                vx = (speedL + speedR) / 2.0
                vth = (speedR - speedL) / 0.35  # Adjust with wheel separation

                self.x += vx * math.cos(self.theta) * dt
                self.y += vx * math.sin(self.theta) * dt
                self.theta += vth * dt

                # Publish Odometry
                odom_msg = Odometry()
                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.twist.twist.linear.x = vx
                odom_msg.twist.twist.angular.z = vth
                self.odom_publisher.publish(odom_msg)

                self.get_logger().info(f"Odometry -> X: {self.x:.2f}, Y: {self.y:.2f}, Theta: {self.theta:.2f}")
        
        except Exception as e:
            self.get_logger().error(f"Failed to read encoder: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdToSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
