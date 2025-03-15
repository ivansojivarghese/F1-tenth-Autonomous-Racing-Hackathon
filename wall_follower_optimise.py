#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.scan_callback, 1)
        self.sub_odom = self.create_subscription(Odometry, "odom", self.odom_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, "drive", 1)
        self.drive_msg = AckermannDriveStamped()

        # Optimized PID parameters
        self.Kp = 0.50  # Increased for more aggressive correction
        self.Ki = 0.040  # Increased for stability at high speeds
        self.Kd = 0.004  # Slightly increased to reduce oscillations

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = 0.0

        self.longitudinal_vel = 0
        self.coefficient_of_friction = 0.35  # Increased for better traction
        self.wheel_base = 0.33

    def getRange(self, scan_data, angle):
        ranges = scan_data.ranges
        angle_rad = angle * (np.pi / 180)
        index = int(abs(angle_rad - scan_data.angle_max) / scan_data.angle_increment)
        return ranges[index]

    def odom_callback(self, odom_data):
        self.longitudinal_vel = odom_data.twist.twist.linear.x

    def scan_callback(self, scan_data):
        secs = scan_data.header.stamp.sec
        nsecs = scan_data.header.stamp.nanosec
        current_time = secs + nsecs * 1e-9
        
        if self.prev_time == 0:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        angle_b = 90
        angle_a = 40
        theta = (angle_b - angle_a) * (np.pi / 180)
        
        distance_b = self.getRange(scan_data, angle_b)
        distance_a = self.getRange(scan_data, angle_a) / 2
        
        alpha = -1 * np.arctan2((distance_a * np.cos(theta) - distance_b), (distance_a * np.sin(theta)))
        actual_distance = distance_b * np.cos(alpha)
        desired_distance = 1.0
        error = desired_distance - actual_distance
        lookahead_distance = self.longitudinal_vel * 0.2
        error_corrected = error + lookahead_distance * np.sin(alpha)

        self.integral += error_corrected * dt
        derivative = (error_corrected - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error_corrected

        steering_angle = (self.Kp * error_corrected) + (self.Ki * self.integral) + (self.Kd * derivative)
        steering_angle = np.clip(steering_angle, -0.4, 0.4)
        self.drive_msg.drive.steering_angle = steering_angle

        steering_angle_degrees = abs(steering_angle * (180 / np.pi))

        # Adaptive speed control based on steering angle
        if steering_angle_degrees < 15:
            max_speed = 12.0  # Higher speed for straights
        elif steering_angle_degrees < 30:
            max_speed = 10.0  # Moderate speed for mild turns
        else:
            max_speed = 8.5  # Lower speed for sharp turns

        self.drive_msg.drive.speed = min(
            max_speed,
            np.sqrt((10 * self.coefficient_of_friction * self.wheel_base) / np.abs(np.tan(steering_angle)))
        )

        self.pub_drive.publish(self.drive_msg)
        self.get_logger().info(f"steering_angle: {steering_angle_degrees:.2f} | speed: {self.drive_msg.drive.speed:.2f}")


def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
