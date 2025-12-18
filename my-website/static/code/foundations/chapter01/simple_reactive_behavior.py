#!/usr/bin/env python3
"""
Simple Reactive Behavior - Obstacle Avoidance
Foundations Week 1: Embodied Intelligence and Sensor-Motor Loops

This example demonstrates a simple reactive behavior where a mobile robot
avoids obstacles by turning away from the nearest detected obstacle.

This is a foundation example showing reactive control without ROS 2 packages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ReactiveObstacleAvoidance(Node):
    """
    A simple reactive obstacle avoidance behavior.

    The robot continuously scans its environment and turns away from
    the nearest obstacle. This demonstrates:
    - Direct sensor-motor coupling
    - Reactive control (no planning or world model)
    - Emergent navigation behavior
    """

    def __init__(self):
        super().__init__('reactive_obstacle_avoidance')

        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters
        self.declare_parameter('default_speed', 0.2)       # m/s
        self.declare_parameter('obstacle_threshold', 0.6)  # meters
        self.declare_parameter('turn_speed', 0.5)          # rad/s

        self.default_speed = self.get_parameter('default_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.turn_speed = self.get_parameter('turn_speed').value

        self.scan_sub  # Prevent unused warning

        self.get_logger().info('Reactive Obstacle Avoidance node started')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold}m')

    def scan_callback(self, msg: LaserScan):
        """
        Process laser scan and compute reactive response.

        Algorithm:
        1. Find the nearest obstacle in field of view
        2. If obstacle is close, turn away from it
        3. Otherwise, move forward

        Args:
            msg (LaserScan): Laser scan data
        """
        # Convert ranges to numpy array for easier processing
        ranges = np.array(msg.ranges)

        # Replace inf and nan with max range
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.where(np.isnan(ranges), msg.range_max, ranges)

        # Find minimum distance and its angle
        min_distance = np.min(ranges)
        min_idx = np.argmin(ranges)

        # Calculate angle of nearest obstacle
        # angle_increment is in radians, angle_min is the starting angle
        obstacle_angle = msg.angle_min + min_idx * msg.angle_increment

        # Reactive decision making
        if min_distance < self.obstacle_threshold:
            # Obstacle detected - turn away from it
            # If obstacle is on the right (negative angle), turn left (positive)
            # If obstacle is on the left (positive angle), turn right (negative)
            angular_vel = -np.sign(obstacle_angle) * self.turn_speed
            linear_vel = self.default_speed * 0.3  # Slow down while turning

            self.get_logger().info(
                f'Obstacle at {min_distance:.2f}m, {np.degrees(obstacle_angle):.0f}° - Turning'
            )
        else:
            # No obstacles - move forward
            linear_vel = self.default_speed
            angular_vel = 0.0

            self.get_logger().debug('Clear path - Moving forward')

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_pub.publish(twist)

    def stop(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().info('Robot stopped')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = ReactiveObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
