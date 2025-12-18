#!/usr/bin/env python3
"""
ROS 2 Wall Following Robot Example
Chapter 1: ROS 2 Nodes and Topics & Embodied Intelligence

This example demonstrates a simple reactive behavior where a mobile robot
follows a wall at a constant distance using proportional control.

Topics:
- Subscribes to: /scan (sensor_msgs/LaserScan) - laser range data
- Publishes to: /cmd_vel (geometry_msgs/Twist) - velocity commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowerNode(Node):
    """
    A reactive wall-following robot using proportional control.

    The robot uses a side-mounted laser scanner to maintain a constant
    distance from a wall. This demonstrates closed-loop control and
    sensor-motor coupling.
    """

    def __init__(self):
        super().__init__('wall_follower')

        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Control parameters
        self.declare_parameter('target_distance', 0.5)  # meters
        self.declare_parameter('forward_speed', 0.2)    # m/s
        self.declare_parameter('kp', 0.5)               # Proportional gain
        self.declare_parameter('min_distance', 0.2)     # Safety threshold

        # Get parameters
        self.target_distance = self.get_parameter('target_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp').value
        self.min_distance = self.get_parameter('min_distance').value

        # Prevent unused variable warning
        self.scan_sub

        self.get_logger().info('Wall Follower node started')
        self.get_logger().info(f'Target distance: {self.target_distance}m')
        self.get_logger().info(f'Forward speed: {self.forward_speed}m/s')
        self.get_logger().info(f'Proportional gain: {self.kp}')

    def scan_callback(self, msg: LaserScan):
        """
        Process laser scan data and compute velocity commands.

        This is the core sensor-motor loop:
        1. Read sensor data (wall distance)
        2. Compute error from target distance
        3. Apply proportional control
        4. Publish velocity command

        Args:
            msg (LaserScan): Laser scan message containing range measurements
        """
        # Get distance to wall on the right side (90 degrees)
        # Assuming laser scan covers 360 degrees
        num_ranges = len(msg.ranges)
        right_idx = num_ranges // 4  # 90 degrees to the right

        # Handle invalid readings (inf, nan)
        wall_distance = msg.ranges[right_idx]
        if wall_distance == float('inf') or wall_distance != wall_distance:
            # No valid reading, stop the robot
            self.get_logger().warn('No valid wall detected')
            self.publish_velocity(0.0, 0.0)
            return

        # Safety check: if too close, stop
        if wall_distance < self.min_distance:
            self.get_logger().warn(f'Too close to wall: {wall_distance:.2f}m')
            self.publish_velocity(0.0, 0.0)
            return

        # Proportional controller
        # error = actual_distance - target_distance
        error = wall_distance - self.target_distance

        # Negative gain: turn toward wall when too far, away when too close
        angular_vel = -self.kp * error

        # Limit angular velocity to reasonable range
        max_angular_vel = 1.0  # rad/s
        angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))

        # Publish velocity command
        self.publish_velocity(self.forward_speed, angular_vel)

        # Log for debugging
        self.get_logger().debug(
            f'Distance: {wall_distance:.2f}m, '
            f'Error: {error:.2f}m, '
            f'Angular vel: {angular_vel:.2f}rad/s'
        )

    def publish_velocity(self, linear_x: float, angular_z: float):
        """
        Publish velocity command to /cmd_vel topic.

        Args:
            linear_x (float): Forward velocity in m/s
            angular_z (float): Angular velocity in rad/s
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    node = WallFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop command on exit
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.get_logger().info('Shutting down, robot stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
