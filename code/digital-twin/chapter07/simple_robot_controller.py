#!/usr/bin/env python3
"""
Simple Robot Controller for Gazebo Digital Twin
Module 2 - Week 6: Gazebo Physics Simulation

This example demonstrates controlling a robot in Gazebo simulation,
showing the digital twin concept where we control a virtual robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class SimpleRobotController(Node):
    """
    Simple controller that makes a robot drive in a square pattern.

    Demonstrates:
    - Publishing velocity commands to Gazebo
    - Receiving odometry feedback
    - State machine for sequential behaviors
    """

    # States for square pattern
    STATE_FORWARD = 0
    STATE_TURN = 1

    def __init__(self):
        super().__init__('simple_robot_controller')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Control parameters
        self.declare_parameter('side_length', 2.0)  # meters
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # State machine
        self.state = self.STATE_FORWARD
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.sides_completed = 0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.odom_sub  # Prevent unused warning

        self.get_logger().info('Simple Robot Controller started')
        self.get_logger().info(f'Will drive in a square with side length: {self.side_length}m')

    def odom_callback(self, msg: Odometry):
        """
        Update current position from odometry.

        Args:
            msg (Odometry): Odometry message from Gazebo
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw angle
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """
        Main control loop - executes state machine for square pattern.
        """
        twist = Twist()

        if self.state == self.STATE_FORWARD:
            # Drive forward until distance reached
            distance = math.sqrt(
                (self.current_x - self.start_x) ** 2 +
                (self.current_y - self.start_y) ** 2
            )

            if distance < self.side_length:
                # Keep driving forward
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            else:
                # Switch to turning
                self.state = self.STATE_TURN
                self.start_theta = self.current_theta
                self.get_logger().info(f'Completed side {self.sides_completed + 1}/4')

        elif self.state == self.STATE_TURN:
            # Turn 90 degrees
            angle_turned = abs(self.current_theta - self.start_theta)

            if angle_turned < math.pi / 2 - 0.1:  # 90 degrees minus tolerance
                # Keep turning
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            else:
                # Switch to forward
                self.state = self.STATE_FORWARD
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.sides_completed += 1

                if self.sides_completed >= 4:
                    self.get_logger().info('Square completed! Stopping.')
                    self.cmd_pub.publish(Twist())  # Stop
                    self.timer.cancel()
                    return

        # Publish velocity command
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    controller = SimpleRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # Stop robot on exit
        controller.cmd_pub.publish(Twist())
        controller.get_logger().info('Controller stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
