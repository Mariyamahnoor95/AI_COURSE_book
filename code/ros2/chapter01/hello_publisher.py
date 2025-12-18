#!/usr/bin/env python3
"""
ROS 2 Publisher Example - Hello World
Chapter 1: ROS 2 Nodes and Topics

This simple publisher demonstrates the basics of creating a ROS 2 node
that publishes messages to a topic at regular intervals.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloPublisher(Node):
    """
    A simple publisher node that sends "Hello, ROS 2!" messages.

    This node publishes String messages to the 'hello_topic' topic
    every second using a timer callback.
    """

    def __init__(self):
        super().__init__('hello_publisher')

        # Create publisher: topic='hello_topic', msg_type=String, queue_size=10
        self.publisher = self.create_publisher(String, 'hello_topic', 10)

        # Create timer that calls timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Message counter
        self.count = 0

        self.get_logger().info('Hello Publisher node started')

    def timer_callback(self):
        """
        Timer callback function that publishes messages.
        Called every second by the timer.
        """
        msg = String()
        msg.data = f'Hello, ROS 2! Message #{self.count}'

        # Publish the message
        self.publisher.publish(msg)

        # Log to console
        self.get_logger().info(f'Published: "{msg.data}"')

        self.count += 1


def main(args=None):
    """Main entry point for the node."""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = HelloPublisher()

    try:
        # Spin the node (keeps it running and processing callbacks)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
