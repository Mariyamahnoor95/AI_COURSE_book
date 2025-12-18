#!/usr/bin/env python3
"""
ROS 2 Subscriber Example - Hello World
Chapter 1: ROS 2 Nodes and Topics

This simple subscriber demonstrates how to create a ROS 2 node
that listens to messages on a topic and processes them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloSubscriber(Node):
    """
    A simple subscriber node that receives messages from hello_topic.

    This node subscribes to String messages on the 'hello_topic' topic
    and prints them to the console when received.
    """

    def __init__(self):
        super().__init__('hello_subscriber')

        # Create subscription: topic='hello_topic', msg_type=String,
        # callback=message_callback, queue_size=10
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.message_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Hello Subscriber node started')
        self.get_logger().info('Waiting for messages on hello_topic...')

    def message_callback(self, msg):
        """
        Callback function that processes incoming messages.

        Args:
            msg (String): The received message from the topic
        """
        # Log the received message
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """Main entry point for the node."""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = HelloSubscriber()

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
