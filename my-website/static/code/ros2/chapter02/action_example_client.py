#!/usr/bin/env python3
"""
ROS 2 Action Client Example
Chapter 2: ROS 2 Services and Actions

This example demonstrates creating an action client that requests
Fibonacci sequence computation with feedback monitoring.
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """
    Action client that requests Fibonacci sequence computation.

    Demonstrates:
    - Sending goals to action servers
    - Receiving feedback during execution
    - Handling goal results
    - Cancelling goals
    """

    def __init__(self):
        super().__init__('fibonacci_action_client')

        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.get_logger().info('Fibonacci Action Client started')

    def send_goal(self, order: int):
        """
        Send a goal to compute Fibonacci sequence.

        Args:
            order (int): Number of elements in the sequence
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: Fibonacci({order})')

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register callback for when goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called when server accepts or rejects the goal.

        Args:
            future: Future containing goal handle
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')

        # Get result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called when feedback is received from server.

        Args:
            feedback_msg: Feedback message with current sequence
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        """
        Called when final result is received.

        Args:
            future: Future containing result
        """
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Goal succeeded!')
            self.get_logger().info(f'Final sequence: {result.sequence}')
        elif status == 5:  # CANCELED
            self.get_logger().warn('Goal was canceled')
            self.get_logger().info(f'Partial sequence: {result.sequence}')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

        # Shutdown after receiving result
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Get command line argument
    if len(sys.argv) != 2:
        print('Usage: action_example_client.py <order>')
        print('Example: python3 action_example_client.py 10')
        sys.exit(1)

    try:
        order = int(sys.argv[1])
    except ValueError:
        print('Error: Argument must be an integer')
        sys.exit(1)

    client = FibonacciActionClient()

    try:
        client.send_goal(order)
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()


if __name__ == '__main__':
    main()
