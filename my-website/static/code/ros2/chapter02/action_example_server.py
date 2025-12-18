#!/usr/bin/env python3
"""
ROS 2 Action Server Example
Chapter 2: ROS 2 Services and Actions

This example demonstrates creating an action server for a long-running
task (Fibonacci sequence generation) with feedback and cancellation support.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """
    Action server that computes Fibonacci sequence.

    Actions are for long-running tasks that provide:
    - Goal: What to achieve
    - Feedback: Progress updates during execution
    - Result: Final outcome
    - Cancellation: Ability to stop the task
    """

    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Fibonacci Action Server started')

    def goal_callback(self, goal_request):
        """
        Accept or reject goals based on request parameters.

        Args:
            goal_request: The goal request message

        Returns:
            GoalResponse: ACCEPT or REJECT
        """
        # Validate goal - reject if order is negative or too large
        if goal_request.order < 0:
            self.get_logger().warn(f'Rejecting negative order: {goal_request.order}')
            return GoalResponse.REJECT

        if goal_request.order > 50:
            self.get_logger().warn(f'Rejecting order too large: {goal_request.order}')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accepting goal: compute Fibonacci({goal_request.order})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation requests.

        Args:
            goal_handle: The goal being cancelled

        Returns:
            CancelResponse: ACCEPT or REJECT
        """
        self.get_logger().info('Received cancellation request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the long-running task with feedback.

        Args:
            goal_handle: Handle for this goal execution

        Returns:
            result: The final result message
        """
        self.get_logger().info('Executing goal...')

        # Initialize Fibonacci sequence
        sequence = [0, 1]
        order = goal_handle.request.order

        # Feedback message
        feedback_msg = Fibonacci.Feedback()

        # Compute Fibonacci sequence step by step
        for i in range(1, order):
            # Check if cancellation was requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')

                # Return partial result
                result = Fibonacci.Result()
                result.sequence = sequence
                return result

            # Compute next number
            sequence.append(sequence[i] + sequence[i - 1])

            # Send feedback
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Publishing feedback: {sequence}')

            # Simulate computation time
            time.sleep(0.5)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return final result
        result = Fibonacci.Result()
        result.sequence = sequence

        self.get_logger().info(f'Goal succeeded with result: {sequence}')

        return result


def main(args=None):
    rclpy.init(args=args)

    server = FibonacciActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
