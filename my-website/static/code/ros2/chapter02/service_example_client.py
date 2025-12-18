#!/usr/bin/env python3
"""
ROS 2 Service Client Example
Chapter 2: ROS 2 Services and Actions

This example demonstrates creating a service client that
requests addition of two integers from a service server.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """
    Service client that requests addition of two integers.

    This demonstrates how to call services in ROS 2.
    """

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client for the /add_two_ints service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Connected to Add Two Ints service')

    def send_request(self, a: int, b: int):
        """
        Send a request to the service and wait for response.

        Args:
            a (int): First integer
            b (int): Second integer

        Returns:
            int: The sum of a and b
        """
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')

        # Call service asynchronously
        future = self.cli.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Received response: {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)

    # Get command line arguments
    if len(sys.argv) != 3:
        print('Usage: service_example_client.py <a> <b>')
        print('Example: python3 service_example_client.py 5 7')
        sys.exit(1)

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Error: Arguments must be integers')
        sys.exit(1)

    client = AddTwoIntsClient()

    try:
        result = client.send_request(a, b)
        if result is not None:
            print(f'\nResult: {a} + {b} = {result}')
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
