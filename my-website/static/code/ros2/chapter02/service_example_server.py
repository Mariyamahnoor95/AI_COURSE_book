#!/usr/bin/env python3
"""
ROS 2 Service Server Example
Chapter 2: ROS 2 Services and Actions

This example demonstrates creating a simple service server that
adds two integers together (synchronous request-response pattern).
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    Service server that adds two integers.

    This demonstrates the request-response pattern in ROS 2 services.
    Services are synchronous - the client waits for the server's response.
    """

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Add Two Ints Service Server started')
        self.get_logger().info('Waiting for requests on /add_two_ints...')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that processes the request and returns a response.

        Args:
            request: AddTwoInts.Request with .a and .b fields
            response: AddTwoInts.Response with .sum field

        Returns:
            response: The filled response object
        """
        # Perform the computation
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
