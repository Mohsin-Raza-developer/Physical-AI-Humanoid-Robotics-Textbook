#!/usr/bin/env python3
"""
Service Server Example - Calculator Service

This example demonstrates how to create a ROS 2 service server that performs
calculations. The server receives two integers and returns their sum.

Service Type: example_interfaces/srv/AddTwoInts
- Request: a (int64), b (int64)
- Response: sum (int64)
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorService(Node):
    """
    A ROS 2 service server that adds two integers together.
    """
    
    def __init__(self):
        super().__init__('calculator_service')
        
        # Create the service server
        # The service name is '/add_two_ints'
        # The service type is AddTwoInts
        # The callback function is handle_add_two_ints
        self.service = self.create_service(
            AddTwoInts, 
            '/add_two_ints', 
            self.handle_add_two_ints
        )
        
        self.get_logger().info('Service server started, waiting for requests...')

    def handle_add_two_ints(self, request, response):
        """
        Callback function for handling service requests.
        
        Args:
            request: The AddTwoInts service request object containing a and b
            response: The AddTwoInts service response object to be returned
            
        Returns:
            The response object with the calculated sum
        """
        # Calculate the sum
        response.sum = request.a + request.b
        
        # Log the request and response
        self.get_logger().info(
            f'Received request: {request.a} + {request.b} = {response.sum}'
        )
        
        # Return the response
        return response


def main(args=None):
    """
    Main function to initialize and run the service server.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the CalculatorService node
    calculator_service = CalculatorService()
    
    try:
        # Start the ROS 2 event loop
        # This will keep the service server running and responsive
        # to incoming requests
        rclpy.spin(calculator_service)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        calculator_service.get_logger().info('Shutting down service server...')
    finally:
        # Clean up the node
        calculator_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()