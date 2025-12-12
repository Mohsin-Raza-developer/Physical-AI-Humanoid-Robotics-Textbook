#!/usr/bin/env python3
"""
Service Client Example - Calculator Service Client

This example demonstrates how to create a ROS 2 service client that makes
calculation requests to the calculator service server.

Service Type: example_interfaces/srv/AddTwoInts
- Request: a (int64), b (int64)
- Response: sum (int64)
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorClient(Node):
    """
    A ROS 2 service client that sends requests to the calculator service.
    """
    
    def __init__(self):
        super().__init__('calculator_client')
        
        # Create the service client
        # The service name is '/add_two_ints'
        # The service type is AddTwoInts
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
        
        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /add_two_ints...')
            
        self.get_logger().info('Service client ready')
        
    def send_request(self, a, b):
        """
        Send a request to the service server and wait for the response.
        
        Args:
            a (int): First integer to add
            b (int): Second integer to add
            
        Returns:
            The sum of a and b returned by the service
        """
        # Create a request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Log the request
        self.get_logger().info(f'Sending request: {a} + {b}')
        
        # Call the service asynchronously
        self.future = self.client.call_async(request)
        
        # We'll wait for the response in the main function (not in this method)
        return self.future


def main(args=None):
    """
    Main function to initialize and run the service client.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the CalculatorClient node
    calculator_client = CalculatorClient()
    
    # Define the values to add
    a = 5
    b = 3
    
    # Send the request
    future = calculator_client.send_request(a, b)
    
    try:
        # Spin until the future is complete (i.e., response is received)
        while rclpy.ok():
            rclpy.spin_once(calculator_client)
            
            # Check if the future is complete
            if future.done():
                try:
                    # Get the response
                    response = future.result()
                    
                    # Log the result
                    calculator_client.get_logger().info(
                        f'Result: {a} + {b} = {response.sum}'
                    )
                    
                    # Exit the loop
                    break
                except Exception as e:
                    calculator_client.get_logger().error(f'Service call failed: {e}')
                    break
    except KeyboardInterrupt:
        calculator_client.get_logger().info('Shutting down service client...')
    finally:
        # Clean up the node
        calculator_client.destroy_node()
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    sys.exit(main())