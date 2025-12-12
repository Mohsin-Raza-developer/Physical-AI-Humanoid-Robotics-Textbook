#!/usr/bin/env python3
"""
Action Client Example - Movement Action Client

This example demonstrates how to create a ROS 2 action client that sends
movement goals to the action server and receives feedback.

Action Type: example_interfaces/action/Fibonacci
- Goal: order (int32) - number of Fibonacci numbers to generate
- Feedback: sequence (int32[]) - current sequence during generation
- Result: sequence (int32[]) - final Fibonacci sequence
"""

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class MovementActionClient(Node):
    """
    A ROS 2 action client that sends goals to the movement action server.
    """
    
    def __init__(self):
        super().__init__('movement_action_client')
        
        # Create the action client
        # The action name is '/fibonacci_action'
        # The action type is Fibonacci
        self._action_client = ActionClient(self, Fibonacci, '/fibonacci_action')
        
    def send_goal(self, order):
        """
        Send a goal to the action server and wait for the result.
        
        Args:
            order (int): Number of Fibonacci numbers to generate
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create a goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self.get_logger().info(f'Sending goal with order: {order}')
        
        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Add a callback for when the goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """
        Callback function when the goal response is received.
        
        Args:
            future: Future object containing the goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        # Request the result asynchronously
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
        
    def feedback_callback(self, feedback_msg):
        """
        Callback function for handling feedback from the action server.
        
        Args:
            feedback_msg: Feedback message from the action server
        """
        self.get_logger().info(
            f'Received feedback: Current sequence length: {len(feedback_msg.feedback.sequence)}'
        )
        
    def result_callback(self, future):
        """
        Callback function for handling the result from the action server.
        
        Args:
            future: Future object containing the result
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        
        # Shutdown after receiving the result
        rclpy.shutdown()


def main(args=None):
    """
    Main function to initialize and run the action client.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MovementActionClient node
    action_client = MovementActionClient()
    
    # Send a goal with order 10 (to generate 10 Fibonacci numbers)
    action_client.send_goal(10)
    
    try:
        # Start spinning to process callbacks
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Shutting down action client...')
    finally:
        # Clean up the node
        action_client.destroy_node()


if __name__ == '__main__':
    sys.exit(main())