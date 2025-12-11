#!/usr/bin/env python3
"""
Action Server Example - Movement Action Server

This example demonstrates how to create a ROS 2 action server that simulates
robot movement with feedback. The server receives a goal (coordinates) and
provides feedback during execution.

Action Type: example_interfaces/action/Fibonacci
- Goal: order (int32) - number of Fibonacci numbers to generate
- Feedback: sequence (int32[]) - current sequence during generation
- Result: sequence (int32[]) - final Fibonacci sequence
"""

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class MovementActionServer(Node):
    """
    A ROS 2 action server that simulates movement with feedback.
    """
    
    def __init__(self):
        super().__init__('movement_action_server')
        
        # Create the action server
        # The action name is '/fibonacci_action'
        # The action type is Fibonacci
        # The callback function is execute_fibonacci_goal
        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/fibonacci_action',
            self.execute_fibonacci_goal,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info('Action server started, waiting for goals...')

    def cancel_callback(self, goal_handle):
        """Callback function to handle goal cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_fibonacci_goal(self, goal_handle):
        """
        Execute callback for the action server.
        
        This function runs when a goal is received and handles the
        long-running task of generating a Fibonacci sequence.
        
        Args:
            goal_handle: The goal handle for this specific goal
            
        Returns:
            Fibonacci.Result object containing the final sequence
        """
        self.get_logger().info('Executing goal...')
        
        # Accept the goal
        goal_handle.execute()
        
        # Generate the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]  # Initialize sequence
        
        # Check if the order is too small
        if goal_handle.request.order < 1:
            # Return empty sequence if order is less than 1
            result = Fibonacci.Result()
            result.sequence = feedback_msg.sequence[:goal_handle.request.order]
            return result
            
        # Generate the Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                
                # Create a result with partial sequence
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result
            
            # Calculate next Fibonacci number
            if i < len(feedback_msg.sequence):
                # If we already have this number, skip
                continue
            else:
                # Calculate the next number in the sequence
                next_num = feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
                feedback_msg.sequence.append(next_num)
            
            # Publish feedback periodically
            if i % 2 == 0:  # Provide feedback every 2 iterations
                progress = int((i / goal_handle.request.order) * 100)
                self.get_logger().info(f'Feedback: {progress}% complete - Current sequence: {feedback_msg.sequence}')
                goal_handle.publish_feedback(feedback_msg)
                
                # Simulate some processing time
                time.sleep(0.5)
        
        # Complete the goal
        goal_handle.succeed()
        
        # Create the result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        
        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        
        return result


def main(args=None):
    """
    Main function to initialize and run the action server.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MovementActionServer node
    movement_action_server = MovementActionServer()
    
    try:
        # Start the ROS 2 event loop
        # This will keep the action server running and responsive
        # to incoming goals
        rclpy.spin(movement_action_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        movement_action_server.get_logger().info('Shutting down action server...')
    finally:
        # Clean up the node
        movement_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()