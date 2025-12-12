#!/usr/bin/env python3
"""
Talker Node - Example ROS 2 Publisher
======================================
This node demonstrates a simple publisher that sends string messages.

Expected Output:
  [INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 0"
  [INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 1"
  [INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 2"
  ...

Usage:
  # After building the package with colcon
  ros2 run my_robot_pkg talker
"""

# Import ROS 2 Python library
import rclpy
from rclpy.node import Node

# Import String message type from std_msgs
from std_msgs.msg import String


class TalkerNode(Node):
    """
    A simple publisher node that sends string messages at 1 Hz.

    This demonstrates:
    - Creating a ROS 2 node class
    - Setting up a publisher
    - Using timers for periodic publishing
    - Logging messages
    """

    def __init__(self):
        """Initialize the talker node."""
        # Call parent class constructor with node name
        super().__init__('talker_node')

        # Create publisher for String messages on 'chatter' topic
        # Queue size of 10 means buffer up to 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer that calls timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Counter to track number of messages sent
        self.count = 0

        # Log initialization message
        self.get_logger().info('Talker Node started - publishing to "chatter" topic')

    def timer_callback(self):
        """
        Called every second by the timer.
        Creates and publishes a String message.
        """
        # Create a new String message
        msg = String()

        # Set the message data
        msg.data = f'Hello ROS 2! Count: {self.count}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log what we published
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter for next message
        self.count += 1


def main(args=None):
    """
    Main function - entry point for the node.

    Args:
        args: Command-line arguments (optional)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create instance of TalkerNode
    node = TalkerNode()

    # Spin the node (keep it running and process callbacks)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Talker Node stopped by user')

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


# Run main() when script is executed directly
if __name__ == '__main__':
    main()
