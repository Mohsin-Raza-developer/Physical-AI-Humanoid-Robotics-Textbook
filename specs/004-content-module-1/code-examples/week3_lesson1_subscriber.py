#!/usr/bin/env python3
"""
Temperature Display Subscriber Example
=======================================
This node listens to temperature readings published on a topic and displays them.

Expected Output:
When you run this node (with the publisher also running), you should see:
  [INFO] [minimal_subscriber]: Received temperature: 20.0°C
  [INFO] [minimal_subscriber]: Received temperature: 20.1°C
  [INFO] [minimal_subscriber]: Received temperature: 20.2°C
  ...

Usage:
  # Terminal 1: Run the publisher first
  python3 week3_lesson1_publisher.py

  # Terminal 2: Run this subscriber
  python3 week3_lesson1_subscriber.py
"""

# Import the ROS 2 Python library
import rclpy
from rclpy.node import Node

# Import the message type we'll receive (Float32 for temperature)
from std_msgs.msg import Float32


class MinimalSubscriber(Node):
    """
    A simple ROS 2 node that subscribes to temperature readings.

    This node demonstrates how to receive messages from a topic by
    listening to the 'temperature' topic and displaying each reading.
    """

    def __init__(self):
        """Initialize the subscriber node."""
        # Call the parent class (Node) constructor with the node name
        # This name will appear in 'ros2 node list'
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'temperature' topic
        # - Float32: The message type we expect to receive
        # - 'temperature': The topic name (must match the publisher's topic)
        # - listener_callback: The function to call when a message arrives
        # - 10: Queue size - how many messages to buffer if processing is slow
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        # (subscription needs to be stored to prevent garbage collection)
        self.subscription

        # Log that the subscriber has started
        self.get_logger().info('Temperature Subscriber started - listening to "temperature" topic')

    def listener_callback(self, msg):
        """
        This function is called automatically whenever a message arrives.

        Args:
            msg: The received Float32 message containing temperature data
        """
        # Extract the temperature value from the message
        temperature = msg.data

        # Display the received temperature
        self.get_logger().info(f'Received temperature: {temperature:.1f}°C')

        # Optional: You could add logic here to:
        # - Store temperatures in a list for averaging
        # - Trigger an alert if temperature exceeds a threshold
        # - Convert to Fahrenheit: (temperature * 9/5) + 32
        # - Publish the data to another topic


def main(args=None):
    """
    Main function that initializes ROS 2 and runs the subscriber node.

    Args:
        args: Command-line arguments (optional)
    """
    # Initialize the ROS 2 Python library
    # This MUST be called before creating any nodes
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    node = MinimalSubscriber()

    # Keep the node running and processing incoming messages
    # This will run until interrupted (Ctrl+C)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Subscriber stopped by user')

    # Clean up the node
    node.destroy_node()

    # Shutdown the ROS 2 Python library
    rclpy.shutdown()


# This ensures main() only runs when the script is executed directly
# (not when imported as a module)
if __name__ == '__main__':
    main()
