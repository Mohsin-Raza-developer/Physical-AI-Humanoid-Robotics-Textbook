#!/usr/bin/env python3
"""
Temperature Sensor Publisher Example
=====================================
This node simulates a temperature sensor that publishes readings to a topic.

Expected Output:
When you run this node, you should see output like:
  [INFO] [minimal_publisher]: Publishing temperature: 20.0
  [INFO] [minimal_publisher]: Publishing temperature: 20.1
  [INFO] [minimal_publisher]: Publishing temperature: 20.2
  ...

Usage:
  python3 week3_lesson1_publisher.py
  OR
  ros2 run <package_name> publisher (if installed in a package)
"""

# Import the ROS 2 Python library
import rclpy
from rclpy.node import Node

# Import the message type we'll use (Float32 for temperature)
from std_msgs.msg import Float32


class MinimalPublisher(Node):
    """
    A simple ROS 2 node that publishes temperature readings.

    This node demonstrates the basic publish-subscribe pattern by
    simulating a temperature sensor that sends readings every second.
    """

    def __init__(self):
        """Initialize the publisher node."""
        # Call the parent class (Node) constructor with the node name
        # This name will appear in 'ros2 node list'
        super().__init__('minimal_publisher')

        # Create a publisher that sends Float32 messages on the 'temperature' topic
        # The '10' is the queue size - how many messages to buffer if subscriber is slow
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)

        # Create a timer that calls timer_callback every 1.0 seconds
        # This is how we publish data at regular intervals
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize our temperature counter (starts at 20.0°C)
        self.counter = 0.0

        # Log that the publisher has started
        self.get_logger().info('Temperature Publisher started - publishing to "temperature" topic')

    def timer_callback(self):
        """
        This function is called every second by the timer.
        It creates a message, fills it with data, and publishes it.
        """
        # Create a new Float32 message
        msg = Float32()

        # Set the temperature value (base temperature + counter)
        msg.data = 20.0 + self.counter

        # Publish the message to the 'temperature' topic
        self.publisher_.publish(msg)

        # Log the published value so we can see what's happening
        self.get_logger().info(f'Publishing temperature: {msg.data:.1f}°C')

        # Increment the counter for the next reading (simulates temperature change)
        self.counter += 0.1


def main(args=None):
    """
    Main function that initializes ROS 2 and runs the publisher node.

    Args:
        args: Command-line arguments (optional)
    """
    # Initialize the ROS 2 Python library
    # This MUST be called before creating any nodes
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    node = MinimalPublisher()

    # Keep the node running and processing callbacks (like our timer)
    # This will run until interrupted (Ctrl+C)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info('Publisher stopped by user')

    # Clean up the node
    node.destroy_node()

    # Shutdown the ROS 2 Python library
    rclpy.shutdown()


# This ensures main() only runs when the script is executed directly
# (not when imported as a module)
if __name__ == '__main__':
    main()
