```python
#!/usr/bin/env python3
# Example ROS 2 Python node for Physical AI

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PhysicalAINode(Node):
    def __init__(self):
        super().__init__('physical_ai_node')
        self.publisher = self.create_publisher(String, 'physical_ai_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Physical AI message: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    physical_ai_node = PhysicalAINode()
    rclpy.spin(physical_ai_node)
    physical_ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()