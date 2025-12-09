---
title: Nodes and Packages
sidebar_position: 3
description: Creating and managing ROS 2 packages and nodes
---

# Nodes and Packages

In ROS 2, a node is a process that performs computation. Nodes are organized into packages for easier code reuse and distribution.

## Creating a Package

To create a new package in ROS 2:

```bash
ros2 pkg create --build-type ament_python my_robot_package
```

## Creating a Node

A basic ROS 2 node implementation in Python:
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
```