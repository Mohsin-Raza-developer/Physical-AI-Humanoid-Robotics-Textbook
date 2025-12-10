---
title: Launch Files and Parameters
sidebar_position: 5
description: Managing ROS 2 nodes with launch files and parameters
---

# Launch Files and Parameters

Launch files in ROS 2 allow you to start multiple nodes with a single command and manage their configuration parameters.

## Launch Files

Launch files are Python scripts that define how to launch a system of nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
        )
    ])
```

## Parameters

Parameters in ROS 2 can be set at launch time using YAML configuration files.