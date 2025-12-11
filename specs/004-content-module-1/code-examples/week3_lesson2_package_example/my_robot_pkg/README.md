# my_robot_pkg

Example ROS 2 package demonstrating basic package structure and organization.

## Description

This package contains a simple talker node that publishes string messages to the `/chatter` topic at 1 Hz. It demonstrates the minimal structure required for a ROS 2 Python package.

## Package Structure

```
my_robot_pkg/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration
├── setup.cfg            # Additional setup configuration
├── resource/
│   └── my_robot_pkg     # Package marker file
├── my_robot_pkg/        # Python source code directory
│   ├── __init__.py      # Makes directory a Python package
│   └── talker_node.py   # Simple publisher node
└── README.md            # This file
```

## Dependencies

- ROS 2 Humble
- Python 3.10+
- rclpy
- std_msgs

## Building

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Build this package
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash
```

## Usage

Run the talker node:

```bash
ros2 run my_robot_pkg talker
```

Expected output:
```
[INFO] [talker_node]: Talker Node started - publishing to "chatter" topic
[INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 0"
[INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 1"
[INFO] [talker_node]: Publishing: "Hello ROS 2! Count: 2"
...
```

## Verifying Communication

In another terminal, echo the topic to see published messages:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /chatter
```

List all active topics:

```bash
ros2 topic list
```

Get information about the chatter topic:

```bash
ros2 topic info /chatter
```

## Learning Resources

This package is used in **Week 3 Lesson 2: Nodes and Packages** of the Physical AI & Humanoid Robotics textbook.

## License

Apache-2.0

## Author

Student Name (student@example.com)
