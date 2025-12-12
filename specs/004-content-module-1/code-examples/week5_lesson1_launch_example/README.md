# Week 5 Lesson 1: Launch Files Example

This directory contains example code for the launch files lesson, demonstrating how to create multi-node launch files with parameter configuration in ROS 2.

## Files

- `multi_node_launch.py`: Python launch file that starts multiple nodes with parameters
- `params.yaml`: YAML file containing parameter configuration for the nodes

## How to Run

1. Make sure you have ROS 2 Humble installed and sourced
2. Build the demo packages if not already done:
   ```bash
   ros2 pkg list | grep demo_nodes
   ```
3. Launch the multi-node system:
   ```bash
   ros2 launch specs/004-content-module-1/code-examples/week5_lesson1_launch_example/multi_node_launch.py
   ```
   
   Or if you've built the package containing this launch file:
   ```bash
   ros2 launch week5_lesson1_launch_example multi_node_launch.py
   ```

4. You should see output from the talker and listener nodes

## Expected Output

- Talker node will publish "Hello World" messages to the configured topic
- Listener node will receive and display the published messages
- Parameter server will show the loaded parameters

## Customization

You can pass launch arguments when running the launch file:
```bash
ros2 launch week5_lesson1_launch_example multi_node_launch.py topic_name:=my_custom_topic
```

This will make the nodes publish and subscribe to the topic named "my_custom_topic" instead of "chatter".