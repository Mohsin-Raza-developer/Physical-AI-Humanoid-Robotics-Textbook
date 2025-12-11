#!/usr/bin/env python3
"""
Multi-node launch example for Week 5 Lesson 1 (Launch Files)

This launch file starts multiple nodes with parameters:
- A talker node that publishes messages
- A listener node that subscribes to messages
- A parameter server node with configuration parameters

Launch with: ros2 launch week5_lesson1_launch_example multi_node_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments that can be passed to the launch file
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='chatter',
        description='Name of the topic to publish to'
    )

    # Get the launch configuration
    topic_name = LaunchConfiguration('topic_name')

    # Create the talker node
    talker_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='talker_node',
        parameters=[
            {'topic_name': topic_name}
        ],
        remappings=[
            ('chatter', topic_name)
        ]
    )

    # Create the listener node
    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='listener_node',
        parameters=[
            {'topic_name': topic_name}
        ],
        remappings=[
            ('chatter', topic_name)
        ]
    )

    # Create a parameter server node (simulated)
    param_server_node = Node(
        package='demo_nodes_py',
        executable='parameter_client',
        name='param_server',
        parameters=[
            {'param_a': 'value_a'},
            {'param_b': 42},
            {'param_c': True}
        ]
    )

    # Return the launch description containing all nodes
    return LaunchDescription([
        topic_name_arg,
        talker_node,
        listener_node,
        param_server_node
    ])


# Expected output when running the launch file:
# [INFO] [launch]: All processes have started...
# [INFO] [talker_node-1]: process started with pid [XXXXX]
# [INFO] [listener_node-2]: process started with pid [XXXXX]
# [INFO] [param_server-3]: process started with pid [XXXXX]
# [INFO] [talker_node-1]: Publishing: 'Hello World: XX'
# [INFO] [listener_node-2]: I heard: [Hello World: XX]
# [INFO] [launch]: Process talker_node-1 exited with code XXX
# [INFO] [launch]: Process listener_node-2 exited with code XXX
# [INFO] [launch]: Process param_server-3 exited with code XXX