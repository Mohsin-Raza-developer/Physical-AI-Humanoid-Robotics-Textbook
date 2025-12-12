---
title: Module 1 Troubleshooting FAQ
sidebar_label: Troubleshooting FAQ
sidebar_position: 98
description: Common issues and solutions for ROS 2 Module 1
tags: [ros2, troubleshooting, faq, help]
---

# Module 1: ROS 2 Troubleshooting FAQ

This FAQ compiles the most common issues students encounter in Module 1 (ROS 2 Fundamentals) with detailed solutions and prevention strategies.

## Quick Navigation

- [Environment and Setup Issues](#environment-and-setup-issues)
- [Package and Build Issues](#package-and-build-issues)
- [Node and Communication Issues](#node-and-communication-issues)
- [Launch File Issues](#launch-file-issues)
- [URDF and Visualization Issues](#urdf-and-visualization-issues)
- [General Debugging Tips](#general-debugging-tips)

---

## Environment and Setup Issues

### Q1: "Package 'std_msgs' not found" or "ModuleNotFoundError: No module named 'rclpy'"

**Symptom**:
```
ModuleNotFoundError: No module named 'std_msgs'
ModuleNotFoundError: No module named 'rclpy'
```

**Cause**: ROS 2 environment not sourced in current terminal

**Solution**:
```bash
source /opt/ros/humble/setup.bash
# Then run your script again
python3 your_script.py
```

**Prevention**: Add to `~/.bashrc` to source automatically:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Related**: Week 3 Lesson 1, Week 3 Lesson 2

---

### Q2: ROS 2 commands not found (ros2, colcon, etc.)

**Symptom**:
```bash
ros2: command not found
colcon: command not found
```

**Cause**: ROS 2 not installed or environment not sourced

**Solution**:
1. Verify ROS 2 installation:
   ```bash
   ls /opt/ros/
   ```

2. If installed, source it:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. If not installed, follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html)

**Related**: Setup Documentation

---

### Q3: "package.xml not found" or workspace issues after build

**Symptom**:
```
ERROR: package.xml not found
Workspace not set up correctly
```

**Cause**: Not running commands from the correct directory

**Solution**:
```bash
# Ensure you're in the workspace root
cd ~/ros2_ws
colcon build

# Source the workspace overlay
source install/setup.bash
```

**Prevention**: Always verify your current directory:
```bash
pwd  # Should show ~/ros2_ws or your workspace path
```

**Related**: Week 3 Lesson 2, Week 4 Lesson 2

---

## Package and Build Issues

### Q4: "No packages found" after creating package

**Symptom**: Package not recognized by `ros2 pkg list` or colcon

**Cause**: Package not in a proper workspace `src/` directory

**Solution**:
1. Verify directory structure:
   ```bash
   ~/ros2_ws/
   ├── src/
   │   └── your_package/
   │       ├── package.xml
   │       ├── setup.py
   │       └── your_package/
   ```

2. Rebuild workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select your_package
   source install/setup.bash
   ```

**Prevention**: Always create packages inside `src/` directory

**Related**: Week 3 Lesson 2

---

### Q5: "Setup.py or setup.cfg not found" during build

**Symptom**:
```
ERROR: setup.py or setup.cfg not found
```

**Cause**: Missing or incorrectly configured `setup.py` in Python package

**Solution**:
1. Ensure `setup.py` exists in package root
2. Verify `setup.py` has correct structure:
   ```python
   from setuptools import setup

   package_name = 'your_package'

   setup(
       name=package_name,
       version='0.0.1',
       packages=[package_name],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your@email.com',
       description='Package description',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'node_name = your_package.node_file:main',
           ],
       },
   )
   ```

**Related**: Week 3 Lesson 2, Week 4 Lesson 2

---

### Q6: Custom message/service types not found

**Symptom**:
```
ModuleNotFoundError: No module named 'your_package.msg'
Package 'your_interfaces' not found
```

**Cause**: Custom interfaces package not built or not sourced

**Solution**:
1. Build the interfaces package first:
   ```bash
   colcon build --packages-select your_interfaces
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Verify messages are generated:
   ```bash
   ros2 interface list | grep your_interfaces
   ```

4. Add dependency to `package.xml`:
   ```xml
   <depend>your_interfaces</depend>
   ```

**Prevention**: Always build interfaces packages before packages that use them

**Related**: Week 4 Lesson 2

---

### Q7: Build errors with custom interfaces

**Symptom**:
```
CMake Error: Could not find rosidl_default_generators
```

**Cause**: Missing dependencies in `package.xml` or `CMakeLists.txt`

**Solution**:
1. Add to `package.xml`:
   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

2. Add to `CMakeLists.txt`:
   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/YourMessage.msg"
     "srv/YourService.srv"
   )
   ```

**Related**: Week 4 Lesson 2

---

## Node and Communication Issues

### Q8: Nodes not communicating (messages not received)

**Symptom**: Publisher works but subscriber receives nothing

**Causes and Solutions**:

**1. Topic name mismatch**:
```bash
# Check what topics exist
ros2 topic list

# Check publisher topic
ros2 topic info /your_topic

# Verify subscriber is listening to the same topic
```

**2. QoS (Quality of Service) mismatch**:
- Publisher and subscriber must have compatible QoS settings
- Default QoS usually works for basic cases

**3. Timing issue** (subscriber started after messages sent):
- Ensure subscriber runs before or at same time as publisher
- Or use launch files to coordinate startup

**Related**: Week 3 Lesson 1

---

### Q9: Service call hangs or times out

**Symptom**: Service client waits indefinitely or times out

**Cause**: Service server not running or name mismatch

**Solution**:
1. Check if service exists:
   ```bash
   ros2 service list
   ros2 service type /your_service
   ```

2. Test service manually:
   ```bash
   ros2 service call /your_service your_pkg/srv/YourService "{field: value}"
   ```

3. Verify server node is running:
   ```bash
   ros2 node list
   ```

**Prevention**: Start server before client, or add retry logic

**Related**: Week 4 Lesson 1

---

### Q10: Action goal not accepted or no feedback received

**Symptom**: Action client sends goal but receives no response

**Cause**: Action server not running or interface mismatch

**Solution**:
1. Verify action server is running:
   ```bash
   ros2 action list
   ros2 action info /your_action
   ```

2. Test action manually:
   ```bash
   ros2 action send_goal /your_action your_pkg/action/YourAction "{goal_field: value}"
   ```

3. Check action definition matches between client and server

**Related**: Week 4 Lesson 1

---

## Launch File Issues

### Q11: "Launch file not found"

**Symptom**:
```
Package 'your_package' not found
No launch files found
```

**Cause**: Launch file not in `launch/` directory or package not built

**Solution**:
1. Ensure launch file is in `launch/` directory
2. Verify `.launch.py` extension
3. Add to `setup.py`:
   ```python
   import os
   from glob import glob

   setup(
       # ... other fields
       data_files=[
           (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
       ],
   )
   ```

4. Rebuild package:
   ```bash
   colcon build --packages-select your_package
   source install/setup.bash
   ```

**Related**: Week 5 Lesson 1

---

### Q12: Import errors in launch files

**Symptom**:
```python
ModuleNotFoundError: No module named 'launch'
```

**Cause**: Missing launch dependencies in `package.xml`

**Solution**:
Add dependencies to `package.xml`:
```xml
<exec_depend>ros2launch</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Rebuild and source workspace

**Related**: Week 5 Lesson 1

---

### Q13: Launch file syntax errors

**Symptom**: Python syntax errors when running launch file

**Causes and Solutions**:

**1. Missing `generate_launch_description()` function**:
```python
def generate_launch_description():
    return LaunchDescription([
        # your nodes here
    ])
```

**2. Incorrect LaunchDescription format**:
- Must return `LaunchDescription` object
- Nodes and actions must be in a list: `LaunchDescription([...])`

**3. Indentation errors**:
- Python is sensitive to indentation
- Use consistent spaces (4 spaces recommended)

**Related**: Week 5 Lesson 1

---

### Q14: Remapping or parameters not applied

**Symptom**: Nodes start but don't use expected topic names or parameters

**Cause**: Incorrect remapping or parameter syntax

**Solution**:
1. Remapping syntax:
   ```python
   remappings=[('/original_topic', '/new_topic')]
   ```

2. Parameter syntax:
   ```python
   parameters=[{'param_name': value}]
   # or from file:
   parameters=['path/to/params.yaml']
   ```

3. Verify with:
   ```bash
   ros2 topic list  # Check topic names
   ros2 param list  # Check parameters
   ```

**Related**: Week 5 Lesson 1

---

## URDF and Visualization Issues

### Q15: URDF fails to load in RViz2

**Symptom**: RViz2 shows errors or robot doesn't appear

**Causes and Solutions**:

**1. XML syntax errors**:
```bash
# Validate URDF
check_urdf your_robot.urdf
```

**2. Missing robot_state_publisher**:
```bash
# Manually run robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat your_robot.urdf)"
```

**3. TF frames not being published**:
- Ensure all joints have TF publishers
- Check: `ros2 run tf2_tools view_frames`

**Related**: Week 5 Lesson 2

---

### Q16: URDF joint/link errors

**Symptom**:
```
Error: joint 'your_joint' parent link 'link1' not found
```

**Cause**: Parent-child link mismatch in URDF

**Solution**:
1. Verify link names match exactly (case-sensitive)
2. Ensure parent link is defined before child joint
3. Check URDF structure:
   ```xml
   <link name="base_link"/>
   <joint name="joint1" type="revolute">
     <parent link="base_link"/>
     <child link="link1"/>
   </joint>
   <link name="link1"/>
   ```

**Related**: Week 5 Lesson 2

---

### Q17: RViz2 won't start or crashes

**Symptom**: RViz2 window doesn't appear or crashes immediately

**Causes and Solutions**:

**1. Display not set (WSL2/SSH)**:
```bash
export DISPLAY=:0
# or for WSL2 with WSLg:
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

**2. OpenGL issues**:
```bash
# Check OpenGL support
glxinfo | grep "OpenGL"

# Try software rendering
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

**3. Missing dependencies**:
```bash
sudo apt install ros-humble-rviz2
```

**Related**: Week 5 Lesson 2, Setup Documentation

---

## General Debugging Tips

### Debugging Workflow

1. **Check ROS 2 environment**:
   ```bash
   echo $ROS_DISTRO  # Should show "humble"
   ros2 --version
   ```

2. **List running nodes**:
   ```bash
   ros2 node list
   ros2 node info /node_name
   ```

3. **Inspect topics**:
   ```bash
   ros2 topic list
   ros2 topic echo /topic_name
   ros2 topic hz /topic_name  # Check message rate
   ```

4. **Check services and actions**:
   ```bash
   ros2 service list
   ros2 action list
   ```

5. **View logs**:
   ```bash
   ros2 run your_package your_node --ros-args --log-level debug
   ```

### Common Command Reference

```bash
# Environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build
cd ~/ros2_ws
colcon build
colcon build --packages-select your_package

# Run nodes
ros2 run package_name node_name

# Launch files
ros2 launch package_name launch_file.launch.py

# Topics
ros2 topic list
ros2 topic echo /topic_name
ros2 topic pub /topic_name std_msgs/msg/String "data: 'test'"

# Nodes
ros2 node list
ros2 node info /node_name

# Parameters
ros2 param list
ros2 param get /node_name param_name
ros2 param set /node_name param_name value

# Services
ros2 service list
ros2 service call /service_name package/srv/ServiceType "{request_field: value}"

# Actions
ros2 action list
ros2 action send_goal /action_name package/action/ActionType "{goal_field: value}"

# Packages
ros2 pkg list
ros2 pkg prefix package_name

# Interfaces
ros2 interface list
ros2 interface show std_msgs/msg/String
```

### Getting More Help

1. **Check lesson-specific troubleshooting sections** - Each lesson has a "Common Mistakes" section
2. **ROS 2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
3. **ROS Answers**: [https://answers.ros.org/](https://answers.ros.org/)
4. **Course Discussion Forum**: Check course platform for Q&A
5. **Enable debug logging**:
   ```bash
   ros2 run your_package your_node --ros-args --log-level debug
   ```

---

## Related Resources

- [Week 3 Lesson 1: ROS 2 Architecture](./week-3-lesson-1-ros2-architecture#common-mistakes-and-troubleshooting)
- [Week 3 Lesson 2: Nodes and Packages](./week-3-lesson-2-nodes-packages#common-mistakes)
- [Week 4 Lesson 1: Services and Actions](./week-4-lesson-1-services-actions#common-mistakes)
- [Week 4 Lesson 2: Building Packages](./week-4-lesson-2-building-packages#common-mistakes)
- [Week 5 Lesson 1: Launch Files](./week-5-lesson-1-launch-files#common-mistakes)
- [Week 5 Lesson 2: URDF for Humanoids](./week-5-lesson-2-urdf-humanoids#common-mistakes)
- [Version Compatibility Guide](./version-compatibility)
- [Software Setup Guide](../setup/software-setup)

---

**Last Updated**: December 2024
**Applies To**: Module 1 (Weeks 3-5) - ROS 2 Humble
