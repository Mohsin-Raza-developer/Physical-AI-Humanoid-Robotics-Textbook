---
title: ROS 2 Package Development Project
sidebar_label: Module 1 Assessment
sidebar_position: 1
description: Module 1 assessment project for ROS 2 fundamentals
tags: [ros2, assessment, project, week-5]
---

# Module 1 Assessment: ROS 2 Package Development Project

## Overview

This project assesses your understanding of ROS 2 fundamentals by requiring you to build a complete ROS 2 package that demonstrates nodes, topics, services, actions, launch files, and URDF integration.

**Duration:** Weeks 3-5
**Weight:** 15% of final grade
**Submission:** End of Week 5

## Learning Objectives

By completing this project, you will demonstrate your ability to:

1. Create a properly structured ROS 2 package using colcon
2. Implement publisher and subscriber nodes
3. Design and implement custom service and action servers
4. Write launch files with parameters and node configuration
5. Create or modify a URDF model for a robot
6. Document your code and test functionality

## Project Requirements

### 1. Package Structure (10 points)

Create a ROS 2 package named `<yourname>_robot_controller` with:

- Proper `package.xml` with all dependencies
- `CMakeLists.txt` or `setup.py` configured correctly
- Organized directory structure:
  ```
  <yourname>_robot_controller/
  ├── <yourname>_robot_controller/
  │   ├── __init__.py
  │   ├── sensor_publisher.py
  │   ├── control_node.py
  │   ├── emergency_stop_service.py
  │   └── navigation_action_server.py
  ├── launch/
  │   ├── robot_system.launch.py
  │   └── params.yaml
  ├── urdf/
  │   └── my_robot.urdf
  ├── config/
  │   └── controller_params.yaml
  ├── package.xml
  ├── setup.py
  └── README.md
  ```

### 2. Sensor Publisher Node (15 points)

**File:** `sensor_publisher.py`

Create a node that publishes simulated sensor data:

- **Topic:** `/sensors/lidar` (type: `sensor_msgs/LaserScan`)
- **Topic:** `/sensors/camera` (type: `sensor_msgs/Image`)
- **Topic:** `/sensors/imu` (type: `sensor_msgs/Imu`)
- **Frequency:** 10 Hz for all sensors
- **Requirements:**
  - Use a single node with multiple publishers
  - Simulate realistic sensor data (random noise, valid ranges)
  - Implement proper QoS settings for each sensor type

### 3. Control Subscriber Node (15 points)

**File:** `control_node.py`

Create a node that subscribes to sensor data and publishes control commands:

- **Subscribes to:** `/sensors/lidar`, `/sensors/imu`
- **Publishes to:** `/cmd_vel` (type: `geometry_msgs/Twist`)
- **Logic:**
  - If LIDAR detects obstacle < 1.0m ahead, stop (velocity = 0)
  - If obstacle is 1.0-2.0m ahead, slow down (velocity *= 0.5)
  - Otherwise, move forward at 0.5 m/s
  - Use IMU data to log orientation (roll, pitch, yaw)

### 4. Service Server (15 points)

**File:** `emergency_stop_service.py`

Create a service that provides emergency stop functionality:

- **Service Name:** `/emergency_stop`
- **Service Type:** `std_srvs/SetBool`
- **Behavior:**
  - When `data=True`, publish stop command to `/cmd_vel` and return success
  - When `data=False`, resume normal operation
  - Log all emergency stop requests with timestamps

**Test Command:**
```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```

### 5. Action Server (20 points)

**File:** `navigation_action_server.py`

Create an action server for goal-based navigation:

- **Action Name:** `/navigate_to_goal`
- **Action Type:** `geometry_msgs/action/Navigate` (you may use a standard action or create a custom one)
- **Behavior:**
  - Accept a goal position (x, y, theta)
  - Publish periodic feedback showing distance remaining
  - Simulate movement by publishing to `/cmd_vel`
  - Return success when "within 0.1m of goal"
  - Allow preemption (cancellation) of goals

**Test Command:**
```bash
ros2 action send_goal /navigate_to_goal geometry_msgs/action/Navigate "{goal: {x: 5.0, y: 3.0, theta: 1.57}}"
```

### 6. Launch File System (10 points)

**File:** `robot_system.launch.py`

Create a launch file that:

- Starts all nodes (sensor_publisher, control_node, emergency_stop_service, navigation_action_server)
- Loads parameters from `params.yaml`
- Launches RViz2 with a custom configuration
- Publishes the robot URDF to `/robot_description`
- Starts `robot_state_publisher` node

**Launch Command:**
```bash
ros2 launch <yourname>_robot_controller robot_system.launch.py
```

### 7. URDF Robot Model (10 points)

**File:** `my_robot.urdf`

Create or modify a simple robot URDF with:

- Base link (chassis)
- At least 2 wheels (left and right)
- A sensor link (LIDAR or camera)
- Proper joint definitions (continuous for wheels)
- Visual and collision geometries
- Inertial properties

**Validation:**
```bash
check_urdf my_robot.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat my_robot.urdf)"
```

### 8. Documentation (5 points)

**File:** `README.md`

Provide clear documentation including:

- Package overview and purpose
- Installation instructions
- How to build and run the package
- Description of each node and its functionality
- Example commands for testing
- Known issues or limitations

## Deliverables

Submit the following via the course portal:

1. **Source Code:** Complete ROS 2 package directory (zip file)
2. **README.md:** Comprehensive documentation
3. **Demo Video:** 2-3 minute screen recording showing:
   - Launch file starting all nodes
   - `ros2 topic list` and `ros2 node list` output
   - RViz2 displaying the robot model
   - Service call demonstration
   - Action goal execution
4. **Written Report:** 2-page PDF including:
   - Architecture diagram of your system
   - Challenges faced and how you solved them
   - Lessons learned

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Package Structure** | 10 | Proper organization, dependencies, build system configuration |
| **Sensor Publisher** | 15 | Multiple publishers, realistic data, proper QoS |
| **Control Node** | 15 | Correct subscription logic, obstacle avoidance behavior |
| **Service Server** | 15 | Functional service, correct request/response handling |
| **Action Server** | 20 | Goal handling, feedback, preemption support |
| **Launch File** | 10 | All nodes launch correctly, parameters loaded |
| **URDF Model** | 10 | Valid robot description, visualizes in RViz2 |
| **Documentation** | 5 | Clear README, demo video, written report |
| **Total** | 100 | |

### Grade Breakdown

- **90-100:** Exceptional - All requirements met, code is clean and well-documented, advanced features implemented
- **80-89:** Proficient - All core requirements met, minor issues in documentation or edge cases
- **70-79:** Developing - Most requirements met, some functionality incomplete or buggy
- **60-69:** Beginning - Basic structure present, significant functionality missing
- **Below 60:** Incomplete - Major requirements not met

## Testing Checklist

Before submission, verify:

- [ ] Package builds without errors: `colcon build --packages-select <yourname>_robot_controller`
- [ ] All nodes start successfully via launch file
- [ ] `ros2 topic list` shows all expected topics
- [ ] `ros2 topic echo /sensors/lidar` displays sensor data
- [ ] `ros2 topic echo /cmd_vel` shows control commands
- [ ] Emergency stop service responds correctly
- [ ] Navigation action accepts goals and provides feedback
- [ ] Robot URDF displays properly in RViz2
- [ ] No errors or warnings in terminal output
- [ ] README accurately describes the system

## Tips for Success

1. **Start Early:** Begin in Week 3, don't wait until Week 5
2. **Test Incrementally:** Build one node at a time and test before moving on
3. **Use ROS 2 Tools:** Familiarize yourself with `ros2 node`, `ros2 topic`, `ros2 service`, `ros2 action` commands
4. **Check Examples:** Review ROS 2 tutorials and example packages
5. **Ask for Help:** Use office hours or discussion forums if stuck
6. **Version Control:** Use Git to track your progress

## Common Pitfalls to Avoid

- Forgetting to source your workspace: `source install/setup.bash`
- Incorrect QoS settings causing message drops
- Not handling node shutdown gracefully (use `rclpy.spin()` correctly)
- URDF syntax errors (use `check_urdf` tool)
- Hardcoding values instead of using parameters
- Not testing on a clean build before submission

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Creating a ROS 2 Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Writing a Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [Launch File Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)

## Submission

**Due Date:** End of Week 5 (see course schedule for exact date)
**Submit to:** Course portal (link provided in Canvas/LMS)
**Late Policy:** 10% deduction per day, maximum 3 days late

## Academic Integrity

This is an individual project. You may discuss high-level concepts with classmates, but all code must be your own. Copying code from online sources or classmates will result in a zero and academic misconduct report.

---

**Questions?** Contact the instructor or TA during office hours or via the course forum.
