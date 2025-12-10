---
title: Gazebo Simulation Implementation Project
sidebar_label: Module 2 Assessment
sidebar_position: 1
description: Module 2 assessment project for Gazebo and Unity simulation
tags: [gazebo, unity, simulation, assessment, project, week-7]
---

# Module 2 Assessment: Gazebo Simulation Implementation Project

## Overview

This project assesses your ability to create realistic robot simulations using Gazebo, URDF/SDF modeling, and ROS 2 integration. You will build a complete simulated environment with a custom robot, sensors, and control systems.

**Duration:** Weeks 6-7
**Weight:** 20% of final grade
**Submission:** End of Week 7

## Learning Objectives

By completing this project, you will demonstrate your ability to:

1. Create custom Gazebo worlds with obstacles and environmental features
2. Design and implement robot models using URDF and SDF formats
3. Integrate realistic sensors (LIDAR, cameras, IMU) into simulations
4. Connect Gazebo to ROS 2 for robot control
5. Visualize simulation data in RViz2
6. Apply physics simulation for realistic robot behavior

## Project Requirements

### 1. Custom Gazebo World (15 points)

**File:** `worlds/robot_arena.world`

Create a Gazebo world that includes:

- Ground plane with textured surface
- At least 5 obstacles of different shapes (boxes, cylinders, spheres)
- Physical barriers creating a maze or navigation challenge
- Lighting (at least 2 light sources)
- Optional: Models from Gazebo model database

**Requirements:**
- World must be at least 10m x 10m
- Obstacles should create navigation challenges
- Use SDF format
- Include spawn point for your robot

### 2. Robot Model (25 points)

**Files:** `urdf/simulation_robot.urdf` or `sdf/simulation_robot.sdf`

Design a mobile robot with the following specifications:

**Physical Structure:**
- Differential drive base (2 wheels + 1 caster)
- Chassis: 0.5m x 0.4m x 0.2m
- Wheels: 0.1m radius
- Sensor mast: 0.3m tall

**Sensors (must include all):**
- **LIDAR:** 360-degree laser scanner
  - Range: 0.1m to 10m
  - Angular resolution: 1 degree
  - Update rate: 10 Hz
- **RGB Camera:** Forward-facing
  - Resolution: 640x480
  - FOV: 60 degrees
  - Update rate: 30 Hz
- **Depth Camera:** (Optional but recommended)
  - Resolution: 320x240
  - Range: 0.5m to 5m
- **IMU:** 6-DOF inertial measurement unit
  - Accelerometer and gyroscope
  - Update rate: 100 Hz

**Physics Properties:**
- Total mass: ~5kg (distributed appropriately)
- Inertial properties for all links
- Friction coefficients for wheels and caster
- Collision geometries matching visual

### 3. Gazebo Plugins Integration (20 points)

Configure and integrate the following Gazebo ROS 2 plugins:

**Differential Drive Controller:**
```xml
<plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
  <update_rate>50</update_rate>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>true</publish_wheel_tf>
</plugin>
```

**Sensor Plugins:**
- `libgazebo_ros_ray_sensor.so` for LIDAR
- `libgazebo_ros_camera.so` for RGB camera
- `libgazebo_ros_imu_sensor.so` for IMU

### 4. ROS 2 Integration (15 points)

Create a ROS 2 package `<yourname>_gazebo_sim` with:

**Teleop Control Node:**
- Subscribe to `/cmd_vel`
- Allow keyboard control of the robot
- Implement safety limits (max speed, acceleration)

**Sensor Data Logger:**
- Subscribe to all sensor topics
- Log data to CSV files with timestamps
- Calculate and display statistics (min, max, average)

**Launch File:** `simulation.launch.py`
- Launch Gazebo with custom world
- Spawn robot at specified position
- Start RViz2 with appropriate visualization
- Launch teleop and logger nodes

### 5. Navigation Challenge (15 points)

Implement autonomous navigation behavior:

**File:** `navigation_controller.py`

Create a node that:
- Reads LIDAR data to detect obstacles
- Implements simple wall-following algorithm OR
- Implements obstacle avoidance with goal seeking
- Publishes velocity commands to `/cmd_vel`
- Logs navigation events (turns, stops, goal reached)

**Test Scenario:**
- Place robot at starting position
- Define goal position in the world
- Robot should navigate from start to goal avoiding obstacles

### 6. Visualization and Analysis (5 points)

Configure RViz2 to display:

- Robot model (URDF)
- LIDAR scan data
- Camera feed
- Odometry path
- TF tree showing all transforms
- Map (if using SLAM - optional)

**File:** `config/rviz_config.rviz`

### 7. Unity Integration (Bonus: +10 points)

Integrate Unity for enhanced visualization:

- Import robot URDF using Unity Robotics Hub
- Create Unity scene matching Gazebo world
- Establish ROS-Unity communication
- Display camera feed in Unity
- Demonstrate synchronized movement between Gazebo and Unity

## Deliverables

Submit the following via the course portal:

1. **ROS 2 Package:** Complete source code (zip file) including:
   - URDF/SDF files
   - Gazebo world file
   - Launch files
   - Python nodes
   - Configuration files
   - README.md

2. **Demo Video:** 3-5 minute recording showing:
   - Launching the simulation
   - Robot model in Gazebo and RViz2
   - Teleop control demonstration
   - Autonomous navigation challenge
   - Sensor data visualization
   - Unity integration (if attempted)

3. **Technical Report:** 3-4 page PDF including:
   - System architecture diagram
   - URDF/SDF design decisions and calculations
   - Sensor configuration and calibration
   - Navigation algorithm explanation
   - Performance analysis (frame rate, sensor accuracy)
   - Challenges and solutions
   - Screenshots of Gazebo, RViz2, and Unity (if applicable)

4. **Sensor Data:** CSV files with at least 30 seconds of logged data

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Custom World** | 15 | Appropriate complexity, obstacles, lighting, SDF format |
| **Robot Model** | 25 | Complete sensors, proper physics, valid URDF/SDF |
| **Gazebo Plugins** | 20 | All plugins configured, sensors publishing data |
| **ROS 2 Integration** | 15 | Launch system, teleop, data logging functional |
| **Navigation** | 15 | Autonomous behavior, obstacle avoidance working |
| **Visualization** | 5 | RViz2 properly configured and functional |
| **Documentation** | 5 | Clear README, video, technical report |
| **Bonus: Unity** | +10 | Successful Unity integration and synchronization |
| **Total** | 100 (+10) | |

### Grade Breakdown

- **90-100:** Exceptional - All requirements exceeded, clean code, excellent documentation, bonus completed
- **80-89:** Proficient - All core requirements met, good documentation, minor issues
- **70-79:** Developing - Most requirements met, navigation partially working, adequate documentation
- **60-69:** Beginning - Basic simulation works, significant features missing
- **Below 60:** Incomplete - Major components not functional

## Testing Checklist

Before submission, verify:

- [ ] Gazebo launches without errors: `ros2 launch <yourname>_gazebo_sim simulation.launch.py`
- [ ] Robot spawns correctly in the world
- [ ] All sensor topics publish data: `ros2 topic list`
- [ ] Teleop control moves robot: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- [ ] LIDAR data visible in RViz2
- [ ] Camera feed displays in RViz2 or image viewer
- [ ] Autonomous navigation avoids obstacles
- [ ] No physics explosions or instabilities
- [ ] Odometry TF frames publish correctly
- [ ] Simulation runs at acceptable frame rate (>20 FPS)

## Tips for Success

1. **Start with Simple World:** Test robot in empty world first, add complexity later
2. **Validate URDF/SDF:** Use `check_urdf` and `gz sdf` to validate syntax
3. **Tune Physics:** Adjust mass, inertia, friction for stable simulation
4. **Monitor Performance:** Use `rqt_graph` to visualize node connections
5. **Save Configuration:** Export RViz2 config to preserve visualization setup
6. **Version Control:** Use Git to track changes and prevent data loss

## Common Pitfalls to Avoid

- **Physics Instabilities:** Robot "explodes" due to incorrect inertia or collision geometries
  - Solution: Use appropriate mass distribution and collision shapes
- **Sensor Not Publishing:** Plugin configured incorrectly
  - Solution: Check topic names, update rates, and plugin parameters
- **Poor Performance:** Simulation runs slowly
  - Solution: Reduce sensor resolution, simplify collision meshes, optimize world
- **TF Errors:** Missing or incorrect transforms
  - Solution: Ensure `robot_state_publisher` is running, check URDF joint definitions
- **Robot Drifts:** Wheel odometry inaccurate
  - Solution: Tune wheel separation and diameter parameters

## Example Launch Command

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select <yourname>_gazebo_sim
source install/setup.bash

# Launch simulation
ros2 launch <yourname>_gazebo_sim simulation.launch.py world:=robot_arena

# In separate terminals:
# View sensor data
ros2 topic echo /scan

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Start autonomous navigation
ros2 run <yourname>_gazebo_sim navigation_controller
```

## Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [SDF Format Specification](http://sdformat.org/)
- [Gazebo ROS 2 Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

## Submission

**Due Date:** End of Week 7 (see course schedule)
**Submit to:** Course portal
**Late Policy:** 10% deduction per day, maximum 3 days late

## Academic Integrity

This is an individual project. You may use online resources and tutorials, but all code integration and design decisions must be your own. Cite any external models or code snippets used.

---

**Questions?** Contact the instructor or TA during office hours or via the course forum.
