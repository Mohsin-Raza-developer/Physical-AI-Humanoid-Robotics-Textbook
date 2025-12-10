---
title: Glossary
sidebar_label: Glossary
sidebar_position: 1
description: Technical terms and definitions for Physical AI and Humanoid Robotics
---

# Glossary of Terms

Quick reference for technical terms used throughout the Physical AI Humanoid Robotics course.

## A

**Action (ROS 2)**
: Long-running tasks in ROS 2 with feedback and cancellation. Unlike services, actions provide intermediate feedback during execution.

**Actuator**
: A mechanical component that moves or controls a mechanism (motors, servos, hydraulics).

**ADAS (Advanced Driver Assistance Systems)**
: Vehicle systems that use sensors and AI for safety features like lane keeping and collision avoidance.

## B

**Bipedal Locomotion**
: Walking or movement using two legs, a key challenge in humanoid robotics requiring balance and stability control.

**BNO055**
: A 9-axis Inertial Measurement Unit (IMU) combining accelerometer, gyroscope, and magnetometer.

## C

**CapEx (Capital Expenditure)**
: Upfront costs for purchasing equipment (workstations, robots) vs. OpEx (operational costs).

**Colcon**
: The build system for ROS 2 packages, replacing catkin from ROS 1.

**Computer Vision**
: Field of AI enabling computers to interpret and understand visual information from images and videos.

**CUDA (Compute Unified Device Architecture)**
: NVIDIA's parallel computing platform for GPU-accelerated applications.

## D

**Depth Camera**
: Camera that measures distance to objects (e.g., Intel RealSense D435i) using stereo vision or structured light.

**Digital Twin**
: A virtual replica of a physical robot used for simulation, testing, and training before real-world deployment.

**DOF (Degrees of Freedom)**
: Number of independent movements a robot joint or system can perform. Humanoids typically have 20-40+ DOF.

## E

**Embodied AI**
: Artificial intelligence systems deployed in physical robots that interact with the real world through sensors and actuators.

**End Effector**
: The device at the end of a robotic arm (gripper, hand, tool) that interacts with objects.

## F

**Forward Kinematics**
: Calculating the position and orientation of a robot's end effector given joint angles.

## G

**Gazebo**
: Open-source physics-based robot simulator widely used with ROS for testing before hardware deployment.

**Grasping**
: Robot capability to pick up and manipulate objects using grippers or hands.

**GPT (Generative Pre-trained Transformer)**
: Large language models (like GPT-4) used for natural language understanding in conversational robotics.

## H

**HRI (Human-Robot Interaction)**
: Field studying how humans and robots communicate and collaborate effectively.

**Humanoid Robot**
: Robot with a human-like body structure (head, torso, two arms, two legs).

## I

**IMU (Inertial Measurement Unit)**
: Sensor measuring acceleration, rotation, and magnetic field - critical for robot balance and localization.

**Inference**
: Running a trained AI model to make predictions (vs. training the model).

**Inverse Kinematics**
: Calculating joint angles needed to position a robot's end effector at a desired location.

**Isaac ROS**
: NVIDIA's hardware-accelerated ROS 2 packages for perception, navigation, and manipulation.

**Isaac Sim**
: NVIDIA's photorealistic robot simulator built on Omniverse, optimized for AI training.

## J

**Jetson**
: NVIDIA's embedded computing platform for edge AI (Jetson Orin Nano, Orin NX, etc.).

**Joint**
: Connection point between robot links allowing movement (revolute, prismatic, spherical).

## K

**Kinematics**
: Study of robot motion without considering forces (positions, velocities, accelerations).

## L

**Launch File**
: ROS 2 file (Python or XML) that starts multiple nodes with parameters and configurations.

**Lidar (Light Detection and Ranging)**
: Sensor using laser pulses to measure distances and create 3D maps of environments.

**Locomotion**
: Robot movement capability (walking, running, jumping for humanoids).

## M

**Manipulation**
: Robot's ability to interact with and move objects using arms and grippers.

**MoveIt**
: ROS framework for motion planning, manipulation, and collision avoidance (MoveIt 2 for ROS 2).

## N

**Nav2**
: ROS 2 navigation stack for autonomous mobile robot navigation with obstacle avoidance.

**Node (ROS 2)**
: Independent process that performs computation and communicates via topics, services, or actions.

## O

**Omniverse**
: NVIDIA's platform for 3D simulation and collaboration, foundation for Isaac Sim.

**ONNX (Open Neural Network Exchange)**
: Standard format for representing machine learning models across frameworks.

**OpEx (Operational Expenditure)**
: Recurring costs (cloud instances, maintenance) vs. CapEx (upfront equipment purchases).

## P

**Package (ROS 2)**
: Organizational unit containing nodes, libraries, configuration files, and launch files.

**Perception**
: Robot's ability to sense and interpret its environment (vision, LIDAR, depth sensing).

**Physical AI**
: AI systems deployed in physical embodiments (robots) that interact with the real world.

**Publish-Subscribe**
: ROS 2 communication pattern where nodes publish data to topics that other nodes subscribe to.

## Q

**QoS (Quality of Service)**
: ROS 2 policies controlling message reliability, durability, and delivery guarantees.

## R

**RealSense**
: Intel's line of depth cameras (D435i, D455) with RGB, depth, and IMU sensors.

**Reinforcement Learning (RL)**
: Machine learning approach where agents learn optimal behaviors through trial and error.

**RGB-D**
: Image data combining RGB color with Depth information.

**Robot Description**
: Formal specification of robot geometry, links, joints, and sensors (URDF, SDF, USD).

**ROS (Robot Operating System)**
: Middleware framework for robot software development. ROS 2 is the latest version.

**rviz2**
: ROS 2 visualization tool for displaying sensor data, robot models, and trajectories.

## S

**SDF (Simulation Description Format)**
: XML format for describing simulation worlds and robots in Gazebo.

**Sensor Fusion**
: Combining data from multiple sensors (camera, LIDAR, IMU) for more accurate perception.

**Service (ROS 2)**
: Request-response communication pattern for quick, synchronous operations.

**Sim-to-Real**
: Process of training robots in simulation and transferring learned behaviors to physical hardware.

**SLAM (Simultaneous Localization and Mapping)**
: Robot capability to build maps while tracking its own position.

## T

**TensorRT**
: NVIDIA's SDK for high-performance deep learning inference optimization.

**Topic (ROS 2)**
: Named channel for asynchronous data streaming between nodes.

**TOPS (Tera Operations Per Second)**
: Measure of AI processing performance (Jetson Orin Nano: 40 TOPS).

## U

**Unity**
: Game engine used for high-fidelity robot simulation with Unity Robotics Hub.

**URDF (Unified Robot Description Format)**
: XML format for describing robot kinematics, dynamics, and visualization in ROS.

**USD (Universal Scene Description)**
: Pixar's format for 3D scenes, used in Isaac Sim for robot and environment representation.

## V

**VLA (Vision-Language-Action)**
: AI models that combine visual perception, natural language understanding, and physical action planning.

**VSLAM (Visual SLAM)**
: SLAM using camera images (vs. LIDAR SLAM).

## W

**Whisper**
: OpenAI's speech recognition model for converting voice to text.

**Workspace (ROS 2)**
: Directory structure containing ROS 2 packages, build files, and installation.

## X

**Xacro**
: XML macro language for simplifying URDF files with variables and reusable components.

## Y

**YOLO (You Only Look Once)**
: Real-time object detection algorithm family (YOLOv5, YOLOv8) for computer vision.

## Z

**ZMP (Zero Moment Point)**
: Concept in bipedal robotics for maintaining balance by controlling where weight is distributed.

---

## Related Resources

- [References](./references) - Academic papers and documentation
- [Additional Reading](./additional-reading) - Tutorials and learning materials
