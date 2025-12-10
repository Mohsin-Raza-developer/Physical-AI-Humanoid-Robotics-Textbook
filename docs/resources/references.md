---
title: References
sidebar_label: References
sidebar_position: 2
description: Academic papers, documentation, and official resources
---

# References and Citations

Curated list of academic papers, official documentation, and authoritative resources referenced throughout the course.

## Official Documentation

### ROS 2

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) - Official ROS 2 documentation
- [ROS 2 Design Documents](https://design.ros2.org/) - Architectural decisions and rationale
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html) - Step-by-step guides

### NVIDIA Isaac

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) - Official Isaac Sim guide
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/) - Hardware-accelerated ROS packages
- [NVIDIA Omniverse](https://docs.omniverse.nvidia.com/) - Platform documentation

### Simulation Platforms

- [Gazebo Documentation](https://gazebosim.org/docs) - Gazebo simulator reference
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - Unity-ROS integration
- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html) - Motion planning framework

### Hardware

- [Jetson Developer Guide](https://developer.nvidia.com/embedded/learn/getting-started-jetson) - NVIDIA Jetson setup
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense) - Depth camera SDK
- [Unitree Robotics Documentation](https://www.unitree.com/support/) - Robot platform docs

## Foundational Papers

### Humanoid Robotics

1. Kajita, S., et al. (2003). "Biped walking pattern generation by using preview control of zero-moment point." *IEEE International Conference on Robotics and Automation*.
   - Foundation for bipedal walking control using ZMP

2. Vukobratović, M., & Borovac, B. (2004). "Zero-moment point—thirty five years of its life." *International Journal of Humanoid Robotics*.
   - Historical overview of ZMP concept

3. Hirose, M., & Ogawa, K. (2007). "Honda humanoid robots development." *Philosophical Transactions of the Royal Society A*.
   - Development history of ASIMO and humanoid research

### Reinforcement Learning for Robotics

4. Schulman, J., et al. (2017). "Proximal policy optimization algorithms." *arXiv preprint arXiv:1707.06347*.
   - PPO algorithm widely used in robot control

5. Haarnoja, T., et al. (2018). "Soft actor-critic: Off-policy maximum entropy deep reinforcement learning with a stochastic actor." *ICML*.
   - SAC algorithm for continuous control

6. OpenAI, et al. (2019). "Solving Rubik's Cube with a robot hand." *arXiv preprint arXiv:1910.07113*.
   - Landmark sim-to-real transfer demonstration

### Sim-to-Real Transfer

7. Tobin, J., et al. (2017). "Domain randomization for transferring deep neural networks from simulation to the real world." *IROS*.
   - Key technique for bridging sim-to-real gap

8. Tan, J., et al. (2018). "Sim-to-real: Learning agile locomotion for quadruped robots." *RSS*.
   - Successful transfer of locomotion policies

9. Peng, X. B., et al. (2020). "Learning agile robotic locomotion skills by imitating animals." *RSS*.
   - Imitation learning from animal motion

## Vision-Language-Action Models

### Vision-Language Models

10. Radford, A., et al. (2021). "Learning transferable visual models from natural language supervision." *ICML* (CLIP).
    - Foundation for vision-language understanding

11. Alayrac, J. B., et al. (2022). "Flamingo: a visual language model for few-shot learning." *NeurIPS*.
    - Multi-modal vision-language architecture

### Robotics Foundation Models

12. Brohan, A., et al. (2022). "RT-1: Robotics transformer for real-world control at scale." *arXiv preprint arXiv:2212.06817*.
    - Google's robotics transformer model

13. Brohan, A., et al. (2023). "RT-2: Vision-language-action models transfer web knowledge to robotic control." *CoRL*.
    - Scaling vision-language models to robotics

14. Octo Model Team (2024). "Octo: An open-source generalist robot policy." *arXiv preprint*.
    - Open-source VLA model

### Embodied AI

15. Deitke, M., et al. (2022). "Behavior-1K: A Benchmark for Embodied AI with 1,000 Everyday Activities and Realistic Simulation." *CoRL*.
    - Comprehensive embodied AI benchmark

16. Szot, A., et al. (2021). "Habitat 2.0: Training home assistants to rearrange their habitat." *NeurIPS*.
    - Simulation platform for embodied AI research

## Perception and SLAM

### Visual SLAM

17. Mur-Artal, R., et al. (2015). "ORB-SLAM: a versatile and accurate monocular SLAM system." *IEEE Transactions on Robotics*.
    - Influential monocular SLAM system

18. Campos, C., et al. (2021). "ORB-SLAM3: An accurate open-source library for visual, visual-inertial and multi-map SLAM." *IEEE Transactions on Robotics*.
    - Multi-sensor SLAM framework

### Object Detection

19. Redmon, J., et al. (2016). "You only look once: Unified, real-time object detection." *CVPR* (YOLO).
    - Real-time object detection breakthrough

20. He, K., et al. (2017). "Mask R-CNN." *ICCV*.
    - Instance segmentation for robotics

## Manipulation and Grasping

21. Levine, S., et al. (2018). "Learning hand-eye coordination for robotic grasping with deep learning and large-scale data collection." *International Journal of Robotics Research*.
    - Data-driven grasping approaches

22. Mahler, J., et al. (2017). "Dex-Net 2.0: Deep learning to plan robust grasps with synthetic point clouds and analytic grasp metrics." *RSS*.
    - Grasp planning with deep learning

23. Morrison, D., et al. (2020). "Learning robust, real-time, reactive robotic grasping." *International Journal of Robotics Research*.
    - Reactive grasping strategies

## Natural Language Processing for Robotics

24. Tellex, S., et al. (2011). "Understanding natural language commands for robotic navigation and mobile manipulation." *AAAI*.
    - Natural language grounding for robots

25. Ahn, M., et al. (2022). "Do as I can, not as I say: Grounding language in robotic affordances." *CoRL* (SayCan).
    - LLM-based robot task planning

26. Huang, W., et al. (2022). "Language models as zero-shot planners: Extracting actionable knowledge for embodied agents." *ICML*.
    - Using LLMs for robot planning

## Books and Textbooks

### Robotics Fundamentals

- Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control*. Springer.
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson.
- Thrun, S., et al. (2005). *Probabilistic Robotics*. MIT Press.

### ROS and Robot Programming

- Quigley, M., et al. (2015). *Programming Robots with ROS*. O'Reilly Media.
- Martinez, A., & Fernández, E. (2021). *A Concise Introduction to Robot Programming with ROS2*. CRC Press.

### AI and Machine Learning

- Sutton, R. S., & Barto, A. G. (2018). *Reinforcement Learning: An Introduction* (2nd ed.). MIT Press.
- Goodfellow, I., et al. (2016). *Deep Learning*. MIT Press.

## Standards and Specifications

- **URDF Specification**: [ROS URDF XML Spec](http://wiki.ros.org/urdf/XML)
- **SDF Specification**: [SDF Format Spec](http://sdformat.org/)
- **USD Specification**: [Universal Scene Description](https://openusd.org/release/index.html)
- **DDS (Data Distribution Service)**: [OMG DDS Standard](https://www.dds-foundation.org/)

## Industry Reports and Whitepapers

- NVIDIA (2023). "Isaac Sim for Robotics Development" - Technical whitepaper
- Boston Dynamics (2022). "Atlas: The Next Generation" - Technical overview
- OpenAI (2023). "GPT-4 Technical Report" - Model capabilities and safety

## Open-Source Repositories

### Robot Models and Datasets

- [MuJoCo Menagerie](https://github.com/deepmind/mujoco_menagerie) - Robot model collection
- [RoboNet](https://www.robonet.wiki/) - Large-scale robot manipulation dataset
- [Bridge Data](https://sites.google.com/view/bridgedata) - Robot manipulation benchmark

### Software Libraries

- [PyRobot](https://github.com/facebookresearch/pyrobot) - Python API for robot control
- [RobotBenchmark](https://github.com/google-research/robotics_transformer) - Benchmarking tools
- [Isaac Gym](https://developer.nvidia.com/isaac-gym) - GPU-accelerated RL environments

## Conference and Journal Resources

### Key Conferences

- **ICRA** (International Conference on Robotics and Automation) - IEEE flagship robotics conference
- **IROS** (International Conference on Intelligent Robots and Systems) - IEEE robotics conference
- **RSS** (Robotics: Science and Systems) - Leading robotics research conference
- **CoRL** (Conference on Robot Learning) - Machine learning for robotics
- **NeurIPS** (Neural Information Processing Systems) - AI/ML conference with robotics track

### Key Journals

- *IEEE Transactions on Robotics*
- *International Journal of Robotics Research*
- *Autonomous Robots*
- *Robotics and Autonomous Systems*

## Online Courses

- [Coursera: Modern Robotics](https://www.coursera.org/specializations/modernrobotics) - Northwestern University
- [edX: Robotics MicroMasters](https://www.edx.org/micromasters/pennx-robotics) - University of Pennsylvania
- [Udacity: Robotics Nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209)

---

## Citation Style

This course uses IEEE citation style. For citing these references in your assignments:

```
[1] S. Kajita et al., "Biped walking pattern generation by using preview
    control of zero-moment point," in Proc. IEEE Int. Conf. Robot.
    Autom. (ICRA), 2003, pp. 1620-1626.
```

## Related Resources

- [Glossary](./glossary) - Technical term definitions
- [Additional Reading](./additional-reading) - Tutorials and learning materials
