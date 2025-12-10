---
title: Capstone - Autonomous Conversational Humanoid System
sidebar_label: Capstone Project
sidebar_position: 1
description: Final capstone project for the Physical AI Humanoid Robotics course
tags: [vla, capstone, humanoid, conversational-ai, final-project, week-13]
---

# Capstone Project: Autonomous Conversational Humanoid System

## Overview

This capstone project represents the culmination of your learning journey through Physical AI and Humanoid Robotics. You will integrate all course concepts—ROS 2, simulation, AI perception, and vision-language-action models—to build a voice-controlled autonomous humanoid robot capable of understanding natural language, navigating environments, perceiving objects, and performing manipulation tasks.

**Duration:** Weeks 11-13 (3 weeks)
**Weight:** 40% of final grade
**Submission:** End of Week 13
**Team Size:** 1-2 students (individual or pair)

## Learning Objectives

By completing this capstone, you will demonstrate mastery of:

1. Full-stack robot system integration (perception, cognition, action)
2. Multimodal AI (vision, language, and action models)
3. Humanoid kinematics and locomotion control
4. Human-robot interaction design
5. Real-time system performance optimization
6. Professional technical communication and presentation

## Project Vision

**Scenario:** "Robot, please bring me the red cup from the kitchen table."

Your humanoid robot must:
1. **Understand** the command via speech recognition (Whisper)
2. **Plan** the task using natural language reasoning (GPT-4)
3. **Navigate** to the kitchen avoiding obstacles (Nav2 + Isaac ROS)
4. **Perceive** and identify the red cup (YOLO/Detectron2)
5. **Manipulate** by grasping the cup safely (MoveIt 2)
6. **Return** to the user and hand over the cup
7. **Confirm** completion verbally (text-to-speech)

## System Requirements

### 1. Simulation Platform (10 points)

**Environment:** NVIDIA Isaac Sim

Create or adapt a photorealistic home/office environment:
- At least 3 distinct rooms (living room, kitchen, bedroom)
- Furniture: tables, chairs, shelves
- Navigable space with obstacles
- Target objects placed on surfaces
- Humanoid robot (Unitree G1, custom, or similar)

**Acceptance Criteria:**
- Scene loads in &lt;30 seconds
- Robot can walk/move through all rooms
- Objects are graspable (proper physics properties)

### 2. Speech Interface (10 points)

**Input:** OpenAI Whisper or Google Speech-to-Text
**Output:** Text-to-Speech (gTTS, AWS Polly, or similar)

Implement voice command processing:
- Microphone input (ReSpeaker or similar)
- Real-time speech-to-text conversion
- Natural language command parsing
- Voice feedback for status updates

**Test Commands (must support all):**
- "Go to the kitchen"
- "Find the red cup"
- "Bring me the blue ball"
- "What do you see?"
- "Stop moving"

### 3. Natural Language Understanding (15 points)

**Using:** GPT-4, GPT-3.5-turbo, or Claude API

Implement task planning with LLM:
- Parse natural language commands into actionable steps
- Generate task plans (sequence of robot actions)
- Handle ambiguity and clarification requests
- Provide natural language feedback

**Example Task Breakdown:**

User: "Bring me the red cup from the kitchen table"

LLM Output:
```json
{
  "task": "fetch_object",
  "steps": [
    {"action": "navigate", "target": "kitchen"},
    {"action": "search_object", "object": "red cup", "location": "table"},
    {"action": "grasp", "object": "red cup"},
    {"action": "navigate", "target": "user_location"},
    {"action": "handover", "object": "red cup"}
  ],
  "constraints": ["avoid_obstacles", "safe_grasp"],
  "response": "I will navigate to the kitchen, find the red cup on the table, and bring it to you."
}
```

### 4. Perception System (15 points)

**Computer Vision Pipeline:**

**4A. Object Detection (8 points)**
- Detect and classify objects in scene
- Bounding boxes with confidence scores
- Minimum 10 object classes supported
- Real-time inference (≥10 FPS)

**4B. Spatial Localization (7 points)**
- Estimate 3D position of detected objects
- Use depth camera or stereo vision
- Transform object coordinates to robot frame
- Publish object poses to TF tree

**Required Detections:**
- Household objects (cup, ball, bottle, book, etc.)
- Furniture (chair, table, shelf)
- Obstacles (walls, doorways)

### 5. Navigation System (15 points)

**Autonomous Navigation:**

Implement goal-based navigation:
- Accept goals from task planner
- Generate collision-free paths
- Navigate through doorways
- Dynamic obstacle avoidance
- Localization using VSLAM or LIDAR SLAM

**Test Scenarios:**
1. Navigate from living room to kitchen
2. Navigate while person walks in front
3. Navigate through narrow doorway
4. Recovery from dead-end situations

**Performance Metrics:**
- Navigation success rate: &gt;85%
- Average time to goal: &lt;2 minutes
- Collision rate: &lt;5% of runs

### 6. Manipulation and Grasping (15 points)

**Dexterous Manipulation:**

Implement pick-and-place:
- Generate grasp poses from object detection
- Plan collision-free arm trajectories (MoveIt 2)
- Execute grasp with appropriate force
- Object handover to human

**Grasp Requirements:**
- Minimum 3 different object shapes (cylinder, box, sphere)
- Success rate: &gt;70% for known objects
- No object damage or dropping
- Safe handover (gentle release near human hand)

**Humanoid Specifics:**
- Use humanoid arms (7+ DOF)
- Coordinate hand and arm motion
- Optional: Bipedal balance during manipulation

### 7. Humanoid Locomotion (Bonus: +10 points)

**Bipedal Walking:**

If using bipedal humanoid, implement:
- Stable walking gait (forward, backward, turning)
- Balance control using IMU feedback
- Stair climbing (optional, +5 additional points)

**Locomotion Metrics:**
- Walking speed: ≥0.3 m/s
- Stability: No falls during navigation
- Energy efficiency: Track joint torques

### 8. System Integration and Orchestration (10 points)

**High-Level Architecture:**

Create a state machine or behavior tree:
- Coordinate perception, planning, and action
- Handle failures gracefully (retry, replan, ask for help)
- Log all actions and state transitions
- Provide real-time status updates

**System Components:**
```
┌─────────────────────────────────────────────┐
│  Voice Interface (Whisper + TTS)            │
└──────────────┬──────────────────────────────┘
               │
┌──────────────▼──────────────────────────────┐
│  Task Planner (GPT-4 / Claude)              │
└──────────────┬──────────────────────────────┘
               │
       ┌───────┴───────┐
       │               │
┌──────▼──────┐ ┌─────▼──────┐
│ Perception  │ │ Navigation │
│ (YOLO/D2)   │ │ (Nav2)     │
└──────┬──────┘ └─────┬──────┘
       │               │
       └───────┬───────┘
               │
┌──────────────▼──────────────────────────────┐
│  Manipulation (MoveIt 2)                    │
└──────────────┬──────────────────────────────┘
               │
┌──────────────▼──────────────────────────────┐
│  Humanoid Controller (ROS 2 Control)        │
└─────────────────────────────────────────────┘
```

### 9. Performance and Reliability (5 points)

**System Metrics:**

Monitor and report:
- End-to-end task completion time
- Success rate across 10 test scenarios
- Component latencies (perception, planning, execution)
- Resource usage (CPU, GPU, memory)
- Error recovery rate

**Minimum Standards:**
- Task completion time: &lt;5 minutes
- Success rate: &gt;75%
- System uptime: &gt;90% during demo

### 10. Documentation and Presentation (10 points)

**Technical Documentation:**

**README.md** (2 points):
- System overview and architecture
- Installation and setup instructions
- How to run demos and tests
- Troubleshooting guide

**Technical Report** (5 points):
8-10 page PDF including:
- Abstract and introduction
- System architecture with diagrams
- Detailed component descriptions
- Algorithm explanations
- Performance evaluation and results
- Challenges and solutions
- Future work and improvements
- References

**Final Presentation** (3 points):
10-minute presentation to class covering:
- Project motivation and goals
- System demonstration (live or video)
- Technical highlights
- Lessons learned
- Q&A session

## Deliverables

Submit via the course portal by end of Week 13:

1. **Source Code:**
   - Complete ROS 2 workspace (Git repository preferred)
   - Isaac Sim scene files
   - Launch files and configuration
   - AI model files or download scripts
   - Requirements.txt / package.xml with dependencies

2. **Demo Video:**
   - 5-7 minute comprehensive demonstration
   - Show at least 3 complete task executions
   - Include voice commands and robot responses
   - Show system recovery from one failure scenario
   - Picture-in-picture showing robot POV and external view

3. **Technical Report:**
   - 8-10 page PDF (IEEE or ACM format)
   - Include architecture diagrams
   - Performance graphs and tables
   - Code snippets for key algorithms

4. **Presentation Slides:**
   - 10-12 slides for final presentation
   - PDF and PPTX formats

5. **Performance Log:**
   - CSV or JSON with benchmark data
   - 10+ test scenario results

6. **Optional: Physical Deployment:**
   - If you deploy to Jetson Orin Nano or physical robot:
     - Video of physical robot execution
     - Sim-to-real analysis document
     - Bonus: +20 points

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Simulation Environment** | 10 | Photorealistic, functional, appropriate complexity |
| **Speech Interface** | 10 | Real-time recognition, clear synthesis, robust |
| **NLU & Task Planning** | 15 | LLM integration, task decomposition, error handling |
| **Perception** | 15 | Accurate detection, 3D localization, real-time |
| **Navigation** | 15 | Reliable goal-reaching, obstacle avoidance |
| **Manipulation** | 15 | Successful grasps, safe handover, diverse objects |
| **System Integration** | 10 | Coordinated components, state management |
| **Performance** | 5 | Meets metrics, benchmarks documented |
| **Documentation** | 10 | Clear report, presentation, code documentation |
| **Bonus: Locomotion** | +10 | Bipedal walking, stability |
| **Bonus: Physical Deploy** | +20 | Successful sim-to-real transfer |
| **Total** | 100 (+30) | |

### Grade Breakdown

- **90-100 (A range):** Exceptional - All requirements exceeded, highly polished system, excellent documentation, innovative features, bonuses completed
- **80-89 (B range):** Proficient - All core requirements met, reliable system performance, good documentation, professional presentation
- **70-79 (C range):** Developing - Most requirements met, system works with limitations, adequate documentation
- **60-69 (D range):** Beginning - Basic functionality demonstrated, significant components incomplete
- **Below 60 (F):** Incomplete - Major requirements not met, system non-functional

## Testing Scenarios

Your system must successfully complete at least 7 of these 10 scenarios:

1. **Fetch Object:** "Bring me the red cup from the kitchen table"
2. **Describe Scene:** "What objects do you see in front of you?"
3. **Navigate:** "Go to the bedroom"
4. **Multi-Step Task:** "Find the blue ball and place it on the shelf"
5. **Clarification:** "Bring me a cup" → "Which cup? The red one or the blue one?"
6. **Obstacle Avoidance:** Navigate while person walks in path
7. **Error Recovery:** "Bring me the purple sphere" (object doesn't exist) → "I cannot find a purple sphere. Should I search another room?"
8. **Handover:** Safely hand object to human within reach
9. **Status Update:** "Where is the red cup?" → "The red cup is on the kitchen table"
10. **Emergency Stop:** "Stop immediately" → Robot halts all motion

## Technical Requirements Summary

### Hardware Requirements
- NVIDIA RTX 3070 or better (for Isaac Sim)
- 32GB+ RAM (64GB recommended)
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Optional: Jetson Orin Nano for physical deployment

### Software Stack
- **Simulation:** NVIDIA Isaac Sim 2023.1.1+
- **Middleware:** ROS 2 Humble
- **Speech:** OpenAI Whisper, gTTS/AWS Polly
- **LLM:** GPT-4 API, GPT-3.5-turbo, or Claude
- **Vision:** YOLOv8, Detectron2, or Isaac ROS DNN
- **Navigation:** Nav2, Isaac ROS VSLAM
- **Manipulation:** MoveIt 2
- **Control:** ROS 2 Control framework

### API Keys Required
- OpenAI API key (GPT-4 + Whisper)
- OR: Anthropic API key (Claude)
- Optional: AWS keys (Polly TTS, RoboMaker)

:::warning Cost Notice
LLM API usage may incur costs (~$5-20 for project duration). Budget accordingly or use free alternatives (local Whisper, GPT-3.5-turbo).
:::

## Development Timeline

### Week 11: Foundation
- Set up Isaac Sim environment
- Integrate speech interface
- Basic LLM task planning
- Simple navigation demos

### Week 12: Integration
- Implement perception pipeline
- Object detection and localization
- Manipulation grasping
- End-to-end testing (partial scenarios)

### Week 13: Polish and Testing
- Complete all 10 test scenarios
- Performance optimization
- Documentation and report writing
- Presentation preparation
- Final testing and bug fixes

## Tips for Success

1. **Start Early:** This is a large project; begin Week 11 Day 1
2. **Incremental Development:** Get one component working before integrating next
3. **Version Control:** Use Git with meaningful commits
4. **Test Continuously:** Don't wait until Week 13 to test integration
5. **Simplify First:** Get basic version working, then add complexity
6. **Backup Plans:** Have fallback strategies for components that don't work
7. **Document as You Go:** Don't leave documentation for the last day
8. **Team Communication:** If working in pairs, meet daily and divide work clearly
9. **Use Office Hours:** Instructors and TAs are available to help
10. **Plan for Failures:** Build in error handling and recovery mechanisms

## Common Pitfalls

- **Scope Creep:** Trying to add too many features instead of polishing core functionality
- **Integration Delays:** Underestimating time needed to integrate components
- **API Costs:** Excessive GPT-4 calls during testing
- **Performance Issues:** Not optimizing until it's too late
- **Documentation Neglect:** No time left for proper documentation
- **Demo Failures:** Not practicing the demo before presentation

## Resources

### Official Documentation
- [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html)
- [Nav2](https://navigation.ros.org/)
- [OpenAI API](https://platform.openai.com/docs/)

### Example Projects
- [SayCan (Google)](https://say-can.github.io/)
- [RT-2 Robotics Transformer](https://robotics-transformer2.github.io/)
- [Mobile ALOHA](https://mobile-aloha.github.io/)

### Course Materials
- All lecture slides and recordings
- Previous module projects and labs
- Setup guides and resource documents

## Submission

**Due Date:** End of Week 13, 11:59 PM (see course calendar)
**Submit to:** Course portal (LMS)
**Late Policy:** Not accepted (capstone is final project)
**Presentation:** In-class during Week 13 lab session

## Academic Integrity

- **Individual or Pair Project:** Teams of 1-2 students
- **External Code:** You may use libraries, tutorials, and example code, but **cite all sources**
- **AI Tools:** You may use ChatGPT/Claude for debugging, but **document usage**
- **Collaboration:** Inter-team help is limited to general discussions, not code sharing
- **Plagiarism:** Copying code from others or past projects is strictly prohibited

## Evaluation Criteria Beyond Rubric

Exceptional projects that may receive &gt;100% (with bonuses) demonstrate:
- **Innovation:** Novel approaches to perception, planning, or manipulation
- **Robustness:** System handles edge cases and recovers from failures gracefully
- **Polish:** Professional-quality documentation, demo, and presentation
- **Real-World Readiness:** System could be deployed with minimal modifications

## Support and Resources

**Office Hours:**
- Instructor: [Days/Times]
- TAs: [Days/Times]

**Discussion Forum:** Use course Slack/Discord for:
- Technical questions
- Team formation
- Debugging help
- Resource sharing

**Lab Access:** Extended lab hours during Week 13 for final testing

**Hardware Loan:** Jetson Orin Nano kits available for checkout (first-come, first-served)

---

## Final Words

This capstone represents the pinnacle of your learning in Physical AI and Humanoid Robotics. It's challenging, but you've been building toward this moment throughout the entire course. Every module—ROS 2, simulation, Isaac perception, VLA models—culminates here.

**Your goal is not perfection—it's integration, creativity, and demonstrating mastery of the full robotic AI stack.**

We're excited to see what you build. Good luck!

---

**Questions?** Contact the instructor immediately. Don't wait until the last week to ask for help.

**Hardware Issues?** Report hardware problems ASAP so we can provide alternatives.

**Team Issues?** Notify instructor by Week 12 if there are collaboration problems.
