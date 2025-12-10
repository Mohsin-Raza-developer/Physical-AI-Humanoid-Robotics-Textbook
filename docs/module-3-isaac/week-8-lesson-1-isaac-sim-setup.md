---
title: Isaac Sim Fundamentals
sidebar_label: "Lesson 1: Isaac Sim Setup"
sidebar_position: 81
description: Introduction to NVIDIA Isaac Sim
tags: [isaac, week-8]
---

# Isaac Sim Fundamentals

Isaac Sim is NVIDIA's simulation application and framework for developing and testing AI-based robotics applications in accelerated simulation.

## Key Features

:::caution Hardware Requirements
Isaac Sim requires significant GPU resources:
- **GPU**: NVIDIA RTX GPU (RTX 2060 or higher) with at least 8GB VRAM
- **Driver**: NVIDIA Driver 525.60.11 or later
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11
- **Not supported on macOS** due to CUDA dependency
:::

- Advanced physics simulation with PhysX
- Flexible scene generation and composition
- Comprehensive sensor simulation
- Ground truth data generation
- Integration with Omniverse

:::note Cloud Alternative
Don't have a powerful GPU? Consider using NVIDIA Omniverse Cloud or cloud computing platforms like AWS EC2 G4/G5 instances with pre-configured Isaac Sim containers.
:::