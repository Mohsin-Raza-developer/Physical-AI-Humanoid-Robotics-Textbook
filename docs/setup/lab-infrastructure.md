---
title: Lab Infrastructure Options
sidebar_label: Lab Infrastructure
sidebar_position: 3
description: Choosing between on-premise and cloud-based lab infrastructure
---

# Lab Infrastructure Options

Building a "Physical AI" lab is a significant investment. You must choose between building a physical **On-Premise Lab** (High CapEx) versus running a **Cloud-Native Lab** (High OpEx).

This guide helps you evaluate the trade-offs and choose the right infrastructure for your teaching environment.

## Option 1: On-Premise Lab (High CapEx)

*Best for: Institutions with upfront budget, long-term programs, hands-on hardware access.*

### Infrastructure Components

| Component | Specification | Quantity | Est. Cost per Unit | Notes |
|-----------|--------------|----------|-------------------|-------|
| **Workstation** | RTX 4080 + i7 + 64GB RAM + Ubuntu 22.04 | 1 per student | $2,500 - $3,500 | Core simulation rig |
| **Edge Kit** | Jetson Orin Nano + RealSense D435i | 1 per student | $700 | Physical AI deployment |
| **Robot (Shared)** | Unitree Go2 or G1 | 1 per 4-6 students | $3,000 - $16,000 | Shared physical platform |
| **Networking** | Gigabit switches, Wi-Fi 6 access points | Lab-wide | $500 - $1,000 | Low-latency robot control |
| **Storage** | NAS for datasets and models | 1 per lab | $1,500 - $3,000 | Shared Isaac Sim assets |

### Sample Lab Configuration (20 Students)

| Item | Quantity | Unit Cost | Total Cost |
|------|----------|-----------|------------|
| Workstations | 20 | $3,000 | $60,000 |
| Jetson Edge Kits | 20 | $700 | $14,000 |
| Unitree Go2 Robots | 4 | $3,000 | $12,000 |
| Networking Equipment | 1 set | $1,000 | $1,000 |
| NAS Storage (10TB) | 1 | $2,000 | $2,000 |
| **Total CapEx** | | | **$89,000** |

### Advantages

- **Zero Latency**: Direct hardware access for real-time control
- **Predictable Costs**: One-time investment, no recurring fees
- **Hands-On Learning**: Students physically interact with robots
- **Data Privacy**: All data stays on-premise
- **Offline Capability**: Works without internet dependency

### Disadvantages

- **High Initial Investment**: $89,000+ upfront cost
- **Maintenance Burden**: IT support for hardware issues
- **Space Requirements**: Dedicated lab space needed
- **Hardware Deprecation**: Equipment becomes outdated

### When to Choose On-Premise

- You have upfront budget but limited operational budget
- You plan to run this course for 3+ years
- You want students to deploy code to physical robots
- Your institution has IT support for lab maintenance
- You prioritize data privacy and offline capabilities

## Option 2: Cloud-Native Lab (High OpEx)

*Best for: Rapid deployment, remote learning, students with weak laptops.*

### Cloud Architecture

```
┌─────────────────────────────────────┐
│  Student Laptop (Any OS)            │
│  - SSH/Web Browser                  │
│  - VS Code Remote                   │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  AWS/Azure Cloud Instance           │
│  - g5.2xlarge (A10G GPU, 24GB)      │
│  - Ubuntu 22.04                     │
│  - Isaac Sim + ROS 2 + Gazebo       │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  Local Edge Kit (Still Required)    │
│  - Jetson Orin Nano                 │
│  - RealSense D435i                  │
└─────────────────────────────────────┘
```

### Cloud Workstation Specifications

**Instance Type:** AWS **g5.2xlarge** or Azure **NC6s_v3**

- **GPU:** NVIDIA A10G (24GB VRAM)
- **vCPUs:** 8
- **RAM:** 32GB
- **Storage:** 200GB EBS volume
- **Software:** NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI)

### Cost Calculation (Per Student)

| Item | Usage | Rate | Cost |
|------|-------|------|------|
| Cloud Instance (g5.2xlarge) | 10 hours/week × 12 weeks | $1.50/hour | $180 |
| Storage (EBS 200GB) | 3 months | $20/month | $60 |
| Data Transfer | ~100GB outbound | $0.09/GB | $9 |
| Jetson Edge Kit (Still needed) | One-time | $700 | $700 |
| **Total per student** | | | **$949** |

### Sample Lab Configuration (20 Students)

| Item | Quantity | Unit Cost | Total Cost |
|------|----------|-----------|------------|
| Cloud Instances (12 weeks) | 20 | $249 | $4,980 |
| Jetson Edge Kits | 20 | $700 | $14,000 |
| Unitree Go2 (Shared) | 2 | $3,000 | $6,000 |
| **Total for 12-week course** | | | **$24,980** |

### Advantages

- **Low Initial Investment**: No hardware purchase needed
- **Scalability**: Easy to add/remove students
- **Remote Learning**: Students work from anywhere
- **Latest Hardware**: Always access to newest GPU instances
- **No Maintenance**: Cloud provider handles infrastructure

### Disadvantages

- **High Recurring Costs**: $250/student/semester
- **Latency Issues**: Cannot control physical robots from cloud in real-time
- **Internet Dependency**: Requires stable, high-bandwidth connection
- **Data Transfer Costs**: Large simulation files expensive to download
- **Limited Customization**: Restricted to cloud provider's offerings

### The Latency Trap (Hidden Cost)

:::warning Critical Issue
Simulating in the cloud works well, but *controlling* a real robot from a cloud instance is dangerous due to latency (50-200ms delays).

**Solution:** Students train in the cloud, download the model weights, and flash them to the local Jetson kit for physical deployment.
:::

### When to Choose Cloud-Native

- You need rapid deployment (start in days, not months)
- You have operational budget but limited capital budget
- Students are remote or have weak personal laptops
- Course is experimental or pilot phase
- You want to avoid hardware maintenance overhead

## Option 3: Hybrid Approach (Recommended)

*Best balance: Cloud for simulation, on-premise for physical deployment.*

### Architecture

```
┌─────────────────────────────────────┐
│  Cloud: Training & Simulation       │
│  - AWS g5.2xlarge instances         │
│  - Isaac Sim, Gazebo, Unity         │
│  - Model training (RL, VLA)         │
└──────────────┬──────────────────────┘
               │ Download Models
               ▼
┌─────────────────────────────────────┐
│  On-Premise: Physical Deployment    │
│  - Jetson Edge Kits (20 units)      │
│  - RealSense Cameras                │
│  - Shared Robots (4 Unitree Go2)    │
└─────────────────────────────────────┘
```

### Cost Breakdown (20 Students)

| Item | Cost | Notes |
|------|------|-------|
| Cloud Instances (12 weeks) | $4,980 | Simulation and training |
| Jetson Edge Kits | $14,000 | Physical AI deployment |
| Shared Robots (4 units) | $12,000 | Hands-on robot interaction |
| **Total Hybrid Cost** | **$30,980** | Best of both worlds |

### Advantages

- Eliminates need for expensive workstations ($60k saved)
- Students train in cloud, deploy to physical edge devices
- Scalable simulation, hands-on robot experience
- Lower total cost than pure on-premise

### Disadvantages

- Still requires physical lab space for robots
- Complexity of managing two environments
- Students need training on both cloud and edge workflows

## Comparison Matrix

| Factor | On-Premise | Cloud-Native | Hybrid |
|--------|------------|--------------|--------|
| **Upfront Cost** | $89,000 | $5,000 | $26,000 |
| **Per-Semester Cost** | $0 | $25,000 | $5,000 |
| **Latency** | Zero | High (50-200ms) | Low (edge local) |
| **Scalability** | Fixed (20 seats) | Unlimited | High |
| **Maintenance** | High | Zero | Low |
| **Physical Robots** | Yes | Limited | Yes |
| **Remote Learning** | No | Yes | Partial |
| **3-Year TCO** | $89,000 | $150,000 | $56,000 |

## Recommendation by Use Case

### Choose On-Premise If:
- You have $90k+ upfront budget
- Course will run 3+ years
- Physical robot interaction is mandatory
- Data privacy is critical

### Choose Cloud-Native If:
- You need rapid deployment (< 1 month)
- Students are remote/distributed
- You have operational budget ($25k/semester)
- Course is experimental/pilot phase

### Choose Hybrid If:
- You want best of both worlds
- Budget is $30-60k total
- You want scalable simulation + physical deployment
- Long-term course with gradual hardware investment

## Implementation Roadmap

### Phase 1: Pilot (Semester 1)
- Use cloud instances for all students
- Purchase 4-6 Jetson Edge Kits for demos
- Share 1-2 robots among the class

### Phase 2: Scale (Semester 2-3)
- Purchase Jetson kits for all students
- Add 2-4 more shared robots
- Continue using cloud for simulation

### Phase 3: Maturity (Year 2+)
- Evaluate workstation purchase if cost-effective
- Transition high-usage students to on-premise
- Keep cloud for overflow and remote students

## Next Steps

- Review [Hardware Requirements](./hardware-requirements) for detailed specs
- Check [Software Setup](./software-setup) for installation guides
- See [Student Kit Guide](./student-kit-guide) for Jetson configuration
