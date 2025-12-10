---
title: Software Setup
sidebar_label: Software Setup
sidebar_position: 2
description: Step-by-step software installation guide for Ubuntu, ROS 2, and Isaac Sim
---

# Software Setup Guide

This guide walks you through installing all required software for the Physical AI Humanoid Robotics course. Follow these steps in order for a smooth setup experience.

## Prerequisites

- Ubuntu 22.04 LTS installed (native or dual-boot recommended)
- At least 100GB free disk space
- Stable internet connection
- NVIDIA GPU with latest drivers installed

## 1. System Preparation

### Update System Packages

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget python3-pip
```

### Install NVIDIA Drivers

For RTX 30-series and newer:

```bash
# Add NVIDIA PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (typically 535 or newer)
sudo ubuntu-drivers autoinstall

# Reboot to apply changes
sudo reboot
```

Verify installation:

```bash
nvidia-smi
```

You should see your GPU listed with driver version and CUDA version.

## 2. Install ROS 2 Humble

### Set Locale

```bash
locale  # Check current settings
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 Repository

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Packages

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
sudo apt install -y python3-colcon-common-extensions
```

### Environment Setup

Add to your `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

```bash
ros2 --help
```

## 3. Install Gazebo Classic

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y gazebo
```

Verify installation:

```bash
gazebo --version
```

## 4. Install NVIDIA Isaac Sim

### Install Omniverse Launcher

1. Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Extract and run the AppImage:

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

### Install Isaac Sim via Omniverse

1. Open Omniverse Launcher
2. Go to "Exchange" tab
3. Search for "Isaac Sim"
4. Click "Install" (version 2023.1.1 or newer)
5. Wait for installation to complete (~20GB download)

### Verify Isaac Sim Installation

Launch Isaac Sim from Omniverse Launcher and check that it opens without errors.

## 5. Install Isaac ROS

### Install Docker (Required for Isaac ROS)

```bash
# Remove old versions
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install prerequisites
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Install NVIDIA Container Toolkit

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Clone Isaac ROS Repositories

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

## 6. Install Unity (Optional)

### Install Unity Hub

```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

### Install Unity Editor

1. Open Unity Hub
2. Go to "Installs" → "Install Editor"
3. Choose Unity 2022.3 LTS or newer
4. Add "Linux Build Support" module

### Install Unity Robotics Hub

In Unity, install via Package Manager:
- Window → Package Manager
- Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

## 7. Additional Development Tools

### Install Visual Studio Code

```bash
sudo snap install --classic code
```

### Install Useful ROS 2 Tools

```bash
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro
```

### Install Python Packages

```bash
pip3 install --upgrade pip
pip3 install numpy opencv-python matplotlib torch torchvision
```

## 8. Verify Complete Setup

### Test ROS 2

```bash
# Terminal 1: Start talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener
ros2 run demo_nodes_py listener
```

### Test Gazebo with ROS 2

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### Test Isaac Sim

Launch Isaac Sim from Omniverse Launcher and load a sample robot scene.

## Troubleshooting

### Common Issues

**Issue: ROS 2 commands not found**
- Solution: Ensure you've sourced the setup file: `source /opt/ros/humble/setup.bash`

**Issue: Gazebo crashes on startup**
- Solution: Check NVIDIA drivers with `nvidia-smi` and ensure OpenGL is working

**Issue: Isaac Sim won't launch**
- Solution: Verify GPU has at least 8GB VRAM and drivers are up to date

**Issue: Docker permission denied**
- Solution: Run `sudo usermod -aG docker $USER` and log out/in again

## Next Steps

- Review [Hardware Requirements](./hardware-requirements) to ensure your system meets specifications
- Explore [Lab Infrastructure](./lab-infrastructure) for deployment options
- Check the [Student Kit Guide](./student-kit-guide) for Jetson setup

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Gazebo Tutorials](https://gazebosim.org/docs)
