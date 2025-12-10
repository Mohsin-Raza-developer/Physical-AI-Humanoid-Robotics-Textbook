---
title: Economy Jetson Student Kit
sidebar_label: Student Kit Guide
sidebar_position: 4
description: Setting up the Economy Jetson Student Kit for Physical AI development
---

# Economy Jetson Student Kit Guide

*Best for: Learning ROS 2, Basic Computer Vision, and Sim-to-Real control.*

This guide walks you through purchasing, assembling, and configuring the Economy Jetson Student Kit - a complete edge AI platform for deploying your trained models to physical hardware.

## Kit Components

| Component | Model | Price (Approx.) | Notes |
|-----------|-------|-----------------|-------|
| **The Brain** | **NVIDIA Jetson Orin Nano Super Dev Kit (8GB)** | **$249** | New official MSRP (Price dropped from ~$499). Capable of 40 TOPS. |
| **The Eyes** | **Intel RealSense D435i** | **$349** | Includes IMU (essential for SLAM). Do not buy the D435 (non-i). |
| **The Ears** | **ReSpeaker USB Mic Array v2.0** | **$69** | Far-field microphone for voice commands (Module 4). |
| **Wi-Fi** | (Included in Dev Kit) | $0 | The new "Super" kit includes the Wi-Fi module pre-installed. |
| **Power/Misc** | SD Card (128GB) + Jumper Wires | $30 | High-endurance microSD card required for the OS. |
| **TOTAL** | | **~$700 per kit** | |

## What This Kit Can Do

- **Run ROS 2 Humble natively** on ARM64 architecture
- **Execute Isaac ROS packages** for hardware-accelerated perception
- **Deploy trained AI models** from Isaac Sim to physical edge device
- **Process real-time sensor data** (RGB-D camera, IMU, microphone)
- **Control physical robots** with low-latency communication
- **Learn resource-constrained AI** by comparing workstation vs. edge performance

## Purchase Links

### Official Sources

- **Jetson Orin Nano Super Dev Kit**: [NVIDIA Store](https://store.nvidia.com/) or [Seeed Studio](https://www.seeedstudio.com/)
- **Intel RealSense D435i**: [Intel Store](https://www.intelrealsense.com/depth-camera-d435i/) or Amazon
- **ReSpeaker Mic Array**: [Seeed Studio](https://www.seeedstudio.com/ReSpeaker-Mic-Array-v2-0.html)
- **128GB High-Endurance microSD Card**: Samsung PRO Endurance or SanDisk High Endurance

:::warning Important
Ensure you purchase the **Jetson Orin Nano Super Dev Kit**, not the older Orin Nano. The "Super" version has better performance and comes with Wi-Fi pre-installed.
:::

## Assembly Instructions

### 1. Prepare the Jetson Orin Nano

#### Flash JetPack OS

1. **Download JetPack SDK**
   - Visit [NVIDIA JetPack Downloads](https://developer.nvidia.com/embedded/jetpack)
   - Download JetPack 6.0 or newer (includes Ubuntu 22.04 + ROS 2 support)

2. **Flash to microSD Card**
   ```bash
   # On your Ubuntu workstation
   sudo apt install -y nvidia-jetpack-sdk-balena-etcher

   # Or use Balena Etcher (GUI)
   # Download from: https://www.balena.io/etcher/
   ```

3. **Insert microSD into Jetson**
   - Insert the flashed microSD card into the Jetson Dev Kit
   - Connect monitor, keyboard, mouse via USB
   - Connect power supply (USB-C, 15W+)

4. **Initial Setup**
   - Boot the Jetson (first boot takes 5-10 minutes)
   - Complete Ubuntu setup wizard
   - Create user account: `student` (or your preferred name)
   - Connect to Wi-Fi network

### 2. Connect the RealSense Camera

1. **Physical Connection**
   - Connect Intel RealSense D435i to Jetson via USB 3.0 port
   - Blue indicator light should turn on

2. **Install RealSense SDK**
   ```bash
   # Add Intel repository
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

   sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

   # Install librealsense
   sudo apt update
   sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
   ```

3. **Test Camera**
   ```bash
   realsense-viewer
   ```
   You should see RGB, Depth, and Infrared streams.

### 3. Connect the ReSpeaker Microphone

1. **Physical Connection**
   - Connect ReSpeaker USB Mic Array to Jetson via USB 2.0 port
   - LEDs should light up in circular pattern

2. **Install ReSpeaker Drivers**
   ```bash
   sudo apt install -y python3-pyaudio portaudio19-dev
   pip3 install respeaker
   ```

3. **Test Microphone**
   ```bash
   # Record test audio
   arecord -D plughw:2,0 -f S16_LE -r 16000 -c 1 test.wav

   # Play back
   aplay test.wav
   ```

## Software Configuration

### 1. Install ROS 2 Humble on Jetson

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Isaac ROS Packages

```bash
# Install prerequisites
sudo apt install -y python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Create workspace
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

# Clone Isaac ROS repositories
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# Install dependencies
cd ~/workspaces/isaac_ros-dev
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
echo "source ~/workspaces/isaac_ros-dev/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install RealSense ROS 2 Wrapper

```bash
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development

cd ~/workspaces/isaac_ros-dev
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 4. Configure Audio for Whisper (Module 4)

```bash
# Install audio libraries
sudo apt install -y portaudio19-dev python3-pyaudio

# Install Whisper dependencies
pip3 install openai-whisper torch torchaudio
```

## Testing Your Kit

### Test 1: RealSense Camera with ROS 2

```bash
# Terminal 1: Launch RealSense node
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: View RGB image
ros2 run rqt_image_view rqt_image_view

# Terminal 3: Echo depth data
ros2 topic echo /camera/depth/image_rect_raw
```

### Test 2: Isaac ROS Visual SLAM

```bash
# Launch VSLAM with RealSense
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Test 3: Voice Input with ReSpeaker

```bash
# Test microphone input
python3 << EOF
import pyaudio
import wave

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1024
RECORD_SECONDS = 5

audio = pyaudio.PyAudio()
stream = audio.open(format=FORMAT, channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)

print("Recording...")
frames = []
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)
print("Done recording.")

stream.stop_stream()
stream.close()
audio.terminate()
EOF
```

## Connecting to Your Workstation

### Network Setup

1. **Connect Jetson and Workstation to same network**
   - Both on same Wi-Fi or Ethernet LAN
   - Find Jetson IP: `hostname -I`

2. **Enable SSH on Jetson**
   ```bash
   sudo apt install -y openssh-server
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

3. **SSH from Workstation**
   ```bash
   ssh student@<jetson-ip-address>
   ```

4. **Set up ROS_DOMAIN_ID** (to avoid cross-talk)
   ```bash
   # On Jetson
   echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

   # On Workstation
   echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc
   ```

### Workflow: Train on Workstation, Deploy to Jetson

1. **Train model on workstation** (Isaac Sim, Gazebo, Unity)
2. **Export model weights** (PyTorch .pth, ONNX, TensorRT)
3. **Transfer to Jetson via SCP**
   ```bash
   scp model_weights.pth student@<jetson-ip>:~/models/
   ```
4. **Run inference on Jetson** with RealSense camera
5. **Monitor performance** (latency, FPS, power consumption)

## Performance Expectations

### Jetson Orin Nano Super (8GB)

- **AI Performance**: 40 TOPS (INT8)
- **Image Processing**: 30 FPS for 1080p object detection (YOLOv5)
- **VSLAM**: 15-20 FPS with RealSense D435i
- **Power Consumption**: 7-15W (much lower than workstation)
- **Latency**: Sub-50ms for perception pipeline

### Comparison: Workstation vs. Jetson

| Metric | RTX 4080 Workstation | Jetson Orin Nano |
|--------|---------------------|------------------|
| **TOPS** | 300+ | 40 |
| **Power** | 320W | 15W |
| **FPS (YOLOv5)** | 120 FPS | 30 FPS |
| **Memory** | 16GB VRAM | 8GB Unified |
| **Cost** | $3,000 | $249 |

:::tip Learning Insight
The Jetson teaches you **resource-constrained AI** - how to optimize models for edge deployment. This is critical for real-world robotics where power, weight, and cost matter.
:::

## Troubleshooting

### Issue: RealSense not detected

```bash
# Check USB connection
lsusb | grep Intel

# If not found, try different USB port
# Ensure using USB 3.0 port (blue color)
```

### Issue: Isaac ROS build fails

```bash
# Increase swap space (Jetson has limited RAM)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Retry build
colcon build --symlink-install --parallel-workers 1
```

### Issue: ROS 2 nodes can't communicate between Jetson and Workstation

```bash
# Ensure ROS_DOMAIN_ID matches on both devices
echo $ROS_DOMAIN_ID  # Should be same number

# Check firewall rules
sudo ufw allow from <workstation-ip>
```

## Next Steps

- Complete [Software Setup](./software-setup) on your workstation
- Review [Module 3: NVIDIA Isaac](../module-3-isaac/intro) for Isaac ROS tutorials
- Start [Module 4: VLA & Capstone](../module-4-vla/intro) for deployment workflows

## Additional Resources

- [NVIDIA Jetson Developer Guide](https://developer.nvidia.com/embedded/develop/getting-started)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
