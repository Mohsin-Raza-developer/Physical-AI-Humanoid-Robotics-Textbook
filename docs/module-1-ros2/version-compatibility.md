---
title: ROS 2 Version Compatibility
sidebar_label: Version Compatibility
sidebar_position: 99
description: ROS 2 version compatibility and migration notes for Module 1
---

# ROS 2 Version Compatibility Guide

## Target Version

**Module 1 content is developed and tested with ROS 2 Humble Hawksbill (LTS)**

- Release Date: May 2022
- Support Until: May 2027
- Ubuntu Version: 22.04 (Jammy Jellyfish)

## Compatibility with Other ROS 2 Versions

### ✅ Compatible Versions

The Module 1 lessons use **core ROS 2 features** that are available across all major ROS 2 distributions. The content should work with:

- **ROS 2 Humble** (LTS) - Recommended ✅
- **ROS 2 Iron** (Non-LTS, May 2023 - Nov 2024)
- **ROS 2 Jazzy** (LTS, May 2024 - May 2029)
- **ROS 2 Rolling** (Latest development)

### Key Features Used (Version-Agnostic)

All lessons use standard ROS 2 APIs that have been stable since Foxy:

- **Python Client Library (rclpy)**: Standard node creation, publishers, subscribers
- **Topics**: Publish-subscribe communication pattern
- **Services**: Synchronous request-response
- **Actions**: Asynchronous goal-feedback-result pattern
- **Launch Files**: Python launch system
- **Parameters**: Dynamic configuration
- **URDF**: Robot description format
- **Packages**: colcon build system

### No Version-Specific Features

Module 1 **does not use** any Humble-specific features, experimental APIs, or deprecated functionality. All code examples follow ROS 2 best practices that work across distributions.

## Migration from Older Versions

### From ROS 2 Foxy (LTS, EOL June 2023)

**Changes Needed**: Minimal to none

- All Module 1 code examples should work without modification
- Package dependencies remain the same
- Launch file syntax is compatible

**Action Required**: Update your system to a supported LTS version (Humble or Jazzy)

### From ROS 2 Galactic (EOL December 2022)

**Changes Needed**: None for Module 1 content

- Core APIs are identical
- Code examples compatible

**Action Required**: Upgrade to Humble or newer

## Using with Newer Versions (Iron, Jazzy, Rolling)

### Expected Behavior

Module 1 code examples **should run without modification** on:

- ROS 2 Iron (Non-LTS)
- ROS 2 Jazzy (LTS)
- ROS 2 Rolling

### Potential Differences

While the core functionality is identical, newer versions may have:

1. **Performance improvements** - Faster message passing, reduced latency
2. **Additional features** - New parameters, enhanced debugging tools
3. **Deprecated warnings** - Older APIs may show deprecation notices (but still work)

### If You Encounter Issues

If you experience problems with Module 1 content on a newer ROS 2 version:

1. Check the [ROS 2 release notes](https://docs.ros.org/en/rolling/Releases.html) for breaking changes
2. Verify your package dependencies in `package.xml`
3. Ensure you've sourced the correct ROS 2 installation:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
4. Report issues in the course repository

## Platform Compatibility

### Supported Platforms

- **Ubuntu 22.04 (Jammy)** - Recommended for Humble ✅
- **Ubuntu 24.04 (Noble)** - Use with Jazzy
- **Windows 10/11** - Via WSL2 (Ubuntu 22.04)
- **macOS** - Via Docker container

### Notes for WSL2 (Windows)

All Module 1 lessons work in WSL2 with Ubuntu 22.04. GUI tools (RViz2) require:

- WSLg (Windows 11 with WSL2)
- X11 forwarding (Windows 10)

### Notes for Docker

When using Docker:
- Use official ROS 2 Docker images: `ros:humble` or `ros:jazzy`
- Mount your workspace as a volume
- Enable GUI forwarding for RViz2

## Verification

To verify your ROS 2 installation is compatible:

```bash
# Check ROS 2 version
echo $ROS_DISTRO

# Verify core packages
ros2 pkg list | grep demo_nodes
ros2 pkg list | grep turtlesim

# Test basic functionality
ros2 run demo_nodes_py talker
```

Expected output:
```
humble  # or iron, jazzy, rolling
```

## Summary

**Bottom Line**: Module 1 uses stable, core ROS 2 features. While developed for Humble, the content is compatible with other ROS 2 distributions without modification.

**Recommendation**: Use ROS 2 Humble (LTS) or Jazzy (LTS) for long-term support and stability.

## Additional Resources

- [ROS 2 Distributions](https://docs.ros.org/en/rolling/Releases.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Migration Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS 2 Platform Support](https://www.ros.org/reps/rep-2000.html)
