# MuJoCo Accessories

A ROS 2 package providing MuJoCo simulation assets and controllers for various mobile robot platforms and manipulators. Currently supports the Clearpath Ridgeback with planned support for additional platforms.

## Overview

This package extends the MuJoCo simulation environment for various mobile robots and manipulators by providing:
- 3D mesh models for visualization
- Controller configurations for different motion types (differential drive, mecanum, etc.)
- Launch files for easy startup
- URDF/XACRO files for robot descriptions
- Example code demonstrating usage

Currently supported platforms:
- Clearpath Ridgeback (differential drive mobile base)

Coming soon:
- Additional mobile platforms
- Manipulator support
- Combined mobile manipulator configurations

## Prerequisites

- ROS 2 (Jazzy)
- MuJoCo 3.3.2
- MuJoCo ROS 2 Control

- Build and source the package.
### 1. Launch

```bash
# For other mobile platforms
ros2 launch mujoco_accessories demo_[platform_name].launch.py

# For manipulators
ros2 launch mujoco_accessories demo_[manipulator_name].launch.py

# For mobile manipulators
ros2 launch mujoco_accessories demo_[platform]_[manipulator].launch.py

```
### 2. Example Run

```bash
ros2  run  mujoco_accessories [robotname]_example 