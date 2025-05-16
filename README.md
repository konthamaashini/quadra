# Unitree Go2 ROS 2 Package

## Overview

This repository contains the ROS 2 package for the Unitree Go2 quadruped robot. It includes the robot description, 3D mesh files for visualization, control scripts, and launch files to simulate the robot in Gazebo .
---

## Package Structure
```plaintext
unitree/
├── CMakeLists.txt                    # CMake build script
├── package.xml                       # ROS 2 package manifest
├── config/                         
├── include/                          # C++ header files (if any)
├── launch/                           # Launch files for Gazebo and RViz
├── mesh/                             # 3D mesh files for URDF visualization
├── scripts/                          # Python control scripts (e.g., teleop.py, joint_control_gui.py)
├── src/                              # C++ source files (if any)
├── urdf/                             # Robot URDF and Xacro files
├── worlds/                           # Custom Gazebo world files
---
Create workspace if not already created:
mkdir -p ~/quadra/src
cd ~/quadra/src


Clone the package:
git clone <repository_url>

Build the package:
cd ~/quadra
colcon build --packages-select unitree
source install/setup.bash

To launch the simulation in Gazebo :
ros2 launch unitree go2_urdflaunch.py

To control the robot using a Python teleoperation script:
ros2 run unitree teleop.py

To open the joint control GUI:
ros2 run unitree joint_control_gui.pi
