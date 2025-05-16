# unitree
# Unitree Go2 ROS 2 Package

## Overview

This repository contains the ROS 2 package for the Unitree Go2 quadruped robot. It includes the robot description, 3D mesh files for visualization, and launch files to simulate the robot in Gazebo and visualize it in RViz.

## Package Structure

```plaintext
unitree/
├── CMakeLists.txt                      # CMake build script
├── package.xml                         # ROS 2 package manifest
├── launch/
│   └── go2_urdflaunch.py               # Launch file for Gazebo and RViz
├── meshes/
│   └── [robot_mesh_files.stl/dae]     # 3D meshes for visualization
├── urdf/
│   └── go2_urdf.urdf                   # Main URDF describing the robot
└── rviz/
    └── go2_config.rviz                 # Optional RViz configuration file
# Create workspace if not already created
mkdir -p ~/quadra/src
cd ~/quadra/src

# Clone the package
git clone <repository_url>

# Build the package
cd ~/quadra
colcon build --packages-select unitree
source install/setup.bash


ros2 launch unitree go2_urdflaunch.py

ros2 run unitree teleop.py

ros2 run unitree joint_control_gui.pi
