# ugv_nav4d_ROS2 Wrapper

This repository provides a ROS 2 wrapper for the [ugv_nav4d](https://github.com/dfki-ric/ugv_nav4d) library, which is a path planning library designed for autonomous vehicle navigation. The `ugv_nav4d_ros2` package enables easy integration of ugv_nav4d into ROS 2 environments, providing useful launch files and visualizations for **MLS** (Multilayered Surface Maps) and **TravMap3D** (Traversability Map 3D).

## Features

- ROS 2 integration of the ugv_nav4d library.
- Launch files for simplified setup and operation.
- Visualization tools for MLS and TravMap3D in RViz.

## Installation

### Dependencies

1. **ROS 2**: Ensure you have ROS 2 Humble installed on your system.
2. **ugv_nav4d**: You will need the original ugv_nav4d library. Install it using the instructions [here](https://github.com/dfki-ric/ugv_nav4d.git). Source the `env.sh` after installation to make ugv_nav4d library visible for ROS2. See details in documentation of ugv_nav4d.
3. **Build**: Build the ROS2 workspace
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
4. **Launch**: 
   ```
   ros2 launch ugv_nav4d_ros2 ugv_nav4d.launch.py
   ```
   Hint: Adjust the arguments pointcloud_topic and goal_topic. The pointcloud_topic expects a pointcloud map generated and corrected based on a SLAM algorithm.