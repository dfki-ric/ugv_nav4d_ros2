# ugv_nav4d_ROS2 Wrapper

This repository provides a ROS2 wrapper for the [ugv_nav4d](https://github.com/dfki-ric/ugv_nav4d) library, which is a path planning library designed for autonomous vehicle navigation. The `ugv_nav4d_ros2` package enables easy integration of ugv_nav4d into ROS2 environments and provides visualizations for **MLS** (Multilayered Surface Maps) and **TravMap3D** (Traversability Map 3D).

## Features

- ROS2 integration of the ugv_nav4d library.
- Visualization tools for MLS and TravMap3D in RViz.

## Installation

### Dependencies

1. **ROS2**: Ensure you have `ROS2 Humble` installed on your system.
2. **ugv_nav4d**: You will need the original `ugv_nav4d` library. Install it using the instructions [here](https://github.com/dfki-ric/ugv_nav4d.git). Source the `env.sh` after installation to make ugv_nav4d library visible for ROS2. See details in documentation of ugv_nav4d.
3. **Build**: Build the ROS2 workspace
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
4. **Launch**: 
   ```
   ros2 launch ugv_nav4d_ros2 ugv_nav4d.launch.py
   ```
   Hint: Adjust the arguments pointcloud_topic and goal_topic. The pointcloud_topic expects a pointcloud map generated and corrected based on a SLAM algorithm. Alternatively, one could also use a PLY file to generate a MLS. See example files [here](https://zenodo.org/records/13771864). Most of the parameters in the [params.yaml](config/params.yaml) are explained in the original documentation of ugv_nav4d. 

### Services / Actions

Services:

Publish the MLS Map
```
ros2 service call /ugv_nav4d_ros2/map_publish std_srvs/srv/Trigger
```

Actions:

Save MLS Map as a file
```
ros2 action send_goal /ugv_nav4d_ros2/save_mls_map ugv_nav4d_ros2/action/SaveMLSMap filename:\ \'\'\
```

### Parameters

| Parameter              | Type    | Default Value       | Description                                                                 |
|------------------------|---------|---------------------|-----------------------------------------------------------------------------|
| `read_pose_from_topic`  | Boolean | `true`              | If `false`, the pose is read using TF2 with `robot_frame` and `world_frame`. |
| `load_mls_from_file`   | Boolean | `false`             | If `false`, then a slam corrected pointcloud is expected on a topic. |
| `mls_file_type`          | String  | `ply` | Options: `bin` or `ply`. |
| `mls_file_path`          | String  | `default_value` | Full path to the MLS file. |
| `mls_gap_size`          | Double  | `0.1` | The gap size used to decide on whether stacked MLS patches can be merged into one patch |
| `robot_frame`          | String  | `robot` | Frame name of the robot in TF2. Use when `read_pose_from_topic` is `false` |
| `world_frame`          | String  | `world` | Frame name of the world in TF2. Use when `read_pose_from_topic` is `false` |
| `grid_resolution`          | Double  | `0.3` | Grid resolution of the MLS and Traversability3d Map  |
| `dist_max_x`          | Double  | `50` | Maximum length of MLS along x-axis (in meters)  |
| `dist_max_y`          | Double  | `50` | Maximum length of MLS along y-axis (in meters) |
| `dist_max_z`          | Double  | `50` | Maximum length of MLS along z-axis (in meters) |
| `dist_min_x`          | Double  | `-50` | Mininum length of MLS along x-axis (in meters) |
| `dist_min_y`          | Double  | `-50` | Mininum length of MLS along y-axis (in meters) |
| `dist_min_z`          | Double  | `-50` | Mininum length of MLS along z-axis (in meters) |

## Bug Reports

To search for bugs or report them, please use GitHubs [Issue-Tracker](https://github.com/dfki-ric/ugv_nav4d_ros2/issues

