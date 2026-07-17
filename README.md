# ugv_nav4d_ROS2 Wrapper

This repository provides a ROS2 wrapper for the [ugv_nav4d](https://github.com/dfki-ric/ugv_nav4d) library, which is a path planning library designed for autonomous vehicle navigation. The `ugv_nav4d_ros2` package enables easy integration of ugv_nav4d into ROS2 environments and provides visualizations for **MLS** (Multilayered Surface Maps) and **TravMap3D** (Traversability Map 3D).

## Features

- ROS2 integration of the ugv_nav4d library.
- Visualization tools for MLS and TravMap3D in RViz.
- Field-operator mission panel with preview/execute gating, pause, resume,
  abort, replan-from-current-pose, return planning, and mission persistence.
- Structured planner/execution status, route-risk summaries, and aggregated
  localization/map/controller/battery/communications health.
- True-3D goals and waypoints, click-to-explain traversability inspection, and
  operational zones (keep-out, caution, speed, preferred, direction, notes).
- Multi-storey speed zones use traversability-surface connectivity within the
  polygon, so overlapping floors are independent without a fixed Z tolerance.
  Invalid or ambiguous surface matches fall back to the lowest overlapping XY
  limit, and a short release delay prevents localization chatter at zone edges.
- Route revalidation when maps or operational zones change. Invalidated routes
  pause instead of continuing on stale planning data.

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

Field execution controls:
```
ros2 service call /ugv_nav4d_ros2/pause_execution std_srvs/srv/Trigger
ros2 service call /ugv_nav4d_ros2/resume_execution std_srvs/srv/Trigger
ros2 service call /ugv_nav4d_ros2/stop_execution std_srvs/srv/Trigger
ros2 service call /ugv_nav4d_ros2/replan_current_mission std_srvs/srv/Trigger
```

Mission persistence uses the `mission_file` parameter:
```
ros2 service call /ugv_nav4d_ros2/save_mission std_srvs/srv/Trigger
ros2 service call /ugv_nav4d_ros2/load_mission std_srvs/srv/Trigger
```

Operator-facing topics:

- `/ugv_nav4d_ros2/planner_status` and `/ugv_nav4d_ros2/execution_status`
- `/ugv_nav4d_ros2/route_risk`
- `/ugv_nav4d_ros2/system_health`
- `/ugv_nav4d_ros2/operational_zones`
- `/ugv_nav4d_ros2/route_valid`

The RViz **Inspect Traversability** tool reports cell classification, slope,
direction, cost, and allowed heading bands. The operational-zone tool exposes
zone type and associated values in its RViz property panel.

Safety policies are conservative by default (`warn`). Set
`low_battery_policy` or `lost_communications_policy` to `pause` or `return` as
required. `return` only prepares a return preview; execution still requires
operator approval. A communications watchdog is activated only when
`heartbeat_required` is true and expects `std_msgs/msg/Empty` on
`/operator_heartbeat`.

`monitor_map_freshness` defaults to `false`, so a pre-generated or slowly
updated MLS remains healthy after its first publication. Enable it only when a
deployment requires periodic traversability-map updates; `map_timeout` then
defines the maximum permitted interval.

Publish the MLS Map
```
ros2 service call /ugv_nav4d_ros2/map_publish std_srvs/srv/Trigger
```

Actions:

Save MLS Map as a file
```
ros2 action send_goal /ugv_nav4d_ros2/save_mls_map ugv_nav4d_ros2/action/SaveMLSMap filename:\ \'\'\
```

## Bug Reports

To search for bugs or report them, please use GitHubs [Issue-Tracker](https://github.com/dfki-ric/ugv_nav4d_ros2/issues
