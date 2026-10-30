# qbot_platform_ros2

The ROS 2 repository for QBot Platform: hardware drivers, localization, and RL policy deployment, for the physical robot and the virtual one in Quanser Interactive Labs.

See [CHANGELOG.md](./CHANGELOG.md) for what changed in each release.

## Overview

This repo provides the low-level ROS 2 interface to the QBot Platform: motor command, sensor interface, and localization (scan-match). It runs on the QBot's onboard Jetson and publishes the sensor and pose topics. On top of that, `qbot_rl_deploy` runs a trained RL navigation policy that uses those topics to drive the robot to a goal.

## Packages in this repo

| Package | Description |
|---|---|
| [`qbot_hardware`](./qbot_hardware/README.md) | Motor control and sensor drivers (wheel encoders, gyro, lidar, etc.) |
| [`qbot_localization`](./qbot_localization/README.md) | Scan-match localization and EKF sensor fusion |
| [`qbot_rl_deploy`](./qbot_rl_deploy/README.md) | Observation builder, policy action server and client, and the navigate launch file |
| [`qbot_rl_interfaces`](./qbot_rl_interfaces/README.md) | Custom observation and action definitions for RL navigation |

## Prerequisites

- ROS 2 Humble, which the physical QBot Platform runs. 
- The ROS package dependencies declared in each `package.xml` can be installed with `rosdep install --from-paths src --ignore-src -y` from the workspace root.
- [Quanser SDK](https://github.com/quanser/quanser_sdk).
- QUARC runtime, with `quarc_run` on `PATH`. Required to run application-specific RT models.
- PyTorch, required by `qbot_rl_deploy`, already satisfied on hardware QBot Platform.
- (Optional) Quanser Interactive Labs and its Python API (`qvl`), for the virtual QBot Platform.

## Getting Started

Prior to running any ROS 2 example, source Humble in your terminal session:

```bash
source /opt/ros/humble/setup.bash
```

### Build

Clone this repo into the `src` folder of a ROS 2 workspace, then build from the workspace root:

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/quanser/qbot_platform_ros2.git ~/ros2_ws/src/qbot_platform_ros2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select qbot_hardware qbot_localization qbot_rl_interfaces qbot_rl_deploy
source install/setup.bash
```

### Running nodes

```bash
ros2 run <package_name> <node_name>
```

Example:
```bash
ros2 run qbot_hardware qbot_platform_driver_interface
```

Nodes should generally run one per terminal session for development/debugging. For normal operation, use a launch file.

### Running launch files

```bash
ros2 launch <package_name> <launch_file_name>.py
```

Example:
```bash
ros2 launch qbot_hardware qbot_platform_launch.py
```

See each package's README for the full list of nodes, topics, parameters, and launch files.

## Guides

- [Running RL mapless navigation](./docs/rl_mapless_navigation.md): run a trained RL navigation policy on the physical QBot Platform or in QLabs, from prerequisites to sending a goal.

## Known Limitations / Operational Notes

- Running RViz locally on the QBot's onboard Jetson via remote desktop causes severe CPU contention and should not be used during real evaluation runs. Visualize from a separate machine on the same ROS 2 domain instead.

## License

Apache-2.0