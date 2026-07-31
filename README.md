# qbot_rl_deploy

ROS 2 packages for deploying trained RL navigation policies on the QBot Platform.

## Overview

This repo provides the ROS 2 nodes that run a trained RL mapless-navigation policy on the QBot Platform: building the policy's observation, running TorchScript inference, and executing goal-directed navigation via a ROS 2 action interface.

This repo depends on sensor and pose topics published by [`qbot_hardware`](https://github.com/quanser/qbot_hardware.git) and its `qbot_localization` package (lidar, joint states, IMU, pose estimate, and the scan-match reset service). It cannot run standalone, `qbot_hardware` must be built and running in the same workspace.


## Packages in this repo

| Package | Description |
|---|---|
| [`qbot_rl_deploy`](./qbot_rl_deploy/README.md) | Deployment nodes for trained RL navigation policies |
| [`qbot_rl_interfaces`](./qbot_rl_interfaces/README.md) |Custom observation message and RL policy action definitions |

## Prerequisites

- ROS 2 Humble (tested distro; no known incompatibility with other ROS 2 distros)
- PyTorch with TorchScript support
- [`qbot_hardware`](https://github.com/quanser/qbot_hardware.git) built and running in the same workspace

## Getting Started

Source ROS 2 Humble before building or running:

```bash
source /opt/ros/humble/setup.bash
```

### Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select qbot_rl_interfaces qbot_rl_deploy
source install/setup.bash
```

### Running

It is recommended to launch `navigate_launch.py` on the edge device without RViz2 (default), and launch `navigate_rviz_only_launch.py` on a remote workstation on the same network.

##### Run on QBot Platform:
```bash
ros2 launch qbot_rl_deploy navigate.launch.py run_dir:=<path-to-trained-run-dir>
```
##### Run on Remote Workstation:
```bash
ros2 launch qbot_rl_deploy navigate_rviz_only.launch.py
```
It is also possible to run RViz2 on the QBot Platform, but this is not recommended due to the high CPU load of RViz2 and the impact on the localization nodes:
```bash
ros2 launch qbot_rl_deploy navigate.launch.py run_dir:=<path-to-trained-run-dir> launch_rviz:=true
``` 

Then set a goal pose in RViz2 to trigger navigation via `navigate_action_client` (or your own client calling the `policy_navigate`
action). See [`qbot_rl_deploy`'s README](./qbot_rl_deploy/README.md) for node details, parameters, and the expected `run_dir` structure.

A trained `run_dir` can be downloaded from qbot_rl_deploy [GitHub Releases](https://github.com/quanser/qbot_rl_deploy/releases)

## Known Limitations / Operational Notes

- Running RViz locally on the QBot's onboard Jetson via remote desktop causes severe CPU contention and should not be used during real evaluation runs. Visualize from a separate machine on the same ROS 2 domain instead.

## License

Apache-2.0