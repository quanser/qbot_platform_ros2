# qbot_rl_deploy

RL policy deployment nodes for mapless navigation on the QBot Platform.

## Overview

This package deploys a trained RL navigation policy on the QBot Platform. A user sets a goal pose in RViz2 (in the robot's odom frame); `observation_builder` combines the goal, lidar, wheel speed, and IMU data into an observation, which `navigate_action_server` feeds to a TorchScript-compiled policy to produce velocity commands, navigating to the goal while avoiding static obstacles, without a map.

This package depends on topics published by [`qbot_hardware`](https://github.com/quanser/qbot_hardware.git) (lidar, joint states, IMU) and its `qbot_localization` package (pose estimate). It cannot run standalone.

## Nodes

### `observation_builder`

Subscribes to goal, sensor, and localization topics, builds the normalized observation vector expected by the trained policy, and publishes it.

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `scan_topic` | `/scan` | `sensor_msgs/LaserScan` |
| `goal_pose_topic` | `/goal_pose` | `geometry_msgs/PoseStamped` |
| `pose_topic` | `pose_estimate` | `geometry_msgs/PoseStamped` |
| `joint_topic` | `qbot_joint` | `sensor_msgs/JointState` |
| `imu_topic` | `qbot_imu` | `sensor_msgs/Imu` |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `obs_topic` | `policy/observation` | `qbot_rl_interfaces/ObservationStamped` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `run_dir` | *(required)* | Path to the training run directory; loads training config for normalization  |
| `data_timeout_sec` | `0.1` | Max age of input sensor data before considered stale |
| `scan_to_base_tx` | `0.15` | Lidar-to-`base_link` static transform (x), matching `fixed_lidar_frame` in `qbot_hardware` |
| `scan_to_base_ty` | `0.0` | Lidar-to-`base_link` static transform (y) |
| `scan_to_base_yaw` | `π/2` | Lidar-to-`base_link` static transform (yaw) |

Observation layout: `obs_dim` (from `run_dir` config) = lidar beams + `goal_dist` + `goal_heading` + `wl` + `wr` + `yaw_rate` (5 non-lidar components). Publishing is event-driven off `pose_topic` (slowest component), with a fallback watchdog at 2 Hz.

### `navigate_action_client`

Example/reference client. Subscribes to the same RViz2 goal topic and uses it only to trigger the `policy_navigate` action, does not consume observations or sensor data directly. Publishes an RViz marker showing the current goal and success radius.

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `goal_pose_topic` | `/goal_pose` | `geometry_msgs/PoseStamped` |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `goal_marker_topic` | `policy_goal_marker` | `visualization_msgs/Marker` |

Action client:
| Action (param) | Default | Type |
|---|---|---|
| `action_name` | `policy_navigate` | `qbot_rl_interfaces/PolicyNavigate` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `timeout_sec` | `60.0` | Sent as part of the goal; server-side timeout before aborting |
| `success_radius_m` | `0.25` | Sent as part of the goal; distance-to-goal threshold for success |

This node is a reference implementation for interfacing with `navigate_action_server`. `timeout_sec` and `success_radius_m` are deliberately client-specified so they can be tuned per-call without restarting the policy server.

### `navigate_action_server`

Loads the trained TorchScript policy from `run_dir` and runs inference, publishing velocity commands. Also manages scan-match reset at episode boundaries.

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `obs_topic` | `policy/observation` | `qbot_rl_interfaces/ObservationStamped` |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `cmd_topic` | `cmd_vel` | `geometry_msgs/Twist` |

Calls:
| Service (param) | Default | Type |
|---|---|---|
| `scan_match_reset_service` | `/reset_scan_match` | `std_srvs/Trigger` (hosted by `qbot_localization`) |

Action server:
| Action (param) | Default | Type |
|---|---|---|
| `action_name` | `policy_navigate` | `qbot_rl_interfaces/PolicyNavigate` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `run_dir` | *(required)* | Path to the training run directory; TorchScript policy loaded from `run_dir/exported/policy.pt` |
| `feedback_rate_hz` | `10.0` | Rate of action feedback publishing |
| `tick_rate_hz` | `100 / decimation` | Control loop rate, derived from the training run's `decimation` config |
| `data_timeout_sec` | `0.2` | Max age of incoming observation data before considered stale |
| `action_no_data_abort_sec` | `2.0` | Abort the action if no valid observation is received for this long |
| `scan_match_reset_wait_sec` | `2.0` | Wait time after calling the reset service before resuming |
| `reset_scan_on_terminal` | `True` | Whether to call the scan-match reset service on episode termination (success/abort) |
| `publish_zero_on_missing_data` | `True` | Publish zero velocity command instead of stale command when observation data is missing |


## Policy Run Directory

Both `observation_builder` and `navigate_action_server` take a `run_dir`
parameter pointing at the same trained policy directory:

```
<run_dir>/
├── exported/
│   └── policy.pt          # TorchScript-compiled policy, loaded by navigate_action_server
├── params/
│   └── env.yaml           # obs_dim, max_lin_vel, max_ang_vel, lidar_max_range,
                           # wheel_speed_obs_max, gyro_yaw_rate_obs_max, decimation, etc.
                           # loaded by both nodes via _load_env_config()
```

A trained `run_dir` can be downloaded from qbot_rl_deploy [GitHub Releases](https://github.com/quanser/qbot_rl_deploy/releases)

## Launch

**`navigate_launch.py`** brings up `observation_builder`, `navigate_action_server`, and `navigate_action_client` for running a trained navigation policy. 

```bash
ros2 launch qbot_rl_deploy navigate_launch.py run_dir:=<path-to-run-dir>
```
By default, this launch file does not start RViz2 (to avoid CPU contention on the QBot's Jetson). To run RViz2 on the QBot, set `launch_rviz:=true` (not recommended for real evaluation runs).

To run RViz2 on a remote workstation, use **`navigate_rviz_only_launch.py`**:

```bash 
ros2 launch qbot_rl_deploy navigate_rviz_only_launch.py
```

## Dependency

- Requires `qbot_hardware` (lidar, joint states, IMU topics) and its `qbot_localization` package (pose estimate, scan-match reset service) to be running. See that repo's README for the corresponding launch file.

- Requires PyTorch / TorchScript for policy inference.

## License

Apache-2.0