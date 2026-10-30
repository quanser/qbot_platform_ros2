# qbot_rl_deploy

RL policy deployment nodes for QBot Platform mapless navigation: observation building, policy
inference behind a ROS 2 action, and a goal client driven from RViz.

For prerequisites and step-by-step run instructions on the physical and virtual QBot Platform, see
[Running RL mapless navigation](../docs/rl_mapless_navigation.md).

## Overview

| Node | Role |
|---|---|
| `observation_builder` | Builds the policy observation from the lidar, scan match pose, wheel speeds, gyro and goal |
| `navigate_action_server` | Runs the TorchScript policy on each observation and publishes velocity commands while a goal is active |
| `navigate_action_client` | Turns each goal pose clicked in RViz into a `PolicyNavigate` action goal |

The pose comes from `qbot_localization`'s `scan_match_node`, and the sensor topics from
`qbot_hardware`. `navigate_launch.py` starts both through `qbot_hardware`'s
`qbot_platform_mapless_nav_bringup_launch.py`.

## Nodes

### `observation_builder`

Subscribes:
| Topic (param) | Default | Type | QoS |
|---|---|---|---|
| `scan_topic` | `/scan` | `sensor_msgs/LaserScan` | Best Effort, depth 1 |
| `pose_topic` | `pose_estimate` | `geometry_msgs/PoseStamped` | depth 1 |
| `goal_pose_topic` | `/goal_pose` | `geometry_msgs/PoseStamped` | depth 1 |
| `joint_topic` | `qbot_joint` | `sensor_msgs/JointState` | depth 2 |
| `imu_topic` | `qbot_imu` | `sensor_msgs/Imu` | depth 2 |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `obs_topic` | `policy/observation` | `qbot_rl_interfaces/ObservationStamped` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `run_dir` | (required) | Training run folder. See [run_dir contract](#run_dir-contract) |
| `scan_topic` | `/scan` | Input lidar scan topic |
| `pose_topic` | `pose_estimate` | Input pose topic |
| `goal_pose_topic` | `/goal_pose` | Input goal topic |
| `joint_topic` | `qbot_joint` | Input wheel joint state topic |
| `imu_topic` | `qbot_imu` | Input IMU topic |
| `obs_topic` | `policy/observation` | Output observation topic |
| `data_timeout_sec` | `0.1` | Maximum age of the lidar, pose, joint and IMU inputs. No observation is published while any is older |
| `scan_to_base_tx` | `0.15` | Lidar to base x offset (m) |
| `scan_to_base_ty` | `0.0` | Lidar to base y offset (m) |
| `scan_to_base_yaw` | `pi/2` | Lidar to base yaw (rad) |

An observation is published each time a new pose arrives, since the scan match pose is the slowest
input. A 2 Hz timer covers the case where poses stop. Nothing is published until a goal has been
received. The observation layout, in policy input order:

| Component | Length | Content |
|---|---|---|
| `lidar` | `obs_dim - 5` | Ranges binned by angle around the base, normalized by the lidar max range |
| `goal_dist` | 1 | Distance from the robot to the goal, normalized by the lidar max range |
| `goal_heading` | 1 | Heading to the goal in the robot frame, normalized by pi |
| `wheel_speed` | 2 | Left and right wheel speeds, normalized by `wheel_speed_obs_max` |
| `gyro` | 1 | Yaw rate, normalized by `gyro_yaw_rate_obs_max` |

Each message also carries the component names, start indices, lengths and source stamps.

### `navigate_action_server`

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `obs_topic` | `policy/observation` | `qbot_rl_interfaces/ObservationStamped` |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `cmd_topic` | `cmd_vel` | `geometry_msgs/Twist` |

Actions:
| Action | Type |
|---|---|
| `policy_navigate` (fixed) | `qbot_rl_interfaces/PolicyNavigate` |

Service clients:
| Service (param) | Default | Type |
|---|---|---|
| `scan_match_reset_service` | `/reset_scan_match` | `std_srvs/Trigger` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `run_dir` | (required) | Training run folder. See [run_dir contract](#run_dir-contract) |
| `obs_topic` | `policy/observation` | Input observation topic |
| `cmd_topic` | `cmd_vel` | Output velocity command topic |
| `feedback_rate_hz` | `10.0` | Action feedback rate |
| `data_timeout_sec` | `0.2` | Maximum observation age. Older observations are not run through the policy |
| `action_no_data_abort_sec` | `2.0` | Minimum time to wait for the first fresh observation after a goal is accepted. The goal ends as a timeout if none arrives |
| `publish_zero_on_missing_data` | `True` | Publish a zero command while the observation is missing or too old |
| `scan_match_reset_service` | `/reset_scan_match` | Reset service of the localization node |
| `scan_match_reset_wait_sec` | `2.0` | Time to wait for the reset service |
| `reset_scan_on_terminal` | `True` | Reset the localization when a goal ends, which moves the `odom` origin to the robot's current pose |

While a goal is active, the policy runs at `1 / (sim.dt * decimation)` from the training config. Its
outputs are treated as normalized to [-1, 1] and scaled by `max_lin_vel` and `max_ang_vel`. A goal
ends when the robot is within `success_radius_m` of it, when `timeout_sec` runs out, or when it is
canceled.

### `navigate_action_client`

Subscribes:
| Topic (param) | Default | Type |
|---|---|---|
| `goal_pose_topic` | `/goal_pose` | `geometry_msgs/PoseStamped` |

Publishes:
| Topic (param) | Default | Type |
|---|---|---|
| `goal_marker_topic` | `policy_goal_marker` | `visualization_msgs/Marker` |

Parameters:
| Parameter | Default | Description |
|---|---|---|
| `goal_pose_topic` | `/goal_pose` | Goal topic, published by RViz's 2D Goal Pose tool |
| `action_name` | `policy_navigate` | Action to send goals to |
| `timeout_sec` | `60.0` | Time limit sent with each goal |
| `success_radius_m` | `0.25` | Success radius sent with each goal, also drawn as the goal marker |
| `goal_marker_topic` | `policy_goal_marker` | Goal marker topic |

A goal received while the action server is not ready is ignored with a warning.

## run_dir contract

`run_dir` is a training run folder. The nodes read:

```
<run_dir>/
├── params/
│   └── env.yaml        # training environment config
└── exported/
    └── policy.pt       # TorchScript policy
```

Required keys in `env.yaml`:

| Key | Used by | Purpose |
|---|---|---|
| `obs_dim` | both | Observation length. The lidar length is `obs_dim - 5` |
| `scene.lidar_sensor.max_distance` | both | Lidar range normalization |
| `wheel_speed_obs_max` | `observation_builder` | Wheel speed normalization |
| `gyro_yaw_rate_obs_max` | `observation_builder` | Yaw rate normalization |
| `max_lin_vel`, `max_ang_vel` | `navigate_action_server` | Command scaling |
| `sim.dt`, `decimation` | `navigate_action_server` | Control rate |

Other files in the folder are ignored. A relative `run_dir` is resolved against the current working
directory.

## Launch

**`navigate_launch.py`** brings up the full stack: the platform bringup (driver, lidar, fixed lidar
frame and `scan_match_node`, plus QLabs for `platform:=virtual`), the three nodes above and,
optionally, RViz. The policy nodes start immediately. RViz starts once the first pose arrives.

The `platform` argument selects the physical QBot Platform or the virtual one in QLabs. For
`platform:=hardware` the launch starts the driver RT model itself. For `platform:=virtual` it starts
QLabs and sets up the arena itself.

```bash
# Physical QBot Platform (default)
ros2 launch qbot_rl_deploy navigate_launch.py run_dir:=<run_dir>

# Virtual QBot Platform in QLabs
ros2 launch qbot_rl_deploy navigate_launch.py platform:=virtual launch_rviz:=true run_dir:=<run_dir>
```

See [Running RL mapless navigation](../docs/rl_mapless_navigation.md) for the full run steps on
each platform.

Main arguments (run with `--show-args` for the full list):

| Argument | Default | Purpose |
|---|---|---|
| `run_dir` | empty | Required. Training run folder. |
| `platform` | `hardware` | `hardware` or `virtual`. |
| `launch_rviz` | `false` | Start RViz on this machine. |
| `rviz_wait_sec` | `300.0` | With `launch_rviz:=true`, seconds to wait for the first pose before RViz starts. The launch shuts down if none arrives. |
| `client_timeout_sec` | `60.0` | Time limit for each goal. |
| `client_success_radius_m` | `0.25` | Distance at which a goal counts as reached. |
| `reset_scan_on_terminal` | `true` | Recalibrate the scan match origin when each goal ends. |
| `driver_model` | `true` | Hardware only. Start the physical driver RT model before the nodes and stop it on exit. Use `false` if it is already running. |
| `qlabs_launch` | `true` | Virtual only. Start QLabs with `qlabs_module` first. It is skipped when QLabs is already running and QLabs is closed when the launch ends. |
| `qlabs_module` | `Warehouse` | Virtual only. Module passed to `QLabs -loadmodule`. |
| `qlabs_exe` | `QLabs` | Virtual only. QLabs executable name or path. |
| `qlabs_setup` | `true` | Virtual only. Run `qlabs_setup.py` (from `qbot_hardware`) to spawn the QBot and start its RT models before the nodes. Use `false` if the QBot is already spawned and its models are running. |
| `qlabs_host` | `localhost` | Virtual only. Host running QLabs. |
| `qlabs_setup_args` | empty | Virtual only. Extra `qlabs_setup.py` arguments, for example `'--x 1.0 --yaw-deg 180'`. |
| `agent_type` | empty | Override the driver and lidar `agent_type` (0 virtual, 1 physical). Empty derives it from `platform`. |

**`navigate_rviz_only_launch.py`** starts only RViz with the `set_goal.rviz` config, for a separate
computer on the same ROS 2 domain as the robot.

```bash
ros2 launch qbot_rl_deploy navigate_rviz_only_launch.py
```

## Dependency

- **`qbot_rl_interfaces`**: the `PolicyNavigate` action and `ObservationStamped` message.
- **`qbot_hardware`** and **`qbot_localization`**: `navigate_launch.py` includes the platform bringup
  and the nodes publish or consume their topics.
- **PyTorch**: `navigate_action_server` loads the TorchScript policy.

## License

Apache-2.0
