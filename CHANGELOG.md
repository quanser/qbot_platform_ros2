# Changelog

All notable changes to this repository are recorded here. Versions follow the release tags.

## [0.2.0] - 2026-10-30

This repository, renamed from `qbot_rl_deploy` to `qbot_platform_ros2`, is now the main ROS 2
repository for the QBot Platform. The RL deployment packages (`qbot_rl_deploy`, `qbot_rl_interfaces`)
and the platform packages (`qbot_hardware`, `qbot_localization`) were developed in separate
repositories and are merged here.

### Added

- `qbot_hardware` package: ROS 2 interface to the QBot Platform.
- `qbot_localization` package: `scan_match_node` (scan match localization against a reference scan)
  and `ekf_localization_node` (EKF fusion through the `ekf_scan_match` RT model, shipped for both
  `rt-linux_qbot_platform` and `rt-linux_x86_64`).
- Support for both virtual and hardware QBot platforms through the `platform` launch argument (`hardware` by default, or `virtual`).
- For `platform:=hardware`, the bringup starts the physical driver RT model before the nodes and stops
  it when the driver node exits (`driver_model`, default `true`).
- Run guide `docs/rl_mapless_navigation.md`: prerequisites, build, separate virtual and hardware run
  steps, and how to send a goal.
- `qbot_rl_deploy` README describing each node's topics, parameters and actions, the `run_dir`
  contract and the launch arguments. This changelog.

### Changed

- `navigate_launch.py` no longer starts the driver, lidar, fixed lidar frame and localization itself.
  It includes the platform bringup and forwards the platform, QLabs and topic arguments.
- `scan_match_node` always recalibrates its reference scan at launch (`calibrate: True`), so the
  `odom` origin is where the robot is when the stack starts.
- `/scan` is published Best Effort (`SensorDataQoS`). All subscribers in this repo and the RViz config
  are Best Effort. A Reliable `/scan` subscriber outside this repo no longer receives scans.
- Smaller queue depths on latest-value topics (depth 1 or 2 instead of 10 or 20) in the driver,
  `scan_match_node`, `observation_builder`, the action server and the action client, so a stalled
  node does not process a backlog of old messages.
- `observation_builder` publishes when a new pose arrives instead of when a new scan arrives, since
  the scan match pose is the slowest input.
- `navigate_action_server`:
  - The control rate is computed from the training config as `1 / (sim.dt * decimation)` instead of
    `100 / decimation`.
  - Policy outputs are treated as normalized to [-1, 1] and scaled by `max_lin_vel` and
    `max_ang_vel`.
- `navigate_action_client` ignores a goal, with a warning, when the action server is not ready, and
  logs feedback at most once per second.
- Driver and lidar nodes: the `driver_uri` and `uri` parameters are replaced by `agent_type` and
  `port_number`. The physical URIs are now fixed in the nodes.
- Lidar node: the publish loop runs at 10 Hz.

### Fixed

- `ekf_localization_node` subscribed to `/scan` as Reliable and received no scans from the Best
  Effort lidar publisher.
- `qbot_platform_manual_drive_launch.py` and `qbot_platform_manual_map_launch.py` looked up the driver
  RT model in the old `qbot_platform` package, which no longer exists.
- `qbot_platform_launch.py` (and the Cartographer and Nav2 launch files that include it),
  `qbot_platform_manual_drive_launch.py` and `qbot_platform_manual_map_launch.py` now set
  `agent_type: 1` on the driver and lidar. Without it the nodes defaulted to the virtual connection,
  so the lidar did not open `/dev/lidar` on a physical QBot.
- `scan_match_node` now saves to `$ROS_HOME/qbot_localization/reference_scan.npy` (`~/.ros` by
  default), set by the new `reference_scan_path` parameter.

### Removed

- Unused RT model start and stop code in `navigate_launch.py`.
- Compiled Python files that were tracked in `qbot_rl_deploy`.

## [0.1.0]

Released as the `qbot_rl_deploy` repository. Initial public release of the RL policy deployment stack for QBot Platform mapless navigation.

### Added

- `qbot_rl_deploy`: `observation_builder`, `navigate_action_server`, `navigate_action_client` and
  `navigate_launch.py`.
- `qbot_rl_interfaces`: `PolicyNavigate` action and `ObservationStamped` message.
- Example trained policy `2026-06-22_17-06-20.zip`, attached to the release.
