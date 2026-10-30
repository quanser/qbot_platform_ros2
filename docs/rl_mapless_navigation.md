# Running RL mapless navigation

Run a trained RL navigation policy on the QBot Platform, either the physical robot or the virtual one
in Quanser Interactive Labs (QLabs). You click a goal in RViz and the policy drives the robot there
using the lidar, wheel speeds, gyro and the scan match pose. No map is needed.

The nodes and launch files used here are described in the package READMEs:
[`qbot_rl_deploy`](../qbot_rl_deploy/README.md), [`qbot_hardware`](../qbot_hardware/README.md) and [`qbot_localization`](../qbot_localization/README.md).

## Prerequisites

Both platforms need this repository built in a ROS 2 workspace (see [Build](#build)) and a trained
run folder (see [The run_dir folder](#the-run_dir-folder)).

### Physical QBot Platform

The QBot Platform runs ROS 2 Humble, and everything else is already installed on it. Build and run
the workspace on the QBot itself. To view RViz on a laptop instead of the QBot's desktop, the laptop
needs ROS 2 Humble (see [Option 2](#option-2-ssh-rviz-on-a-laptop)).

### Virtual QBot Platform (QLabs)

On a Linux computer:

- ROS 2 (tested with Jazzy).
- QUARC runtime, with `quarc_run` on `PATH`.
- QLabs installed and licensed, with the `QLabs` command on `PATH`. The launch file starts QLabs and
  loads the `Warehouse` module itself, so you do not open it first.
- [Quanser SDK](https://github.com/quanser/quanser_sdk) installed.
- PyTorch, importable by the Python that runs the ROS 2 nodes. If it is installed in a Python virtual
  environment, create that environment with `--system-site-packages` so the ROS 2 Python packages
  stay visible, and activate it before building and launching.
- The QLabs Python API (`qvl`), importable by `python3`. It ships with the [Quanser Academic Resources](https://github.com/quanser/quanser_academic_resources).
  Add their Python folder to `PYTHONPATH`, for example in `~/.bashrc`:

  ```bash
  export PYTHONPATH=$PYTHONPATH:<quanser_academic_resources>/0_libraries/python
  ```

The virtual RT models (`QBotPlatform_Workspace` and `qbot_platform_driver_virtual`, built for
`rt-linux_x86_64`) ship in `qbot_hardware/rt_models`.

## Build

Clone the `qbot_platform_ros2` repository into a ROS 2 workspace. The examples below use
`~/ros2_ws`.

```bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/quanser/qbot_platform_ros2.git ~/ros2_ws/src/qbot_platform_ros2
```

If PyTorch is in a Python virtual environment, activate it first. colcon writes the active Python
into the node entry points, which is how the policy nodes find PyTorch.

Source your ROS 2 distribution: `humble` on the physical QBot Platform, or the one installed on your
QLabs computer (tested with `jazzy`). The examples below use `<distro>` for it.

```bash
source /opt/ros/<distro>/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
```

## The run_dir folder

`run_dir` is the trained policy to deploy: a training run folder with `params/env.yaml` and
`exported/policy.pt`. See the [`qbot_rl_deploy` README](../qbot_rl_deploy/README.md#run_dir-contract)
for the exact contents.

An example trained run folder is attached as a zip to the
[releases](https://github.com/quanser/qbot_platform_ros2/releases). The release notes name the
file. Unzip it and pass the extracted folder as `run_dir`.

A relative `run_dir` is resolved against the folder you launch from. Use an absolute path, or
`~/...`, to launch from anywhere.

## Run in QLabs (virtual)

1. Open a terminal and source the workspace. Activate your PyTorch virtual environment first, if
   you use one.

   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. Launch:

   ```bash
   ros2 launch qbot_rl_deploy navigate_launch.py platform:=virtual launch_rviz:=true run_dir:=<run_dir>
   ```

   The launch then runs, in order:

   1. Launch QLabs with the `Warehouse` module.
   2. `qlabs_setup.py` stops old RT models, builds the arena, spawns the QBot and starts the virtual
      RT models. Wait for `qlabs_setup: completed.`
   3. The driver, lidar and fixed lidar frame start, then `scan_match_node` once they publish.
   4. RViz opens once the first pose arrives.

   The policy nodes start at the beginning, in parallel, so they are warm by the time RViz opens.

3. Send a goal. See [Sending a goal](#sending-a-goal).

4. Stop with Ctrl-C. QLabs closes with the launch.

The launch shuts itself down if the QLabs setup fails, if QLabs is closed, if the driver and lidar do
not publish within 60 s, or if no pose arrives within `rviz_wait_sec` (300 s).

## Run on the physical QBot Platform

1. Place the robot where you want the origin. The scan match pose is measured from where the robot
   is when `scan_match_node` calibrates.

2. Start the stack and open RViz with [Option 1](#option-1-remote-desktop-rviz-on-the-qbot) or
   [Option 2](#option-2-ssh-rviz-on-a-laptop) below, then continue with step 3. Either way,
   `platform` defaults to `hardware` and the launch runs, in order:

   1. The driver RT model starts with `quarc_run`.
   2. 2 s later, the driver, lidar and fixed lidar frame start, then `scan_match_node` once they
      publish.

   The policy nodes start at the beginning, in parallel. If the driver RT model is already running,
   for example started at boot, add `driver_model:=false`.

3. Send a goal. See [Sending a goal](#sending-a-goal).

4. Stop with Ctrl-C in the terminal running the launch. The driver RT model stops when the driver
   node exits.

The launch shuts itself down if the driver and lidar do not publish within 60 s.

### Option 1: remote desktop, RViz on the QBot

1. Connect to the QBot's desktop with a remote desktop client.
2. In a terminal on the QBot, source the workspace and launch with RViz:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch qbot_rl_deploy navigate_launch.py launch_rviz:=true run_dir:=<run_dir>
   ```

   RViz opens on the QBot's desktop once the first pose arrives.

This needs only the QBot, but RViz and the remote desktop share the QBot's CPU with the stack and
can slow it down. Use Option 2 for evaluation runs.

### Option 2: SSH, RViz on a laptop

On the QBot:

1. Open a terminal on the QBot over SSH:

   ```bash
   ssh nvidia@<qbot_ip>
   ```

2. Source the workspace and launch without RViz. `launch_rviz` is `false` by default:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch qbot_rl_deploy navigate_launch.py run_dir:=<run_dir>
   ```

On a laptop on the same network as the QBot:

1. Install ROS 2 Humble, to match the QBot.
2. Build the two RL packages of this repository. They are all RViz needs, so the Quanser components
   are not required on the laptop:

   ```bash
   mkdir -p ~/ros2_ws/src
   git clone https://github.com/quanser/qbot_platform_ros2.git ~/ros2_ws/src/qbot_platform_ros2
   source /opt/ros/humble/setup.bash
   cd ~/ros2_ws
   colcon build --packages-select qbot_rl_interfaces qbot_rl_deploy
   ```

3. Use the same `ROS_DOMAIN_ID` as the QBot. Check it on the QBot with `echo $ROS_DOMAIN_ID`. If it
   is empty, the QBot uses the default (0) and the laptop needs nothing set. Otherwise:

   ```bash
   export ROS_DOMAIN_ID=<qbot_domain_id>
   ```

4. Source the workspace and open RViz:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch qbot_rl_deploy navigate_rviz_only_launch.py
   ```

   The scan and the robot pose appear once the QBot publishes its first pose. Send goals from this
   RViz window.

## Sending a goal

1. In RViz, select **2D Goal Pose** and click the goal on the map. RViz publishes it on `/goal_pose`
   in the `odom` frame. Only the position is used. The arrow direction is ignored.
2. A green disc shows the goal. Its radius is the success radius (`client_success_radius_m`,
   0.25 m).
3. The robot drives until it is inside the success radius, or until `client_timeout_sec` (60 s)
   runs out. The terminal logs the result.

Things to know:

- A goal clicked before the action server is ready is ignored, with a warning. Click again.
- With `reset_scan_on_terminal:=true` (the default), `scan_match_node` recalibrates when each goal
  ends. The `odom` origin then moves to where the robot stopped, so the next goal is relative to that
  position. Set it to `false` to keep one origin for the whole session.

For all launch arguments, see the [`qbot_rl_deploy` README](../qbot_rl_deploy/README.md#launch)
or run `ros2 launch qbot_rl_deploy navigate_launch.py --show-args`.
