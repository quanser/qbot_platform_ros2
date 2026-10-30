# Platform bringup for mapless navigation: the QBot driver, lidar and fixed lidar frame, plus the
# scan match localization, which tracks the pose against a reference scan instead of a map.
#
# platform:=hardware
#   1. The physical driver RT model, stopped again when the driver node exits.
#   2. Driver, lidar and fixed lidar frame, 2 s after the model starts.
#   3. With localization:=true, scan_match_node once the driver and lidar are publishing.
#
# platform:=virtual
#   1. QLabs with the chosen module, closed again when the launch ends.
#   2. scripts/qlabs_setup.py builds the arena, spawns the QBot and starts the virtual RT models.
#   3. Steps 2 and 3 of the hardware sequence, against QLabs and without the 2 s delay.

import os
import shlex
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

# agent_type parameter of the driver and lidar nodes: virtual (0) or physical (1).
PLATFORM_AGENT_TYPE = {"hardware": 1, "virtual": 0}

QUARC_TARGET = "tcpip://localhost:17000"
PHYSICAL_DRIVER_MODEL = "qbot_platform_driver_physical"


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _flag(context, name):
    return _arg(context, name).lower() in ("true", "1", "yes")


def _shutdown(reason, message):
    return [LogInfo(msg=message), EmitEvent(event=Shutdown(reason=reason))]


# Platform nodes ------------------------------------------------------------------------------------


def _platform_nodes(platform, agent_type, driver_port, lidar_port):
    driver_params = {
        "agent_type": agent_type,
        "arm_robot": True,
        "speed_limit_mode": 0,
        "body_speed_duration": 0.1,
    }
    lidar_params = {"agent_type": agent_type}
    if platform == "virtual":
        # The ports only select the QLabs connection. The physical URIs are fixed in the nodes.
        driver_params["port_number"] = driver_port
        lidar_params["port_number"] = lidar_port

    return [
        Node(
            package="qbot_hardware",
            executable="qbot_platform_driver_interface",
            name="qbot_platform_driver",
            parameters=[driver_params],
        ),
        Node(
            package="qbot_hardware",
            executable="lidar",
            name="lidar_node",
            parameters=[lidar_params],
        ),
        Node(
            package="qbot_hardware",
            executable="fixed_lidar_frame",
            name="fixed_lidar_frame",
        ),
    ]


# Physical driver RT model (platform:=hardware) ------------------------------------------------------


def _stop_driver_model(_context):
    # An OpaqueFunction still runs while launch shuts down after Ctrl-C, when no new ExecuteProcess
    # can start. Ignore errors, since the model may already have stopped.
    subprocess.run(["quarc_run", "-q", "-t", QUARC_TARGET, PHYSICAL_DRIVER_MODEL], capture_output=True)


def _driver_model_actions(nodes, driver_node):
    """Start the physical driver RT model, then the nodes 2 s later, as qbot_platform_launch.py does.

    The driver node connects to the model only once, at startup, so the model must be running first.
    The model is stopped when the driver node exits.
    """
    model = PathJoinSubstitution(
        [FindPackageShare("qbot_hardware"), "rt_models", f"{PHYSICAL_DRIVER_MODEL}.rt-linux_qbot_platform"]
    )
    model_start = ExecuteProcess(
        cmd=["quarc_run", f"-r -t {QUARC_TARGET}", model, "-d %d -uri tcpip://%m:17001"],
        name="QBotPlatformDriverModelStart",
        shell=True,
    )
    return [
        model_start,
        RegisterEventHandler(
            OnProcessStart(
                target_action=model_start,
                on_start=[
                    LogInfo(msg="Driver RT model starting. Starting the driver, lidar and fixed lidar frame in 2 s."),
                    TimerAction(period=2.0, actions=nodes),
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=driver_node,
                on_exit=[
                    OpaqueFunction(function=_stop_driver_model),
                    LogInfo(msg="Driver node exited. Stopping the driver RT model."),
                ],
            )
        ),
    ]


# Localization --------------------------------------------------------------------------------------


def _localization_actions(context):
    """Start scan_match_node only after the driver and lidar are publishing.

    scan_match_node builds its reference scan from the first scan it receives, which defines the
    origin of the pose frame, so it must not start before the robot and lidar are up. A shell
    process waits for one speed feedback message and one scan. When it succeeds, the node starts.
    """
    speed_topic = _arg(context, "speed_topic")
    scan_topic = _arg(context, "scan_topic")
    timeout = float(_arg(context, "localization_wait_sec"))
    # ros2 topic echo exits at once when the topic does not exist yet, so retry until it does.
    wait_cmd = (
        f"timeout {timeout:g} sh -c "
        + shlex.quote(
            'for t in "$@"; do until ros2 topic echo --once "$t" > /dev/null 2>&1; do sleep 1; done; done'
        )
        + f" sh {shlex.quote(speed_topic)} {shlex.quote(scan_topic)}"
    )
    wait = ExecuteProcess(cmd=[wait_cmd], shell=True, name="WaitForPlatform", output="screen")
    localization = Node(
        package="qbot_localization",
        executable="scan_match_node",
        name="scan_match_node",
        parameters=[
            {
                # Always rebuild the reference scan, which defines the origin of the pose frame.
                "calibrate": True,
                "odom_pub": LaunchConfiguration("odom_pub"),
                "tf_pub": LaunchConfiguration("scan_tf_pub"),
                "speed_topic": speed_topic,
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": scan_topic,
            }
        ],
    )

    def on_wait_exit(event, _context):
        if event.returncode != 0:
            return _shutdown(
                "platform not publishing", f"Driver or lidar did not publish within {timeout:g}s. Shutting down."
            )
        return [LogInfo(msg="Driver and lidar are publishing. Starting scan_match_node."), localization]

    return [
        LogInfo(msg=f"Waiting for '{speed_topic}' and '{scan_topic}' before starting scan_match_node."),
        wait,
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=on_wait_exit)),
    ]


# QLabs (platform:=virtual) -------------------------------------------------------------------------


def _qlabs_running(exe):
    # pgrep -x matches names truncated to 15 characters.
    name = os.path.basename(exe)[:15]
    return subprocess.run(["pgrep", "-x", name], capture_output=True).returncode == 0


def _qlabs_command(exe, module):
    """Command that starts QLabs, and a pattern matching its process for cleanup.

    The QLabs command is a shell wrapper that runs the real binary as a child without exec, so
    signals sent by launch stop the wrapper and leave QLabs open. Run the binary directly when it
    can be found.
    """
    path = shutil.which(exe)
    if path:
        binary = os.path.join(os.path.dirname(os.path.realpath(path)), "QLabs", "Binaries", "Linux", "QLabs")
        if os.access(binary, os.X_OK):
            return [binary, "QLabs", "-loadmodule", module], binary
    return [exe, "-loadmodule", module], None


def _stop_qlabs(pattern):
    def stop(_context):
        subprocess.run(["pkill", "-f", pattern], capture_output=True)

    return stop


def _qlabs_actions(context):
    """Start QLabs with the chosen module unless it is already running. It is closed with the launch."""
    if not _flag(context, "qlabs_launch"):
        return []
    qlabs_exe = _arg(context, "qlabs_exe")
    qlabs_module = _arg(context, "qlabs_module")
    if _qlabs_running(qlabs_exe):
        return [LogInfo(msg="QLabs is already running. Not starting another instance.")]

    # QLabs is long running, so it cannot gate the setup script. The script retries the connection
    # until QLabs has loaded the module.
    qlabs_cmd, qlabs_pattern = _qlabs_command(qlabs_exe, qlabs_module)
    qlabs = ExecuteProcess(cmd=qlabs_cmd, name="QLabs", output="screen", sigterm_timeout="10", sigkill_timeout="5")
    actions = [
        LogInfo(msg=f"Starting QLabs with module '{qlabs_module}'."),
        qlabs,
        RegisterEventHandler(
            OnProcessExit(target_action=qlabs, on_exit=_shutdown("QLabs exited", "QLabs exited. Shutting down."))
        ),
    ]
    if qlabs_pattern:
        # Safety net for a QLabs process that outlives the signals launch sends.
        actions.append(
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_stop_qlabs(qlabs_pattern))]))
        )
    return actions


def _qlabs_setup_actions(context, nodes):
    """Run qlabs_setup.py, then start the nodes if it succeeds or shut down if it fails."""
    setup_script = PathJoinSubstitution([FindPackagePrefix("qbot_hardware"), "lib", "qbot_hardware", "qlabs_setup.py"])
    setup_cmd = [setup_script, "--host", _arg(context, "qlabs_host")]
    setup_cmd += shlex.split(_arg(context, "qlabs_setup_args"))
    setup = ExecuteProcess(cmd=setup_cmd, name="QLabsSetup", output="screen")

    def on_setup_exit(event, _context):
        if event.returncode != 0:
            return _shutdown(
                "QLabs setup failed", f"QLabsSetup failed with exit code {event.returncode}. Shutting down."
            )
        return [LogInfo(msg="QLabsSetup finished. Starting QBot driver, lidar and fixed lidar frame.")] + nodes

    return [setup, RegisterEventHandler(OnProcessExit(target_action=setup, on_exit=on_setup_exit))]


# Launch description --------------------------------------------------------------------------------


def _launch_setup(context):
    platform = _arg(context, "platform")
    if platform not in PLATFORM_AGENT_TYPE:
        raise RuntimeError(f"Launch argument 'platform' must be 'hardware' or 'virtual', got '{platform}'.")
    agent_type_arg = _arg(context, "agent_type").strip()
    agent_type = int(agent_type_arg) if agent_type_arg else PLATFORM_AGENT_TYPE[platform]

    nodes = _platform_nodes(platform, agent_type, int(_arg(context, "driver_port")), int(_arg(context, "lidar_port")))
    driver_node = nodes[0]
    if _flag(context, "localization"):
        nodes += _localization_actions(context)
    actions = [LogInfo(msg=f"qbot_platform_mapless_nav_bringup: platform={platform}, agent_type={agent_type}")]

    if platform == "hardware":
        if _flag(context, "driver_model"):
            return actions + _driver_model_actions(nodes, driver_node)
        return actions + nodes
    if not _flag(context, "qlabs_setup"):
        return actions + nodes
    return actions + _qlabs_actions(context) + _qlabs_setup_actions(context, nodes)


def generate_launch_description():
    declare_args = [
        # Platform
        DeclareLaunchArgument(
            "platform",
            default_value="hardware",
            description="'hardware' for the physical QBot Platform, 'virtual' for QLabs.",
        ),
        DeclareLaunchArgument(
            "agent_type",
            default_value="",
            description="Override the agent_type of the driver and lidar (0 virtual, 1 physical). "
            "Empty derives it from 'platform'.",
        ),
        DeclareLaunchArgument(
            "driver_port", default_value="18888", description="QLabs port of the QBot driver (virtual only)."
        ),
        DeclareLaunchArgument("lidar_port", default_value="18918", description="QLabs port of the lidar (virtual only)."),
        DeclareLaunchArgument(
            "driver_model",
            default_value="true",
            description="Start the physical driver RT model before the nodes and stop it when the driver "
            "node exits (hardware only). Use false if it is already running.",
        ),
        # Localization
        DeclareLaunchArgument(
            "localization",
            default_value="false",
            description="Start scan_match_node after the driver and lidar are publishing.",
        ),
        DeclareLaunchArgument(
            "localization_wait_sec",
            default_value="60.0",
            description="Seconds to wait for the driver and lidar before giving up.",
        ),
        DeclareLaunchArgument("speed_topic", default_value="/qbot_speed_feedback"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("imu_topic", default_value="qbot_imu"),
        DeclareLaunchArgument("odom_pub", default_value="true"),
        DeclareLaunchArgument("scan_tf_pub", default_value="true"),
        # QLabs (virtual only)
        DeclareLaunchArgument(
            "qlabs_launch",
            default_value="true",
            description="Start QLabs with 'qlabs_module' (virtual only). It is skipped if QLabs is "
            "already running and is closed when the launch ends.",
        ),
        DeclareLaunchArgument("qlabs_module", default_value="Warehouse", description="QLabs module to load."),
        DeclareLaunchArgument("qlabs_exe", default_value="QLabs", description="QLabs executable name or path."),
        DeclareLaunchArgument(
            "qlabs_setup",
            default_value="true",
            description="Run qlabs_setup.py before the nodes (virtual only). Use false if the "
            "QBot is already spawned and its driver model is running.",
        ),
        DeclareLaunchArgument(
            "qlabs_host", default_value="localhost", description="Host running QLabs (virtual only)."
        ),
        DeclareLaunchArgument(
            "qlabs_setup_args",
            default_value="",
            description="Extra arguments for qlabs_setup.py, e.g. '--x 1.0 --yaw-deg 180 --no-rt-model'.",
        ),
    ]

    return LaunchDescription(declare_args + [OpaqueFunction(function=_launch_setup)])
