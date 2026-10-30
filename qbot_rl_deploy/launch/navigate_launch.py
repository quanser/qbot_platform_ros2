import shlex
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def wait_for_localization(nodes):
    """Start RViz only once scan_match_node publishes a pose.

    The bringup starts QLabs, the arena, the driver, the lidar and scan_match_node in sequence, which
    takes a while. An empty RViz window during that time looks like a fault, so a shell process waits
    for one pose message first. The policy nodes start with the bringup so their warm-up overlaps it.
    """

    def setup(context):
        if LaunchConfiguration("launch_rviz").perform(context).lower() not in ("true", "1", "yes"):
            return []
        pose_topic = LaunchConfiguration("pose_topic").perform(context)
        timeout = float(LaunchConfiguration("rviz_wait_sec").perform(context))
        # ros2 topic echo exits at once when the topic does not exist yet, so retry until it does.
        wait_cmd = (
            f"timeout {timeout:g} sh -c "
            + shlex.quote('until ros2 topic echo --once "$1" > /dev/null 2>&1; do sleep 1; done')
            + f" sh {shlex.quote(pose_topic)}"
        )
        wait = ExecuteProcess(cmd=[wait_cmd], shell=True, name="WaitForLocalization", output="screen")

        def on_wait_exit(event, _context):
            if event.returncode != 0:
                return [
                    LogInfo(msg=f"No pose on '{pose_topic}' within {timeout:g}s. Shutting down."),
                    EmitEvent(event=Shutdown(reason="localization not publishing")),
                ]
            return [LogInfo(msg="Localization is up. Starting RViz.")] + nodes

        return [
            LogInfo(msg=f"Waiting for a pose on '{pose_topic}' before starting RViz."),
            wait,
            RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=on_wait_exit)),
        ]

    return OpaqueFunction(function=setup)


def generate_launch_description():

    # Declare launch arguments with default values
    declare_args = [
        # Platform selection. 'hardware' is the physical QBot Platform, 'virtual' is QLabs.
        DeclareLaunchArgument(
            'platform',
            default_value='hardware',
            description="'hardware' for the physical QBot Platform, 'virtual' for QLabs.",
        ),
        # Forwarded to qbot_hardware's qbot_platform_mapless_nav_bringup_launch.py.
        DeclareLaunchArgument('agent_type', default_value='',
                              description="Override driver/lidar agent_type (0 virtual, 1 physical). Empty derives it from 'platform'."),
        DeclareLaunchArgument('driver_model', default_value='true',
                              description='Start the physical driver RT model before the nodes and stop it on exit (hardware only). Use false if it is already running.'),
        DeclareLaunchArgument('qlabs_setup', default_value='true',
                              description='Run the QLabs spawn script first (virtual only).'),
        DeclareLaunchArgument('qlabs_launch', default_value='true',
                              description='Start QLabs with qlabs_module (virtual only). Closed with the launch.'),
        DeclareLaunchArgument('qlabs_module', default_value='Warehouse'),
        DeclareLaunchArgument('qlabs_exe', default_value='QLabs'),
        DeclareLaunchArgument('qlabs_host', default_value='localhost'),
        DeclareLaunchArgument('qlabs_setup_args', default_value='',
                              description="Extra arguments for qlabs_setup.py, e.g. '--x 1.0 --yaw-deg 180'."),
        DeclareLaunchArgument('odom_pub', default_value='true'),
        DeclareLaunchArgument('scan_tf_pub', default_value='true'),
        DeclareLaunchArgument('speed_topic', default_value='/qbot_speed_feedback'),
        DeclareLaunchArgument("run_dir", default_value=""),
        DeclareLaunchArgument("lidar_dim", default_value="360"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("goal_pose_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("pose_topic", default_value="pose_estimate"),
        DeclareLaunchArgument("obs_topic", default_value="policy/observation"),
        DeclareLaunchArgument("action_name", default_value="policy_navigate"),
        DeclareLaunchArgument("cmd_topic", default_value="cmd_vel"),
        DeclareLaunchArgument("joint_topic", default_value="qbot_joint"),
        DeclareLaunchArgument("imu_topic", default_value="qbot_imu"),
        DeclareLaunchArgument("observation_data_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("planner_data_timeout_sec", default_value="0.2"),
        DeclareLaunchArgument("scan_match_reset_service", default_value="/reset_scan_match"),
        DeclareLaunchArgument("reset_scan_on_terminal", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("qbot_rl_deploy"),
                "rviz",
                "set_goal.rviz",
            ]),
        ),
        DeclareLaunchArgument("client_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument("client_success_radius_m", default_value="0.25"),
        DeclareLaunchArgument(
            "rviz_wait_sec",
            default_value="300.0",
            description="Seconds to wait for the first pose before RViz starts. The launch shuts down "
            "if none arrives. Covers QLabs start-up, the arena setup and the driver, lidar and scan "
            "match start.",
        ),
    ]

    run_dir = LaunchConfiguration("run_dir")
    lidar_dim = LaunchConfiguration("lidar_dim")
    scan_topic = LaunchConfiguration("scan_topic")
    goal_pose_topic = LaunchConfiguration("goal_pose_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    obs_topic = LaunchConfiguration("obs_topic")
    joint_topic = LaunchConfiguration("joint_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    speed_topic = LaunchConfiguration("speed_topic")
    action_name = LaunchConfiguration("action_name")

    # navigate_action_server, navigate_action_client and RViz
    cmd_topic = LaunchConfiguration("cmd_topic")
    observation_data_timeout_sec = LaunchConfiguration("observation_data_timeout_sec")
    planner_data_timeout_sec = LaunchConfiguration("planner_data_timeout_sec")
    scan_match_reset_service = LaunchConfiguration("scan_match_reset_service")
    reset_scan_on_terminal = LaunchConfiguration("reset_scan_on_terminal")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    client_timeout_sec = LaunchConfiguration("client_timeout_sec")
    client_success_radius_m = LaunchConfiguration("client_success_radius_m")

    # Driver, lidar and fixed lidar frame (plus the QLabs spawn step for platform:=virtual)
    # live in qbot_hardware so both platforms share one bringup.
    platform_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("qbot_hardware"),
                "launch",
                "qbot_platform_mapless_nav_bringup_launch.py",
            ])
        ),
        launch_arguments={
            "platform": LaunchConfiguration("platform"),
            "agent_type": LaunchConfiguration("agent_type"),
            "driver_model": LaunchConfiguration("driver_model"),
            "qlabs_setup": LaunchConfiguration("qlabs_setup"),
            "qlabs_launch": LaunchConfiguration("qlabs_launch"),
            "qlabs_module": LaunchConfiguration("qlabs_module"),
            "qlabs_exe": LaunchConfiguration("qlabs_exe"),
            "qlabs_host": LaunchConfiguration("qlabs_host"),
            "qlabs_setup_args": LaunchConfiguration("qlabs_setup_args"),
            # scan_match_node is started by the bringup once the driver and lidar are publishing.
            "localization": "true",
            "odom_pub": LaunchConfiguration("odom_pub"),
            "scan_tf_pub": LaunchConfiguration("scan_tf_pub"),
            "speed_topic": speed_topic,
            "imu_topic": imu_topic,
            "scan_topic": scan_topic,
        }.items(),
    )

    observation_builder_node = Node(
        package="qbot_rl_deploy",
        executable="observation_builder",
        name="observation_builder",
        parameters=[
            {
                "run_dir": run_dir,
                "scan_topic": scan_topic,
                "goal_pose_topic": goal_pose_topic,
                "pose_topic": pose_topic,
                "joint_topic": joint_topic,
                "imu_topic": imu_topic,
                "obs_topic": obs_topic,
                "data_timeout_sec": observation_data_timeout_sec,
            }
        ],
    )

    navigate_server_node = Node(
        package="qbot_rl_deploy",
        executable="navigate_action_server",
        name="navigate_action_server",
        parameters=[
            {
                "run_dir": run_dir,
                "obs_topic": obs_topic,
                "cmd_topic": cmd_topic,
                "data_timeout_sec": planner_data_timeout_sec,
                "scan_match_reset_service": scan_match_reset_service,
                "reset_scan_on_terminal": reset_scan_on_terminal,
            }
        ],
    )

    navigate_client_node = Node(
        package="qbot_rl_deploy",
        executable="navigate_action_client",
        name="navigate_action_client",
        parameters=[
            {
                "goal_pose_topic": goal_pose_topic,
                "action_name": action_name,
                "timeout_sec": client_timeout_sec,
                "success_radius_m": client_success_radius_m,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        declare_args + [
            platform_bringup,
            observation_builder_node,
            navigate_server_node,
            navigate_client_node,
            wait_for_localization([rviz_node]),
        ]
    )
