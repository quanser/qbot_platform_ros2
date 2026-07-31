import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def exit_qbot_platform_driver_interface_cb(context):
    subprocess.run(
        "quarc_run -q -t tcpip://localhost:17000 qbot_platform_driver_physical",
        shell=True,
        capture_output=True,
    )

def exit_ekf_scan_match_cb(context):
    subprocess.run(
        "quarc_run -q -t tcpip://localhost:17000 ekf_scan_match",
        shell=True,
        capture_output=True,
    )

def generate_launch_description():

    # Declare launch arguments with default values
    declare_args = [
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

    # Policy planner
    cmd_topic = LaunchConfiguration("cmd_topic")
    observation_data_timeout_sec = LaunchConfiguration("observation_data_timeout_sec")
    planner_data_timeout_sec = LaunchConfiguration("planner_data_timeout_sec")
    scan_match_reset_service = LaunchConfiguration("scan_match_reset_service")
    reset_scan_on_terminal = LaunchConfiguration("reset_scan_on_terminal")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    client_timeout_sec = LaunchConfiguration("client_timeout_sec")
    client_success_radius_m = LaunchConfiguration("client_success_radius_m")

    driver_model_rt_executable = PathJoinSubstitution(
        [
            FindPackageShare("qbot_hardware"),
            "rt_models",
            "qbot_platform_driver_physical.rt-linux_qbot_platform",
        ]
    )

    rt_model_start = ExecuteProcess(
        cmd=[
            "quarc_run",
            "-r -t tcpip://localhost:17000",
            driver_model_rt_executable,
            "-d %d -uri tcpip://%m:17001",
        ],
        name="QBotPlatformDriverModelStart",
        shell=True,
    )

    ekf_scan_match_rt_executable = PathJoinSubstitution(
        [
            FindPackageShare("qbot_localization"),
            "rt_models",
            "ekf_scan_match.rt-linux_qbot_platform",
        ]
    )
 
    ekf_rt_model_start = ExecuteProcess(
        cmd=[
            "quarc_run",
            "-r -t tcpip://localhost:17000",
            ekf_scan_match_rt_executable,
            "-d %d -uri tcpip://%m:17002",
        ],
        name="EkfScanMatchModelStart",
        shell=True,
    )

    qbot_driver_node = Node(
        package="qbot_hardware",
        executable="qbot_platform_driver_interface",
        name="qbot_platform_driver",
        parameters=[{"arm_robot": True,
                     "speed_limit_mode": 0,
                     "body_speed_duration": 0.1}],
    )

    lidar_node = Node(
        package="qbot_hardware",
        executable="lidar",
        name="lidar_node",
    )

    fixed_frame = Node(
            package='qbot_hardware',
            executable='fixed_lidar_frame',
            name='fixed_lidar_frame',
        )

    localization_node = Node(
            package='qbot_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            parameters=[{
                'odom_pub': LaunchConfiguration('odom_pub'),
                'tf_pub': LaunchConfiguration('scan_tf_pub'),
                'speed_topic': speed_topic,
                'imu_topic': imu_topic,
                'scan_topic': scan_topic,
                }]
            )

    observation_node = Node(
        package="qbot_rl_deploy",
        executable="observation_builder",
        name="observation_builder",
        parameters=[
            {
                "run_dir": run_dir,
                "lidar_dim": lidar_dim,
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

    policy_server = Node(
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

    policy_client = Node(
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

    start_after_rt_model = RegisterEventHandler(
        OnProcessStart(
            target_action=rt_model_start,
            on_start=[
                LogInfo(msg="RT model started. Waiting 2 seconds before starting policy stack nodes."),
                TimerAction(
                    period=2.0,
                    actions=[qbot_driver_node, 
                             lidar_node, 
                             fixed_frame, 
                             ekf_rt_model_start,
                             localization_node, 
                             observation_node, 
                             policy_server,
                             policy_client,
                             rviz_node],
                ),
            ],
        )
    )

    stop_rt_model_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=qbot_driver_node,
            on_exit=[
                OpaqueFunction(function=exit_qbot_platform_driver_interface_cb),
                LogInfo(msg="Driver node exited; stopping QUARC RT model."),
            ],
        )
    )

    stop_ekf_model_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_node,
            on_exit=[
                OpaqueFunction(function=exit_ekf_scan_match_cb),
                LogInfo(msg="localization_node exited; stopping ekf_scan_match RT model."),
            ],
        )
    )
    
    return LaunchDescription(
        declare_args + [
            rt_model_start,
            start_after_rt_model,
            stop_rt_model_on_exit,
            stop_ekf_model_on_exit,
        ]
    )
