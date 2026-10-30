#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan

from qbot_rl_interfaces.msg import ObservationStamped


class ObservationBuilder(Node):
    def __init__(self) -> None:
        super().__init__("observation_builder")

        run_dir_str = str(self.declare_parameter("run_dir", "").value)
        if run_dir_str == "":
            raise RuntimeError("Parameter 'run_dir' is required for observation_builder.")
        self.run_dir = Path(run_dir_str).expanduser().resolve()

        env_cfg = self._load_env_config(self.run_dir)

        self.obs_dim = int(self._require_key(env_cfg, ["obs_dim"]))
        self.lidar_dim = self.obs_dim - 5  # lidar + goal_dist + goal_heading + wl + wr + yaw_rate

        self.lidar_max_range = float(self._require_key(env_cfg, ["scene", "lidar_sensor", "max_distance"]))
        self.wheel_speed_obs_max = float(self._require_key(env_cfg, ["wheel_speed_obs_max"]))
        self.gyro_yaw_rate_obs_max = float(self._require_key(env_cfg, ["gyro_yaw_rate_obs_max"]))

        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.goal_pose_topic = str(self.declare_parameter("goal_pose_topic", "/goal_pose").value)
        self.pose_topic = str(self.declare_parameter("pose_topic", "pose_estimate").value)
        self.joint_topic = str(self.declare_parameter("joint_topic", "qbot_joint").value)
        self.imu_topic = str(self.declare_parameter("imu_topic", "qbot_imu").value)
        self.obs_topic = str(self.declare_parameter("obs_topic", "policy/observation").value)
        # Rate of the fallback watchdog timer only. Normal publishing is triggered by
        # pose_callback (see subscription setup below).
        self.publish_rate_hz = 2.0
        self._last_publish_time: Optional[float] = None
        self._min_publish_gap_sec = 0.05  # below the ~100 ms pose period, so pose-triggered publishes are never skipped
        self.data_timeout_sec = float(self.declare_parameter("data_timeout_sec", 0.1).value)

        # Hardcoded transform from fixed_lidar_frame: base_link <- base_scan.
        self.scan_to_base_tx = float(self.declare_parameter("scan_to_base_tx", 0.15).value)
        self.scan_to_base_ty = float(self.declare_parameter("scan_to_base_ty", 0.0).value)
        self.scan_to_base_yaw = float(self.declare_parameter("scan_to_base_yaw", np.pi /2).value)

        self._component_names = ["lidar", "goal_dist", "goal_heading", "wheel_speed", "gyro"]
        self._component_start_idx = [0, self.lidar_dim, self.lidar_dim + 1, self.lidar_dim + 2, self.lidar_dim + 4]
        self._component_lengths = [self.lidar_dim, 1, 1, 2, 1]

        self._latest_lidar_norm: Optional[np.ndarray] = None
        self._latest_goal_xy: Optional[np.ndarray] = None  # [x, y] in odom
        self._latest_pose_xyyaw: Optional[np.ndarray] = None  # [x, y, yaw] in odom
        self._latest_joint_vel: Optional[np.ndarray] = None  # [left, right]
        self._latest_yaw_rate: Optional[float] = None

        self._last_lidar_time: Optional[float] = None
        self._last_goal_pose_time: Optional[float] = None
        self._last_pose_time: Optional[float] = None
        self._last_joint_time: Optional[float] = None
        self._last_imu_time: Optional[float] = None

        self._last_lidar_stamp: Optional[TimeMsg] = None
        self._last_pose_stamp: Optional[TimeMsg] = None
        self._last_joint_stamp: Optional[TimeMsg] = None
        self._last_imu_stamp: Optional[TimeMsg] = None

        self.obs_pub = self.create_publisher(ObservationStamped, self.obs_topic, 1)

        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)  # matches /scan publisher
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, scan_qos)
        self.create_subscription(PoseStamped, self.goal_pose_topic, self.goal_pose_callback, 1)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 1)
        self.create_subscription(JointState, self.joint_topic, self.joint_callback, 2)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 2)
        # Observation publish event-driven from pose_callback, since
        # scan-match (/pose_estimate, ~10Hz) is the slowest required input.
        # This watchdog timer is a low-rate fallback to catch cases where pose
        # updates stop arriving (e.g. scan-match stalls)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1e-3), self._publish_observation_if_ready)

        self.get_logger().info(
            "observation_builder started. "
            f"run_dir={self.run_dir}, obs_dim={self.obs_dim}, lidar_dim={self.lidar_dim}, "
            f"scan_topic={self.scan_topic}, goal_pose_topic={self.goal_pose_topic}, pose_topic={self.pose_topic}, "
            f"joint_topic={self.joint_topic}, imu_topic={self.imu_topic}, obs_topic={self.obs_topic}, "
            f"watchdog_rate_hz={self.publish_rate_hz:.1f}"
        )

    def _load_env_config(self, run_dir: Path) -> Dict[str, Any]:
        env_path = run_dir / "params" / "env.yaml"
        if not env_path.exists():
            raise RuntimeError(f"Missing training config: {env_path}")

        try:
            with env_path.open("r", encoding="utf-8") as f:
                data = yaml.full_load(f)
        except Exception as exc:
            raise RuntimeError(f"Failed to parse {env_path}: {exc}") from exc

        if not isinstance(data, dict):
            raise RuntimeError(f"Unexpected YAML structure in {env_path}; expected a mapping.")
        return data

    def _require_key(self, root: Dict[str, Any], path: List[str]) -> Any:
        cur: Any = root
        for key in path:
            if not isinstance(cur, dict) or key not in cur:
                joined = ".".join(path)
                raise RuntimeError(f"Missing required key '{joined}' in env.yaml")
            cur = cur[key]
        return cur

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _stamp_to_sec(stamp: TimeMsg) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _copy_stamp(stamp: TimeMsg) -> TimeMsg:
        copied = TimeMsg()
        copied.sec = int(stamp.sec)
        copied.nanosec = int(stamp.nanosec)
        return copied

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self._latest_goal_xy = np.array(
            [float(msg.pose.position.x), float(msg.pose.position.y)],
            dtype=np.float32,
        )
        self._last_goal_pose_time = self._stamp_to_sec(msg.header.stamp)

    def pose_callback(self, msg: PoseStamped) -> None:
        self._latest_pose_xyyaw = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(self._yaw_from_quaternion(msg.pose.orientation)),
            ],
            dtype=np.float32,
        )
        self._last_pose_time = self._stamp_to_sec(msg.header.stamp)
        self._last_pose_stamp = self._copy_stamp(msg.header.stamp)
        # The scan-match pose is the slowest required input and is only available
        # after the matcher finishes, so publish as soon as it arrives.
        self._publish_observation_if_ready()

    def joint_callback(self, msg: JointState) -> None:
        if len(msg.velocity) < 2:
            return
        # JointState ordering is [right, left]; convert to [left, right].
        self._latest_joint_vel = np.array([float(msg.velocity[1]), float(msg.velocity[0])], dtype=np.float32)
        self._last_joint_time = self._stamp_to_sec(msg.header.stamp)
        self._last_joint_stamp = self._copy_stamp(msg.header.stamp)

    def imu_callback(self, msg: Imu) -> None:
        self._latest_yaw_rate = float(msg.angular_velocity.z)
        self._last_imu_time = self._stamp_to_sec(msg.header.stamp)
        self._last_imu_stamp = self._copy_stamp(msg.header.stamp)

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = self._scan_to_interpolated_ranges(msg)
        self._latest_lidar_norm = np.clip(ranges / self.lidar_max_range, 0.0, 1.0)
        self._last_lidar_time = self._stamp_to_sec(msg.header.stamp)
        self._last_lidar_stamp = self._copy_stamp(msg.header.stamp)

    def _scan_to_interpolated_ranges(self, msg: LaserScan) -> np.ndarray:
        ranges_in = np.asarray(msg.ranges, dtype=np.float64)
        if ranges_in.size == 0:
            return np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float64)

        raw_angles = msg.angle_min + np.arange(ranges_in.size, dtype=np.float64) * msg.angle_increment
        valid = np.isfinite(ranges_in) & (ranges_in > 0.0)
        if float(msg.range_min) > 0.0:
            valid &= ranges_in >= float(msg.range_min)
        if float(msg.range_max) > 0.0:
            valid &= ranges_in <= float(msg.range_max)

        if not np.any(valid):
            return np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float64)

        tx = self.scan_to_base_tx
        ty = self.scan_to_base_ty
        yaw = self.scan_to_base_yaw
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        xs = ranges_in[valid] * np.cos(raw_angles[valid])
        ys = ranges_in[valid] * np.sin(raw_angles[valid])
        xb = cy * xs - sy * ys + tx
        yb = sy * xs + cy * ys + ty

        rb = np.hypot(xb, yb)
        ab = np.arctan2(yb, xb)

        finite = np.isfinite(rb) & np.isfinite(ab)
        rb = rb[finite]
        ab = ab[finite]

        out = np.full(self.lidar_dim, self.lidar_max_range, dtype=np.float64)
        if rb.size == 0:
            return out

        # direct bin index from angle, wrapped into [0, lidar_dim)
        bin_width = 2.0 * math.pi / self.lidar_dim
        idx = np.floor((ab + math.pi) / bin_width).astype(np.int64) % self.lidar_dim

        rb_clipped = np.clip(rb, 0.0, self.lidar_max_range)

        # scatter-min: each bin takes the closest hit that landed in it
        np.minimum.at(out, idx, rb_clipped)

        return out

    def _is_fresh(self, last_time: Optional[float], now: float) -> bool:
        return last_time is not None and (now - last_time) <= self.data_timeout_sec

    def _publish_observation_if_ready(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            # Keeps the watchdog timer from publishing right after a pose-triggered publish.
            self._last_publish_time is not None
            and (now - self._last_publish_time) < self._min_publish_gap_sec
        ):
            return
        
        lidar_fresh = self._is_fresh(self._last_lidar_time, now)
        pose_fresh = self._is_fresh(self._last_pose_time, now)
        joint_fresh = self._is_fresh(self._last_joint_time, now)
        imu_fresh = self._is_fresh(self._last_imu_time, now)

        # Goal is expected to be latched/static; only require that it exists,
        # not that its timestamp stays within data_timeout_sec.
        if not (lidar_fresh and pose_fresh and joint_fresh and imu_fresh):
            self.get_logger().debug(
                f'not publishing observation: stale data: '
                f'lidar_fresh={lidar_fresh}, pose_fresh={pose_fresh}, joint_fresh={joint_fresh}, imu_fresh={imu_fresh}',
                throttle_duration_sec=10.0,
            )
            return

        if (
            self._latest_lidar_norm is None
            or self._latest_goal_xy is None
            or self._latest_pose_xyyaw is None
            or self._latest_joint_vel is None
            or self._latest_yaw_rate is None
            or self._last_lidar_stamp is None
            or self._last_pose_stamp is None
            or self._last_joint_stamp is None
            or self._last_imu_stamp is None
        ):
            self.get_logger().debug(
                f"not publishing observation: missing data: "
                f"lidar_norm={self._latest_lidar_norm is not None}, "
                f"goal_xy={self._latest_goal_xy is not None}, "
                f"pose_xyyaw={self._latest_pose_xyyaw is not None}, "
                f"joint_vel={self._latest_joint_vel is not None}, "
                f"yaw_rate={self._latest_yaw_rate is not None}",
                throttle_duration_sec=10.0,
            )
            return

        dx_w = float(self._latest_goal_xy[0] - self._latest_pose_xyyaw[0])
        dy_w = float(self._latest_goal_xy[1] - self._latest_pose_xyyaw[1])
        goal_dist = math.hypot(dx_w, dy_w)

        yaw = float(self._latest_pose_xyyaw[2])
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        dx_b = cos_y * dx_w + sin_y * dy_w
        dy_b = sin_y * dx_w - cos_y * dy_w
        goal_heading = math.atan2(dy_b, dx_b)

        goal_dist_norm = float(np.clip(goal_dist / self.lidar_max_range, 0.0, 1.0))
        goal_heading_norm = float(np.clip(goal_heading / math.pi, -1.0, 1.0))

        wl_norm = float(np.clip(self._latest_joint_vel[0] / self.wheel_speed_obs_max, -1.0, 1.0))
        wr_norm = float(np.clip(self._latest_joint_vel[1] / self.wheel_speed_obs_max, -1.0, 1.0))
        yaw_rate_norm = float(np.clip(self._latest_yaw_rate / self.gyro_yaw_rate_obs_max, -1.0, 1.0))

        state = np.array(
            [goal_dist_norm, goal_heading_norm, wl_norm, wr_norm, yaw_rate_norm],
            dtype=np.float32,
        )
        # Layout must match policy expectation: [lidar..., goal_dist, goal_heading, wl, wr, yaw_rate]
        obs = np.concatenate([self._latest_lidar_norm.astype(np.float32), state], axis=0)

        if obs.size != self.obs_dim:
            self.get_logger().error(f"Built observation size {obs.size}, expected {self.obs_dim}.")
            return

        msg = ObservationStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.observation = obs.tolist()
        msg.component_names = list(self._component_names)
        msg.component_start_idx = list(self._component_start_idx)
        msg.component_lengths = list(self._component_lengths)
        # Goal distance and heading use scan-match pose source timestamp by design.
        msg.component_stamps = [
            self._copy_stamp(self._last_lidar_stamp),
            self._copy_stamp(self._last_pose_stamp),
            self._copy_stamp(self._last_pose_stamp),
            self._copy_stamp(self._last_joint_stamp),
            self._copy_stamp(self._last_imu_stamp),
        ]

        if not (
            len(msg.component_names)
            == len(msg.component_start_idx)
            == len(msg.component_lengths)
            == len(msg.component_stamps)
        ):
            self.get_logger().error("ObservationStamped metadata arrays have mismatched lengths.")
            return
        self.obs_pub.publish(msg)
        self._last_publish_time = now



def main() -> None:
    rclpy.init()
    node = ObservationBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()