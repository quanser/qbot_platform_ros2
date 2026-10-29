#!/usr/bin/env python3

import time
import threading
import csv
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from qbot_rl_interfaces.action import PolicyNavigate
from qbot_rl_interfaces.msg import ObservationStamped

class NavigateActionServer(Node):
    """Action-driven policy node that runs a TorchScript policy from run_dir/exported/policy.pt."""

    def __init__(self) -> None:
        super().__init__("navigate_action_server")

        run_dir_str = str(self.declare_parameter("run_dir", "").value)
        if run_dir_str == "":
            raise RuntimeError("Parameter 'run_dir' is required for navigate_action_server.")
        self.run_dir = Path(run_dir_str).expanduser().resolve()

        env_cfg = self._load_env_config(self.run_dir)
        self.obs_dim = int(self._require_key(env_cfg, ["obs_dim"]))
        self.max_lin_vel = float(self._require_key(env_cfg, ["max_lin_vel"]))
        self.max_ang_vel = float(self._require_key(env_cfg, ["max_ang_vel"]))
        self.lidar_max_range = float(self._require_key(env_cfg, ["scene", "lidar_sensor", "max_distance"]))
        self.obs_topic = str(self.declare_parameter("obs_topic", "policy/observation").value)
        self.cmd_topic = str(self.declare_parameter("cmd_topic", "cmd_vel").value)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.feedback_rate_hz = float(self.declare_parameter("feedback_rate_hz", 10.0).value)
        sim_dt = float(self._require_key(env_cfg, ["sim", "dt"]))
        decimation = float(self._require_key(env_cfg, ["decimation"]))
        self.tick_rate_hz = float(1.0 / (sim_dt * decimation))
        self.data_timeout_sec = float(self.declare_parameter("data_timeout_sec", 0.2).value)
        self.action_no_data_abort_sec = float(
            self.declare_parameter("action_no_data_abort_sec", 2.0).value
        )
        self.scan_match_reset_service = str(
            self.declare_parameter("scan_match_reset_service", "/reset_scan_match").value
        )
        self.scan_match_reset_wait_sec = float(
            self.declare_parameter("scan_match_reset_wait_sec", 2.0).value
        )
        self.reset_scan_on_terminal = bool(
            self.declare_parameter("reset_scan_on_terminal", True).value
        )
        self.publish_zero_on_missing_data = bool(
            self.declare_parameter("publish_zero_on_missing_data", True).value
        )
        self.declare_parameter("timing_log_csv", os.path.expanduser("~/policy_planner_timing_log.csv"))
        self.timing_log_path = self.get_parameter("timing_log_csv").get_parameter_value().string_value
        self._init_timing_log()

        model_path = self.run_dir / "exported" / "policy.pt"
        if not model_path.exists():
            raise RuntimeError(f"Missing TorchScript model: {model_path}")

        try:
            self.policy = torch.jit.load(str(model_path), map_location=self.device)
            self.policy.eval()
        except Exception as exc:
            raise RuntimeError(f"Failed to load TorchScript model from {model_path}: {exc}") from exc

        try:
            warmup_t0 = time.perf_counter()
            dummy_obs = torch.zeros((1, self.obs_dim), dtype=torch.float32, device=self.device)
            with torch.no_grad():
                self.policy(dummy_obs)
            warmup_ms = (time.perf_counter() - warmup_t0) * 1000.0
            self.get_logger().info(f"Policy warmup inference complete: {warmup_ms:.2f} ms")
        except Exception as exc:
            self.get_logger().warn(f"Policy warmup inference failed (continuing anyway): {exc}")

        self._obs_state: Optional[Tuple[np.ndarray, float]] = None
        self._mission_lock = threading.Lock()
        self._active_goal_handle = None
        self._mission_active = False
        self._mission_start_time: Optional[float] = None
        self._mission_accept_time: Optional[float] = None
        self._mission_timeout_sec = 0.0
        self._mission_success_radius_m = 0.25
        self._mission_done_event = threading.Event()
        self._mission_result: Optional[PolicyNavigate.Result] = None
        self._tick_timer = None
        self._obs_log_once = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_match_reset_cli = self.create_client(Trigger, self.scan_match_reset_service)
        self.create_subscription(ObservationStamped, self.obs_topic, self._obs_cb, 1)

        self.action_server = ActionServer(
            self,
            PolicyNavigate,
            "policy_navigate",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info(
            "policy_planner started. "
            f"run_dir={self.run_dir}, device={self.device}, "
            f"tick_rate_hz={self.tick_rate_hz:.2f},"
            f"reset_scan_on_terminal={self.reset_scan_on_terminal}, "
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
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _obs_cb(self, msg: ObservationStamped) -> None:
        if not (
            len(msg.component_names)
            == len(msg.component_start_idx)
            == len(msg.component_lengths)
            == len(msg.component_stamps)
        ):
            self.get_logger().warn("Dropping observation: ObservationStamped metadata array lengths mismatch.")
            return

        data = np.asarray(msg.observation, dtype=np.float32)
        if data.size != self.obs_dim:
            self.get_logger().warn(f"Received obs size {data.size}, expected {self.obs_dim}.")
            return

        if not np.all(np.isfinite(data)):
            self.get_logger().warn("Received observation with NaN/Inf; dropping frame.")
            return

        self._obs_state = (data, self._stamp_to_sec(msg.header.stamp))

        if not self._obs_log_once:
            self._obs_log_once = True
            self.get_logger().info(
                "ObservationStamped stream active: "
                f"obs_len={data.size}, components={len(msg.component_names)}"
            )

    def _publish_stop(self) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def _goal_cb(self, goal_request: PolicyNavigate.Goal) -> GoalResponse:
        if goal_request.success_radius_m <= 0.0:
            self.get_logger().warn("Rejecting goal: success_radius_m must be > 0.")
            return GoalResponse.REJECT
        if goal_request.timeout_sec <= 0.0:
            self.get_logger().warn("Rejecting goal: timeout_sec must be > 0.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel request accepted for current mission.")
        return CancelResponse.ACCEPT

    def _current_goal_metrics(self) -> Optional[Tuple[float, float]]:
        state = self._obs_state
        if state is None:
            return None
        data, obs_time = state
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - obs_time) > self.data_timeout_sec or data.size < 5:
            return None

        goal_dist_norm = float(data[-5])
        goal_heading_norm = float(data[-4])
        goal_dist = goal_dist_norm * self.lidar_max_range
        goal_heading = goal_heading_norm * np.pi
        return goal_dist, goal_heading

    def _finish_mission(self, goal_handle) -> None:
        with self._mission_lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None
                self._mission_active = False
                self._mission_start_time = None
                self._mission_accept_time = None
                self._publish_stop()

    def _start_tick_timer(self) -> None:
        with self._mission_lock:
            if self._tick_timer is not None:
                return
            period = 1.0 / max(self.tick_rate_hz, 1e-3)
            self._tick_timer = self.create_timer(period, self._mission_tick)

    def _stop_tick_timer(self) -> None:
        with self._mission_lock:
            timer = self._tick_timer
            self._tick_timer = None
        if timer is not None:
            timer.cancel()
            timer.destroy()

    def _set_terminal_result(self, status: str, final_distance: float = float("nan")) -> None:
        goal_handle = None
        with self._mission_lock:
            if self._mission_result is not None:
                return
            goal_handle = self._active_goal_handle
            if goal_handle is None:
                return

            result = PolicyNavigate.Result()
            result.final_distance = float(final_distance)

            if status == "success":
                if goal_handle.is_active:
                    goal_handle.succeed()
                result.succeeded = True
                result.message = "Reached goal"
            elif status == "timeout":
                if goal_handle.is_active:
                    goal_handle.abort()
                result.succeeded = False
                result.message = "Timed out"
            elif status == "canceled":
                if goal_handle.is_active:
                    goal_handle.canceled()
                result.succeeded = False
                result.message = "Canceled"
            elif status == "preempted":
                if goal_handle.is_active:
                    goal_handle.abort()
                result.succeeded = False
                result.message = "Preempted by newer goal"
            else:
                if goal_handle.is_active:
                    goal_handle.abort()
                result.succeeded = False
                result.message = "Failed"

            if self._mission_result is None:
                self._mission_result = result
                self._mission_done_event.set()

    def _mission_tick(self) -> None:
        with self._mission_lock:
            if not self._mission_active or self._active_goal_handle is None:
                return
            goal_handle = self._active_goal_handle
            mission_start_time = self._mission_start_time
            mission_timeout_sec = self._mission_timeout_sec
            mission_success_radius_m = self._mission_success_radius_m

        if goal_handle.is_cancel_requested:
            self._set_terminal_result("canceled")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = 0.0 if mission_start_time is None else now - mission_start_time

        if mission_timeout_sec > 0.0 and elapsed >= mission_timeout_sec:
            metrics = self._current_goal_metrics()
            final_distance = float("nan") if metrics is None else float(metrics[0])
            self._set_terminal_result("timeout", final_distance=final_distance)
            return

        state = self._obs_state
        if state is None or (now - state[1]) > self.data_timeout_sec:
            if self.publish_zero_on_missing_data:
                self._publish_stop()
            return
        obs_array, obs_stamp = state
        obs_age_sec = now - obs_stamp

        obs_t = torch.tensor(obs_array, dtype=torch.float32, device=self.device).unsqueeze(0)

        t0 = time.perf_counter()
        with torch.no_grad():
            output = self.policy(obs_t)
        inference_ms = (time.perf_counter() - t0) * 1000.0

        obs_age_ms = obs_age_sec * 1000.0
        self.get_logger().debug(
        self.get_logger().info(
            f"obs age: {obs_age_ms:.2f} ms | inference: {inference_ms:.2f} ms | "
            f"total: {obs_age_ms + inference_ms:.2f} ms",
            throttle_duration_sec=1.0,
        )

        action = output.detach().cpu().numpy().reshape(-1)

        cmd = Twist()
        # Policy outputs are normalized in [-1, 1]; scale by training-time limits.
        norm_linear = float(np.clip(action[0], -1.0, 1.0))
        norm_angular = float(np.clip(action[1], -1.0, 1.0))
        cmd.linear.x = norm_linear * self.max_lin_vel
        cmd.angular.z = norm_angular * self.max_ang_vel
        self.cmd_pub.publish(cmd)

        metrics = self._current_goal_metrics()
        if metrics is not None and float(metrics[0]) <= mission_success_radius_m:
            self._set_terminal_result("success", final_distance=float(metrics[0]))
            return

        feedback = PolicyNavigate.Feedback()
        feedback.running_time_sec = float(elapsed)
        feedback.distance_to_goal = float("nan") if metrics is None else float(metrics[0])
        feedback.heading_error = float("nan") if metrics is None else float(metrics[1])
        goal_handle.publish_feedback(feedback)

    def _wait_for_fresh_observation_after_accept(self, accept_time: float, timeout_sec: float) -> bool:
        deadline = accept_time + timeout_sec if timeout_sec > 0.0 else float("inf")
        while rclpy.ok():
            now = self.get_clock().now().nanoseconds * 1e-9
            with self._mission_lock:
                if not self._mission_active:
                    return False
                state = self._obs_state
                if state is not None and state[1] > accept_time:
                    return True
            if now >= deadline:
                return False
            time.sleep(0.01)
        return False

    def _call_scan_match_reset_blocking(self) -> None:
        if not self.scan_match_reset_cli.wait_for_service(timeout_sec=self.scan_match_reset_wait_sec):
            self.get_logger().warn(
                f"Scan-match reset service '{self.scan_match_reset_service}' unavailable; skipping reset."
            )
            return

        request = Trigger.Request()
        future = self.scan_match_reset_cli.call_async(request)

        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        if not future.done():
            self.get_logger().warn("Scan-match reset did not complete before shutdown.")
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Scan-match reset call failed: {exc}")
            return

        if response is None:
            self.get_logger().warn("Scan-match reset response is empty.")
            return

        if response.success:
            self.get_logger().info(f"Scan-match reset succeeded: {response.message}")
        else:
            self.get_logger().warn(f"Scan-match reset reported failure: {response.message}")

    def _execute_cb(self, goal_handle) -> PolicyNavigate.Result:
        goal = goal_handle.request
        mission_start_time = self.get_clock().now().nanoseconds * 1e-9
        mission_accept_time = mission_start_time
        mission_timeout_sec = float(goal.timeout_sec)
        mission_success_radius_m = float(goal.success_radius_m)

        with self._mission_lock:
            previous_goal_handle = self._active_goal_handle
            self._active_goal_handle = goal_handle
            self._mission_active = True
            self._mission_start_time = mission_start_time
            self._mission_accept_time = mission_accept_time
            self._mission_timeout_sec = mission_timeout_sec
            self._mission_success_radius_m = mission_success_radius_m
            self._mission_result = None
            self._mission_done_event.clear()

        if previous_goal_handle is not None and previous_goal_handle is not goal_handle:
            self.get_logger().info("Preempting previous active mission with new goal.")

        self.get_logger().info(
            "Accepted policy mission: "
            f"success_radius={mission_success_radius_m:.3f}, "
            f"timeout_sec={mission_timeout_sec:.3f}"
        )

        result = PolicyNavigate.Result()
        skip_reset_for_this_goal = False

        try:
            if goal_handle.is_cancel_requested:
                self._set_terminal_result("canceled")
            else:
                fresh_ok = self._wait_for_fresh_observation_after_accept(
                    mission_accept_time,
                    timeout_sec=max(mission_timeout_sec, self.action_no_data_abort_sec),
                )
                if not fresh_ok:
                    self._set_terminal_result("timeout")
                else:
                    self._start_tick_timer()

            while rclpy.ok():
                with self._mission_lock:
                    is_current_goal = self._active_goal_handle is goal_handle
                if not is_current_goal:
                    skip_reset_for_this_goal = True
                    result.succeeded = False
                    result.final_distance = float("nan")
                    result.message = "Preempted by newer goal"
                    return result
                with self._mission_lock:
                    done = self._mission_done_event.is_set()
                if done:
                    break
                time.sleep(0.01)

            with self._mission_lock:
                if self._mission_result is not None:
                    result = self._mission_result
                else:
                    result.succeeded = False
                    result.final_distance = float("nan")
                    result.message = "Node shutting down"
        finally:
            if not skip_reset_for_this_goal:
                if self.reset_scan_on_terminal:
                    self._call_scan_match_reset_blocking()
                self._stop_tick_timer()
            self._finish_mission(goal_handle)
        return result


def main() -> None:
    rclpy.init()
    node = NavigateActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()