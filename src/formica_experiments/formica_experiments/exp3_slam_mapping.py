"""
exp3_slam_mapping.py
====================
Experiment 3 — SLAM-Based Autonomous Mapping in an Unknown Environment

Objective:
    Validate that the FormicaBot constructs a globally consistent 2D occupancy
    grid map using RPLIDAR A1 + slam_toolbox, achieving localisation RMSE ≤ 0.15 m
    at four known ArUco landmark positions.

Simulation mode (runs without full nav2 stack):

Landmarks:
    Edit LANDMARK_POSITIONS_M below to match the four ArUco marker ground-truth
    coordinates (in the map frame) that you physically measured in the arena.

How to run:
    ros2 run formica_experiments exp3_slam

Expected outputs:
    - Console RMSE per trial and overall
    - ~/formica_experiments/data/exp3_slam_<timestamp>.csv
    - Saved map files via `ros2 run nav2_map_server map_saver_cli`
"""

import math
import time
import random
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

from formica_experiments.data_logger import CsvLogger, ExperimentSummary

# ---------------------------------------------------------------------------
# Configuration — edit to match your physical arena
# Full thesis protocol: 10 trials.
# ---------------------------------------------------------------------------
NUM_TRIALS = 10

# Ground-truth landmark positions in the map frame (x, y) in metres
# Measure these physically with a tape measure before running the experiment.
LANDMARK_POSITIONS_M = [
    (1.00, 0.00),   # Landmark 1 (ArUco ID 0)
    (1.00, 2.00),   # Landmark 2 (ArUco ID 1)
    (0.00, 2.00),   # Landmark 3 (ArUco ID 2)
    (2.00, 2.00),   # Landmark 4 (ArUco ID 3)
]

NAV_GOAL_TOLERANCE_M = 0.20     # nav2 goal tolerance
LOCALIZATION_RMSE_TARGET_M = 0.15
# Thesis protocol: ≥ 95 % known cells before landmark verification.
MAP_COVERAGE_TARGET_PCT = 95.0
MAP_COVERAGE_TIMEOUT_S = 900.0

# Automatic cmd_vel mapping behavior (used when Nav2 goal interface is unavailable).
FORCE_CMDVEL_AUTONOMY = os.environ.get('EXP3_FORCE_CMDVEL_AUTONOMY', '0') == '1'
AUTO_START_TRIALS = os.environ.get('EXP3_AUTO_START', '1') == '1'
CMDVEL_GOAL_TOLERANCE_M = 0.18
CMDVEL_MAX_LINEAR_MPS = 0.072
CMDVEL_MAX_ANGULAR_RADPS = 0.45
CMDVEL_CONTROL_DT_S = 0.1
CMDVEL_MAX_LINEAR_ACCEL = 0.20
CMDVEL_MAX_ANGULAR_ACCEL = 0.80
OBSTACLE_STOP_M = 0.35
OBSTACLE_SLOW_M = 0.60
SCAN_STALE_TIMEOUT_S = 3.5
SCAN_STABLE_MSG_MIN = 4
SCAN_WAIT_TIMEOUT_S = 20.0
MAPPING_SWEEP_SEGMENTS = [
    (CMDVEL_MAX_LINEAR_MPS, 0.0, 7.5),
    (0.00, 0.35, 2.6),
    (CMDVEL_MAX_LINEAR_MPS, 0.0, 6.8),
    (0.00, -0.35, 2.6),
    (CMDVEL_MAX_LINEAR_MPS, 0.0, 7.5),
    (0.00, 0.35, 2.6),
    (CMDVEL_MAX_LINEAR_MPS, 0.0, 6.8),
]


class SlamMappingNode(Node):
    def __init__(self):
        super().__init__('exp3_slam_mapping')
        self.get_logger().info('Experiment 3 — SLAM Mapping starting ...')

        self.declare_parameter('mock_map', False)
        self._mock_map = bool(self.get_parameter('mock_map').value)
        self._mock_coverage_pct = 0.0

        self._current_pose = None
        self._current_odom_pose = None
        self._map_total_cells = 0
        self._map_known_cells = 0
        self._last_scan_wall_time = None
        self._scan_msg_count = 0
        self._front_min_m = float('inf')
        self._avoid_turn_sign = 1.0

        self._use_cmdvel_autonomy = FORCE_CMDVEL_AUTONOMY or self._mock_map
        self._sensor_cb_group = ReentrantCallbackGroup()

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10,
            callback_group=self._sensor_cb_group)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
            callback_group=self._sensor_cb_group)
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 10,
            callback_group=self._sensor_cb_group)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10,
            callback_group=self._sensor_cb_group)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data,
            callback_group=self._sensor_cb_group)

        self._log_pub = self.create_publisher(String, '/experiment_log', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._csv = CsvLogger(
            'exp3_slam',
            ['trial', 'landmark_id', 'gt_x', 'gt_y',
             'est_x', 'est_y', 'error_m', 'coverage_pct']
        )
        self._summary = ExperimentSummary('EXP 3 — SLAM Mapping')

        self.create_timer(3.0, self._start_experiment, callback_group=self._sensor_cb_group)
        self._started = False
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _yaw_from_pose(self, pose) -> float:
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        max_lin_step = CMDVEL_MAX_LINEAR_ACCEL * CMDVEL_CONTROL_DT_S
        max_ang_step = CMDVEL_MAX_ANGULAR_ACCEL * CMDVEL_CONTROL_DT_S
        target_lin = float(linear_x)
        target_ang = float(angular_z)
        delta_lin = max(-max_lin_step, min(max_lin_step, target_lin - self._last_cmd_linear))
        delta_ang = max(-max_ang_step, min(max_ang_step, target_ang - self._last_cmd_angular))
        self._last_cmd_linear += delta_lin
        self._last_cmd_angular += delta_ang
        self._last_cmd_linear = max(
            -CMDVEL_MAX_LINEAR_MPS,
            min(CMDVEL_MAX_LINEAR_MPS, self._last_cmd_linear),
        )
        msg = Twist()
        msg.linear.x = self._last_cmd_linear
        msg.angular.z = self._last_cmd_angular
        self._cmd_pub.publish(msg)

    def _stop_robot(self) -> None:
        self._publish_twist(0.0, 0.0)

    def _current_localization_pose(self):
        return self._current_pose if self._current_pose is not None else self._current_odom_pose

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._current_pose = msg.pose.pose

    def _odom_cb(self, msg: Odometry) -> None:
        self._current_odom_pose = msg.pose.pose

    def _map_cb(self, msg: OccupancyGrid) -> None:
        if self._mock_map:
            return
        self._map_total_cells = msg.info.width * msg.info.height
        self._map_known_cells = sum(
            1 for v in msg.data if v >= 0
        )

    def _get_coverage_pct(self) -> float:
        if self._mock_map:
            return self._mock_coverage_pct
        if self._map_total_cells > 0:
            return (self._map_known_cells / self._map_total_cells) * 100.0
        return 0.0

    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_scan_wall_time = time.time()
        self._scan_msg_count += 1
        if not msg.ranges or msg.angle_increment == 0.0:
            return
        vals = []
        for deg in range(-20, 21):
            a = math.radians(deg)
            idx = int(round((a - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    vals.append(float(r))
        if vals:
            self._front_min_m = min(vals)

    def _scan_healthy(self) -> bool:
        if self._last_scan_wall_time is None:
            return False
        return (time.time() - self._last_scan_wall_time) <= SCAN_STALE_TIMEOUT_S

    def _wait_for_scan_stable(self, timeout_s: float = SCAN_WAIT_TIMEOUT_S) -> bool:
        start = time.time()
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._scan_healthy():
                return True
        return False

    def _run_mapping_sweep(self, max_duration_s: float) -> None:
        start_t = time.time()
        for linear, angular, duration in MAPPING_SWEEP_SEGMENTS:
            seg_start = time.time()
            while time.time() - seg_start < duration:
                if time.time() - start_t > max_duration_s:
                    self._stop_robot()
                    return
                if self._scan_healthy() and self._front_min_m <= OBSTACLE_STOP_M:
                    self._stop_robot()
                    self._publish_twist(0.0, 0.55 * self._avoid_turn_sign)
                    rclpy.spin_once(self, timeout_sec=0.4)
                    self._avoid_turn_sign *= -1.0
                    continue
                self._publish_twist(linear, angular)
                rclpy.spin_once(self, timeout_sec=CMDVEL_CONTROL_DT_S)
                if self._mock_map:
                    self._mock_coverage_pct = min(100.0, self._mock_coverage_pct + 0.5)
                coverage_pct = self._get_coverage_pct()
                if coverage_pct >= MAP_COVERAGE_TARGET_PCT:
                    self._stop_robot()
                    return
            self._stop_robot()
            time.sleep(0.15)

    def _navigate_to_cmdvel(self, x: float, y: float, timeout_s: float = 90.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=CMDVEL_CONTROL_DT_S)
            if not self._scan_healthy():
                self._stop_robot()
                if not self._wait_for_scan_stable(timeout_s=20.0):
                    continue
            pose = self._current_localization_pose()
            if pose is None:
                self._stop_robot()
                continue

            px = pose.position.x
            py = pose.position.y
            yaw = self._yaw_from_pose(pose)
            dx = x - px
            dy = y - py
            dist = math.sqrt(dx * dx + dy * dy)
            if dist <= CMDVEL_GOAL_TOLERANCE_M:
                self._stop_robot()
                return True

            desired_heading = math.atan2(dy, dx)
            heading_err = self._normalize_angle(desired_heading - yaw)

            ang = max(-CMDVEL_MAX_ANGULAR_RADPS, min(CMDVEL_MAX_ANGULAR_RADPS, 1.2 * heading_err))
            if self._scan_healthy() and self._front_min_m <= OBSTACLE_STOP_M:
                lin = 0.0
                ang = 0.55 * self._avoid_turn_sign
                self._avoid_turn_sign *= -1.0
            elif abs(heading_err) > 0.4:
                lin = 0.0
            else:
                lin = min(CMDVEL_MAX_LINEAR_MPS, 0.35 * dist)
                if self._scan_healthy() and self._front_min_m <= OBSTACLE_SLOW_M:
                    lin = min(lin, 0.03)
            self._publish_twist(lin, ang)

        self._stop_robot()
        return False

    def _start_experiment(self) -> None:
        if self._started:
            return
        self._started = True
        self._stop_robot()

        nav2_available = self._nav_client.wait_for_server(timeout_sec=10.0)
        if not nav2_available:
            self._use_cmdvel_autonomy = True
            self.get_logger().warn(
                'NavigateToPose action server not available. '
                'Switching to /cmd_vel autonomous mapping mode.'
            )
        elif self._use_cmdvel_autonomy:
            self.get_logger().info(
                'EXP3_FORCE_CMDVEL_AUTONOMY=1: using /cmd_vel autonomy mode.'
            )

        all_errors = []

        for trial in range(1, NUM_TRIALS + 1):
            if not self._wait_for_scan_stable():
                self.get_logger().error(
                    'LiDAR /scan is not stable. Stopping Experiment 3.'
                )
                self._stop_robot()
                raise SystemExit

            self.get_logger().info(f'\n--- Trial {trial}/{NUM_TRIALS} ---')
            if AUTO_START_TRIALS:
                time.sleep(2.0)
            else:
                self.get_logger().info(
                    'Reset robot to (0,0) and reset map if required, '
                    'then press ENTER to start trial.'
                )
                try:
                    input()
                except EOFError:
                    time.sleep(3.0)

            coverage_start = time.time()
            last_cov_log_t = 0.0
            while True:
                if not self._scan_healthy():
                    self.get_logger().warn('/scan temporarily lost; pausing movement.')
                    self._stop_robot()
                    if not self._wait_for_scan_stable(timeout_s=20.0):
                        self.get_logger().warn('LiDAR recovery delayed; continuing.')

                if self._mock_map:
                    self._mock_coverage_pct = min(100.0, self._mock_coverage_pct + 0.3)

                coverage_pct = self._get_coverage_pct()
                now_t = time.time()
                if now_t - last_cov_log_t >= 15.0:
                    last_cov_log_t = now_t
                    self.get_logger().info(
                        f'  Mapping … coverage {coverage_pct:.1f}% '
                        f'(target {MAP_COVERAGE_TARGET_PCT:.1f}%), '
                        f'elapsed {now_t - coverage_start:.0f}s / {MAP_COVERAGE_TIMEOUT_S:.0f}s'
                    )
                if coverage_pct >= MAP_COVERAGE_TARGET_PCT:
                    self.get_logger().info(
                        f'  Coverage reached {coverage_pct:.1f}% '
                        f'(target {MAP_COVERAGE_TARGET_PCT:.1f}%).'
                    )
                    break
                if time.time() - coverage_start > MAP_COVERAGE_TIMEOUT_S:
                    self.get_logger().warn(
                        f'  Coverage timeout at {coverage_pct:.1f}%. Continuing trial.'
                    )
                    break
                if self._use_cmdvel_autonomy:
                    self._run_mapping_sweep(max_duration_s=6.0)
                else:
                    rclpy.spin_once(self, timeout_sec=0.2)

            trial_errors = []
            for lm_id, (gt_x, gt_y) in enumerate(LANDMARK_POSITIONS_M):
                self.get_logger().info(
                    f'  Navigating to landmark {lm_id}: ({gt_x:.2f}, {gt_y:.2f}) ...'
                )
                if self._use_cmdvel_autonomy:
                    success = self._navigate_to_cmdvel(gt_x, gt_y)
                else:
                    success = self._navigate_to(gt_x, gt_y)
                    if not success:
                        self.get_logger().warn(
                            '  Nav2 goal failed. Falling back to /cmd_vel autonomy mode.'
                        )
                        self._use_cmdvel_autonomy = True
                        success = self._navigate_to_cmdvel(gt_x, gt_y)

                if not success:
                    self.get_logger().warn(f'  Navigation FAILED to landmark {lm_id}.')
                    continue

                time.sleep(0.5)
                rclpy.spin_once(self, timeout_sec=1.0)

                pose = self._current_pose if self._current_pose is not None else self._current_odom_pose
                if pose is None:
                    self.get_logger().warn('No localization pose received.')
                    continue

                est_x = pose.position.x
                est_y = pose.position.y
                error = math.sqrt((est_x - gt_x) ** 2 + (est_y - gt_y) ** 2)
                trial_errors.append(error)
                all_errors.append(error)

                coverage_pct = self._get_coverage_pct()

                self._csv.write_row([
                    trial, lm_id, gt_x, gt_y,
                    round(est_x, 4), round(est_y, 4),
                    round(error, 4), round(coverage_pct, 2)
                ])
                self._summary.add(trial, f'LM{lm_id} error', error, 'm')
                self.get_logger().info(
                    f'  GT=({gt_x:.3f},{gt_y:.3f})  '
                    f'Est=({est_x:.3f},{est_y:.3f})  '
                    f'Error={error:.4f} m  Coverage={coverage_pct:.1f} %'
                )

            if trial_errors:
                trial_rmse = math.sqrt(
                    sum(e ** 2 for e in trial_errors) / len(trial_errors)
                )
                self.get_logger().info(f'  Trial {trial} RMSE = {trial_rmse:.4f} m')

        overall_rmse = 0.0
        if all_errors:
            overall_rmse = math.sqrt(sum(e * e for e in all_errors) / len(all_errors))
            passed = overall_rmse <= LOCALIZATION_RMSE_TARGET_M
            self._csv.write_row(['ALL', 'RMSE', '-', '-', '-', '-',
                                 round(overall_rmse, 4), '-'])
            self._summary.add(0, 'Overall RMSE', overall_rmse, 'm')
            self.get_logger().info(
                f'\n  Overall RMSE = {overall_rmse:.4f} m  '
                f'(target ≤ {LOCALIZATION_RMSE_TARGET_M} m)  '
                f'{"PASS" if passed else "FAIL"}'
            )

        self._summary.print_summary()
        self._csv.close()
        self._stop_robot()

        msg = String()
        msg.data = f'EXP3 complete — RMSE={overall_rmse:.4f}m  {"PASS" if passed else "FAIL"}'
        self._log_pub.publish(msg)
        raise SystemExit

    def _navigate_to(self, x: float, y: float, timeout_s: float = 60.0) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2.')
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.time() + timeout_s
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() > deadline:
                self.get_logger().warn('Navigation timeout.')
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        from action_msgs.msg import GoalStatus
        return result.status == GoalStatus.STATUS_SUCCEEDED


def main(args=None):
    rclpy.init(args=args)
    node = SlamMappingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
