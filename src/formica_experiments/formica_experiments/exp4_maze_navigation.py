"""
exp4_maze_navigation.py
=======================
Experiment 4 — Maze Navigation and Path Efficiency (≥89 % success target)

Objective:
    Validate that the FormicaBot achieves a navigation success rate ≥ 89 %
    across 20 trials navigating from start position S to target position T
    through a physical maze, using the pre-built SLAM map and nav2.

Simulation mode (runs without full nav2 stack):

How to run:
    bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials
    ros2 run formica_experiments exp4_maze                     # if stack already up
"""

import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32

from formica_experiments.data_logger import CsvLogger, ExperimentSummary

# ---------------------------------------------------------------------------
# Configuration — edit to match your maze layout
# ---------------------------------------------------------------------------
NUM_TRIALS = int(os.environ.get("EXP4_NUM_TRIALS", "20"))
TRIAL_TIMEOUT_S = float(os.environ.get("EXP4_TRIAL_TIMEOUT_S", "420"))

START_POS  = (0.0, 0.0)
TARGET_POS = (2.525, 3.035)

MAX_PATH_LENGTH_FACTOR = 1.5
EFFICIENCY_TARGET_PCT  = 89.0
SPEED_SCALE = 0.50
AMCL_JUMP_REJECT_M = 0.55
OBSTACLE_STOP_M = 0.52
ULTRA_STOP_CM = 48.0
OBSTACLE_CRITICAL_M = 0.28
OBSTACLE_ABORT_GRACE_S = 2.5
OBSTACLE_ABORT_MIN_LINEAR_M_S = 0.035
LIDAR_FRONT_HALF_DEG = 58
LIDAR_SPEED_LIMIT_HALF_DEG = 32


EUCLIDEAN_DIST = math.sqrt(
    (TARGET_POS[0] - START_POS[0]) ** 2 +
    (TARGET_POS[1] - START_POS[1]) ** 2
)
MAX_ALLOWED_PATH_M = EUCLIDEAN_DIST * MAX_PATH_LENGTH_FACTOR


class MazeNavigationNode(Node):
    def __init__(self):
        super().__init__('exp4_maze_navigation')
        self.get_logger().info('Experiment 4 — Maze Navigation starting ...')

        self.declare_parameter('mock_nav', False)
        self._mock_nav = bool(self.get_parameter('mock_nav').value)

        self.get_logger().info(
            f'  S={START_POS}  T={TARGET_POS}  '
            f'Euclidean={EUCLIDEAN_DIST:.2f} m  '
            f'MaxPath={MAX_ALLOWED_PATH_M:.2f} m'
        )

        self._current_pose = None
        self._replan_count = 0
        self._path_length_m = 0.0
        self._last_path_pose = None

        self._front_min_m = float("inf")
        self._speed_front_min_m = float("inf")
        self._ultra_cm = float("inf")
        self._odom_linear_x = 0.0
        self._speed_cap_ema = None

        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb_group)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Path, '/plan', self._plan_cb, 10, callback_group=self._cb_group)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 20, callback_group=self._cb_group)
        self.create_subscription(
            Float32, '/sensor/distance', self._ultra_cb, 20,
            callback_group=self._cb_group)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 30,
            callback_group=self._cb_group)

        self._log_pub = self.create_publisher(String, '/experiment_log', 10)
        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)

        self._csv = CsvLogger(
            'exp4_maze',
            ['trial', 'outcome', 'path_length_m', 'time_to_target_s',
             'replan_events', 'failure_mode', 'efficiency_pct']
        )
        self._summary = ExperimentSummary('EXP 4 — Maze Navigation')

        self.create_timer(3.0, self._start_experiment, callback_group=self._cb_group)
        self._started = False
        self._executor_shutdown = None

    def _request_executor_stop(self) -> None:
        fn = self._executor_shutdown
        if fn is not None:
            threading.Timer(0.15, fn).start()
        else:
            raise SystemExit

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._current_pose = msg.pose.pose
        pos = msg.pose.pose.position
        if self._last_path_pose is not None:
            dx = pos.x - self._last_path_pose[0]
            dy = pos.y - self._last_path_pose[1]
            seg = math.sqrt(dx * dx + dy * dy)
            if seg <= AMCL_JUMP_REJECT_M:
                self._path_length_m += seg
        self._last_path_pose = (pos.x, pos.y)

    def _plan_cb(self, _: Path) -> None:
        self._replan_count += 1

    def _scan_cb(self, msg: LaserScan) -> None:
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        def min_in_half_deg(half_deg: int) -> float:
            vals = []
            for deg in range(-half_deg, half_deg + 1):
                a = math.radians(deg)
                idx = int(round((a - msg.angle_min) / msg.angle_increment))
                if 0 <= idx < len(msg.ranges):
                    r = msg.ranges[idx]
                    if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                        vals.append(float(r))
            return min(vals) if vals else float("inf")

        self._front_min_m = min_in_half_deg(LIDAR_FRONT_HALF_DEG)
        self._speed_front_min_m = min_in_half_deg(LIDAR_SPEED_LIMIT_HALF_DEG)

    def _ultra_cb(self, msg: Float32) -> None:
        d = float(msg.data)
        self._ultra_cm = float('inf') if d <= 0.0 else d

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_linear_x = float(msg.twist.twist.linear.x)

    def _publish_speed_limit(self) -> None:
        lim = SpeedLimit()
        lim.percentage = False
        raw = 0.20 * SPEED_SCALE
        sf = self._speed_front_min_m
        if math.isfinite(sf) and sf < 1.2:
            raw = min(raw, max(0.045, 0.35 * sf))
        if self._speed_cap_ema is None:
            self._speed_cap_ema = raw
        else:
            self._speed_cap_ema = 0.22 * raw + 0.78 * self._speed_cap_ema
        lim.speed_limit = float(self._speed_cap_ema)
        self._speed_pub.publish(lim)

    def _scan_healthy(self) -> bool:
        return math.isfinite(self._front_min_m) and self._front_min_m < 1.0e6

    def _translating_for_abort(self) -> bool:
        return abs(self._odom_linear_x) > OBSTACLE_ABORT_MIN_LINEAR_M_S

    def _obstacle_too_close(self) -> bool:
        if not self._translating_for_abort():
            return False
        scan_block = math.isfinite(self._front_min_m) and self._front_min_m <= OBSTACLE_STOP_M
        ultra_block = math.isfinite(self._ultra_cm) and self._ultra_cm <= ULTRA_STOP_CM
        return scan_block or ultra_block

    def _obstacle_critical(self) -> bool:
        if not self._translating_for_abort():
            return False
        scan_crit = math.isfinite(self._front_min_m) and self._front_min_m <= OBSTACLE_CRITICAL_M
        ultra_crit = math.isfinite(self._ultra_cm) and self._ultra_cm <= 22.0
        return scan_crit or ultra_crit

    def _start_experiment(self) -> None:
        if self._started:
            return
        self._started = True

        if not self._mock_nav:
            if not self._nav_client.wait_for_server(timeout_sec=30.0):
                self.get_logger().warn(
                    'NavigateToPose server not available. '
                    'Set mock_nav:=true to run in simulation mode.'
                )
                self._request_executor_stop()
                return
            self.get_logger().info('Running with real Nav2 stack.')
        else:
            self.get_logger().info('[MOCK MODE] Simulating navigation results.')

        successes = 0

        for trial in range(1, NUM_TRIALS + 1):
            self.get_logger().info(
                f'\n--- Trial {trial}/{NUM_TRIALS} ---'
            )
            self.get_logger().info(
                f'Return robot to S={START_POS} and verify AMCL localisation. '
                f'Auto-starting trial in 3 seconds ...'
            )
            time.sleep(3.0)

            self._replan_count  = 0
            self._path_length_m = 0.0
            self._last_path_pose = None

            t_start = time.time()
            self._publish_speed_limit()
            success, failure_mode = self._run_trial()
            elapsed = time.time() - t_start

            path_ok = self._path_length_m <= MAX_ALLOWED_PATH_M
            time_ok = elapsed <= TRIAL_TIMEOUT_S
            trial_passed = success and path_ok and time_ok

            if trial_passed:
                successes += 1
                failure_mode = 'none'
            elif success and not path_ok:
                failure_mode = 'path_too_long'
            elif success and not time_ok:
                failure_mode = 'timeout'

            efficiency_running = (successes / trial) * 100.0

            self._csv.write_row([
                trial,
                'SUCCESS' if trial_passed else 'FAIL',
                round(self._path_length_m, 4),
                round(elapsed, 2),
                self._replan_count,
                failure_mode,
                round(efficiency_running, 2),
            ])
            self._summary.add(trial, 'path_length', self._path_length_m, 'm')
            self._summary.add(trial, 'time_to_target', elapsed, 's')
            self._summary.add(trial, 'replan_events', float(self._replan_count), '')

            self.get_logger().info(
                f'  {"PASS" if trial_passed else "FAIL"}  '
                f'Path={self._path_length_m:.2f} m  '
                f'Time={elapsed:.1f} s  '
                f'Replans={self._replan_count}  '
                f'Failure={failure_mode}  '
                f'Running efficiency={efficiency_running:.1f} %'
            )
            sys.stdout.flush()
            sys.stderr.flush()

        final_efficiency = (successes / NUM_TRIALS) * 100.0
        passed = final_efficiency >= EFFICIENCY_TARGET_PCT

        self._csv.write_row([
            'FINAL', '-', '-', '-', '-', '-', round(final_efficiency, 2)
        ])
        self._summary.add(0, 'Final efficiency', final_efficiency, '%')

        print('\n' + '=' * 60)
        print(f'  Experiment 4 — Final Navigation Efficiency')
        print('=' * 60)
        print(f'  Trials: {NUM_TRIALS}  Successes: {successes}')
        print(f'  Efficiency: {final_efficiency:.1f} %  '
              f'(target ≥ {EFFICIENCY_TARGET_PCT} %)  '
              f'{"PASS" if passed else "FAIL"}')
        print('=' * 60 + '\n')

        self._summary.print_summary()
        self._csv.close()

        msg = String()
        msg.data = f'EXP4 complete — efficiency={final_efficiency:.1f}%  {"PASS" if passed else "FAIL"}'
        self._log_pub.publish(msg)
        self._request_executor_stop()

    def _run_trial(self) -> tuple:
        if self._mock_nav:
            self.get_logger().info('[MOCK] Simulating navigation to target...')
            time.sleep(2.0)
            self._path_length_m = EUCLIDEAN_DIST * 1.05
            return True, 'none'

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = TARGET_POS[0]
        goal_msg.pose.pose.position.y = TARGET_POS[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self._speed_cap_ema = None

        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, 'goal_rejected'

        result_future = goal_handle.get_result_async()
        deadline = time.time() + TRIAL_TIMEOUT_S
        obstacle_check_after = time.time() + OBSTACLE_ABORT_GRACE_S
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._publish_speed_limit()
            if time.time() >= obstacle_check_after:
                if self._obstacle_critical() or self._obstacle_too_close():
                    goal_handle.cancel_goal_async()
                    return False, 'obstacle_abort'
            if time.time() > deadline:
                goal_handle.cancel_goal_async()
                return False, 'timeout'

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'none'
        elif result.status == GoalStatus.STATUS_CANCELED:
            return False, 'canceled'
        else:
            return False, f'nav2_status_{result.status}'


def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    node._executor_shutdown = executor.shutdown
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
