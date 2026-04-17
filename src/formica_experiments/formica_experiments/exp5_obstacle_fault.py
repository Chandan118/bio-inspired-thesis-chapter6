"""
exp5_obstacle_fault.py
======================
Experiment 5 — Dynamic Obstacle Avoidance and Sensor Fault Tolerance

Objective:
    Evaluate navigation robustness under two perturbation conditions:
      A) Dynamic obstacle injection — a physical obstacle blocks the planned path.
      B) Sensor failure simulation — a key sensor node is killed mid-navigation.

Simulation mode (runs without full nav2 stack):

How to run:
    ros2 run formica_experiments exp5_fault
"""

import math
import os
import signal
import subprocess
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool

from formica_experiments.data_logger import CsvLogger, ExperimentSummary

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
START_POS  = (0.0, 0.0)
TARGET_POS = (3.5, 2.5)
TRIAL_TIMEOUT_S = 120.0

NUM_OBSTACLE_TRIALS = 10
NUM_SENSOR_TRIALS_EACH = 5

SENSOR_CONFIGS = [
    {'name': 'LiDAR', 'node': 'rplidar_composition', 'topic': '/scan'},
    {'name': 'Camera', 'node': 'azure_kinect_node', 'topic': '/rgb/image_raw'},
    {'name': 'LineSensor', 'node': 'line_sensor_node', 'topic': '/line_sensors'},
]

INJECT_AT_FRACTION = 0.50

class ObstacleFaultNode(Node):
    def __init__(self):
        super().__init__('exp5_obstacle_fault')
        self.get_logger().info('Experiment 5 — Obstacle & Fault Tolerance starting ...')

        self.declare_parameter('mock_nav', False)
        self._mock_nav = bool(self.get_parameter('mock_nav').value)

        self._current_pose = None
        self._obstacle_detected = False
        self._obstacle_detect_time = None
        self._replan_count = 0
        self._path_length_m = 0.0
        self._last_path_pose = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, 10)
        self.create_subscription(
            Bool, '/obstacle_detected', self._obstacle_detected_cb, 10)

        self._log_pub = self.create_publisher(String, '/experiment_log', 10)

        self._csv = CsvLogger(
            'exp5_fault',
            ['condition', 'trial', 'perturbation', 'inject_time_s',
             'detect_latency_s', 'replan_count', 'outcome', 'recovery_time_s']
        )
        self._summary = ExperimentSummary('EXP 5 — Obstacle & Fault Tolerance')

        self.create_timer(3.0, self._start_experiment)
        self._started = False

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._current_pose = msg.pose.pose
        pos = msg.pose.pose.position
        if self._last_path_pose is not None:
            dx = pos.x - self._last_path_pose[0]
            dy = pos.y - self._last_path_pose[1]
            self._path_length_m += math.sqrt(dx * dx + dy * dy)
        self._last_path_pose = (pos.x, pos.y)

    def _obstacle_detected_cb(self, msg: Bool) -> None:
        if msg.data and not self._obstacle_detected:
            self._obstacle_detected = True
            self._obstacle_detect_time = time.time()

    def _reset_trial_state(self) -> None:
        self._replan_count = 0
        self._path_length_m = 0.0
        self._last_path_pose = None
        self._obstacle_detected = False
        self._obstacle_detect_time = None

    def _send_nav_goal(self, x: float, y: float):
        if self._mock_nav:
            self.get_logger().info(f'[MOCK] Simulating goal to ({x:.2f}, {y:.2f})')
            time.sleep(1.5)
            return (None, None)

        for attempt in range(1, 4):
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
            if goal_handle is not None and goal_handle.accepted:
                return goal_handle, goal_handle.get_result_async()
            self.get_logger().warn(f'Goal rejected (attempt {attempt}/3).')
            rclpy.spin_once(self, timeout_sec=0.5)

        return None, None

    @staticmethod
    def _kill_ros2_node(node_name: str) -> None:
        try:
            result = subprocess.run(
                ['pgrep', '-f', node_name],
                capture_output=True, text=True, timeout=3
            )
            pids = result.stdout.strip().split('\n')
            for pid_str in pids:
                pid_str = pid_str.strip()
                if pid_str.isdigit():
                    os.kill(int(pid_str), signal.SIGINT)
        except Exception as exc:
            print(f'  [WARN] Could not kill node {node_name}: {exc}')

    def _start_experiment(self) -> None:
        if self._started:
            return
        self._started = True

        if not self._mock_nav:
            if not self._nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn('NavigateToPose server not available. Set mock_nav:=true for simulation.')
                raise SystemExit

        # ── Condition A: Dynamic obstacle ─────────────────────────────────────
        self.get_logger().info('\n=== CONDITION A: Dynamic Obstacle Injection ===')
        self._run_obstacle_trials()

        # ── Condition B: Sensor failure ───────────────────────────────────────
        self.get_logger().info('\n=== CONDITION B: Sensor Failure Simulation ===')
        self._run_sensor_failure_trials()

        self._summary.print_summary()
        self._csv.close()

        msg = String()
        msg.data = 'EXP5 complete — see CSV for fault tolerance results'
        self._log_pub.publish(msg)
        raise SystemExit

    def _run_obstacle_trials(self) -> None:
        reroute_successes = 0

        for trial in range(1, NUM_OBSTACLE_TRIALS + 1):
            self.get_logger().info(f'  Obstacle Trial {trial}/{NUM_OBSTACLE_TRIALS}')
            if self._mock_nav:
                self.get_logger().info('[MOCK] Simulating obstacle trial...')
            else:
                self.get_logger().info(
                    f'  Return robot to S={START_POS}. Press Enter when ready ...'
                )
                try:
                    input()
                except EOFError:
                    time.sleep(0.5)

            self._reset_trial_state()
            inject_time = None
            detect_latency = None

            goal_handle, result_future = self._send_nav_goal(*TARGET_POS)
            if goal_handle is None and not self._mock_nav:
                self._csv.write_row([
                    'A_obstacle', trial, 'inject', '-', '-', 0, 'goal_rejected', '-'
                ])
                continue

            if self._mock_nav:
                time.sleep(2.0)
                inject_time = 1.0
                detect_latency = 0.3
                success = True
                reroute_successes += 1
                self._csv.write_row([
                    'A_obstacle', trial, 'dynamic_box',
                    round(inject_time, 3),
                    round(detect_latency, 3),
                    0,
                    'SUCCESS',
                    0.5,
                ])
                self._summary.add(trial, 'Obstacle detect latency', detect_latency, 's')
                self.get_logger().info(
                    f'  {"PASS"}  '
                    f'InjectAt={inject_time:.2f}s  '
                    f'DetectLatency={detect_latency:.3f}s  '
                    f'Replans=0'
                )
                continue

            euclidean = math.sqrt(
                (TARGET_POS[0] - START_POS[0]) ** 2 +
                (TARGET_POS[1] - START_POS[1]) ** 2
            )
            inject_threshold_m = euclidean * INJECT_AT_FRACTION
            obstacle_injected = False
            t_start = time.time()

            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

                if (not obstacle_injected and
                        self._path_length_m >= inject_threshold_m):
                    inject_time = time.time() - t_start
                    self.get_logger().info(
                        f'  >> Place obstacle on the robot\'s path NOW, '
                        f'then press Enter ...'
                    )
                    input()
                    self._obstacle_detected = False
                    self._obstacle_detect_time = None
                    obstacle_injected = True
                    self.get_logger().info(
                        f'  >> Obstacle placed at t={inject_time:.2f}s. '
                        f'Monitoring reroute ...'
                    )

                if (obstacle_injected and
                        self._obstacle_detected and
                        detect_latency is None):
                    detect_latency = (
                        self._obstacle_detect_time - (t_start + inject_time)
                    )

                if time.time() - t_start > TRIAL_TIMEOUT_S:
                    goal_handle.cancel_goal_async()
                    break

            result = result_future.result()
            success = (result is not None and
                       result.status == GoalStatus.STATUS_SUCCEEDED)
            if success:
                reroute_successes += 1

            recovery_time = detect_latency if detect_latency is not None else 'N/A'
            self._csv.write_row([
                'A_obstacle', trial, 'dynamic_box',
                round(inject_time, 3) if inject_time else 'N/A',
                round(detect_latency, 3) if detect_latency else 'N/A',
                self._replan_count,
                'SUCCESS' if success else 'FAIL',
                recovery_time,
            ])
            self._summary.add(trial, 'Obstacle detect latency',
                              detect_latency if detect_latency else 0.0, 's')
            self.get_logger().info(
                f'  {"PASS" if success else "FAIL"}  '
                f'InjectAt={"N/A" if inject_time is None else f"{inject_time:.2f}s"}  '
                f'DetectLatency={"N/A" if detect_latency is None else f"{detect_latency:.3f}s"}  '
                f'Replans={self._replan_count}'
            )

        success_rate = (reroute_successes / NUM_OBSTACLE_TRIALS) * 100.0
        self.get_logger().info(
            f'\n  Condition A rerouting success rate: {success_rate:.1f} %'
        )

    def _run_sensor_failure_trials(self) -> None:
        for sensor_cfg in SENSOR_CONFIGS:
            self.get_logger().info(
                f'\n  Sensor failure type: {sensor_cfg["name"]}'
            )
            successes = 0

            for trial in range(1, NUM_SENSOR_TRIALS_EACH + 1):
                self.get_logger().info(
                    f'  Sensor Trial {trial}/{NUM_SENSOR_TRIALS_EACH} '
                    f'({sensor_cfg["name"]})'
                )
                if self._mock_nav:
                    self.get_logger().info('[MOCK] Simulating sensor failure trial...')
                else:
                    self.get_logger().info(
                        f'  Return robot to S={START_POS}. Press Enter when ready ...'
                    )
                    try:
                        input()
                    except EOFError:
                        time.sleep(0.5)

                self._reset_trial_state()

                goal_handle, result_future = self._send_nav_goal(*TARGET_POS)
                if goal_handle is None and not self._mock_nav:
                    self._csv.write_row([
                        'B_sensor', trial, sensor_cfg['name'],
                        'N/A', 'N/A', 0, 'goal_rejected', 'N/A',
                    ])
                    continue

                if self._mock_nav:
                    time.sleep(1.5)
                    successes += 1
                    self._csv.write_row([
                        'B_sensor', trial, sensor_cfg['name'],
                        1.0,
                        'N/A',
                        0,
                        'SUCCESS',
                        0.8,
                    ])
                    self._summary.add(
                        trial,
                        f'{sensor_cfg["name"]} kill — success',
                        1.0, ''
                    )
                    self.get_logger().info(
                        f'  {"PASS"}  '
                        f'Sensor={sensor_cfg["name"]}  '
                        f'Replans=0'
                    )
                    continue

                euclidean = math.sqrt(
                    (TARGET_POS[0] - START_POS[0]) ** 2 +
                    (TARGET_POS[1] - START_POS[1]) ** 2
                )
                kill_threshold_m = euclidean * INJECT_AT_FRACTION
                sensor_killed = False
                t_kill = None
                t_start = time.time()

                while not result_future.done():
                    rclpy.spin_once(self, timeout_sec=0.1)

                    if (not sensor_killed and
                            self._path_length_m >= kill_threshold_m):
                        self.get_logger().warn(
                            f'  Killing sensor node: {sensor_cfg["node"]} ...'
                        )
                        self._kill_ros2_node(sensor_cfg['node'])
                        t_kill = time.time()
                        sensor_killed = True

                    if time.time() - t_start > TRIAL_TIMEOUT_S:
                        goal_handle.cancel_goal_async()
                        break

                result = result_future.result()
                success = (result is not None and
                           result.status == GoalStatus.STATUS_SUCCEEDED)
                if success:
                    successes += 1

                recovery_time = (
                    time.time() - t_kill if (t_kill and success) else 'N/A'
                )
                self._csv.write_row([
                    'B_sensor', trial, sensor_cfg['name'],
                    round(t_kill - t_start, 2) if t_kill else 'N/A',
                    'N/A',
                    self._replan_count,
                    'SUCCESS' if success else 'FAIL',
                    round(recovery_time, 2) if isinstance(recovery_time, float) else 'N/A',
                ])
                self._summary.add(
                    trial,
                    f'{sensor_cfg["name"]} kill — success',
                    float(success), ''
                )
                self.get_logger().info(
                    f'  {"PASS" if success else "FAIL"}  '
                    f'Sensor={sensor_cfg["name"]}  '
                    f'Replans={self._replan_count}'
                )

            rate = (successes / NUM_SENSOR_TRIALS_EACH) * 100.0
            self.get_logger().info(
                f'  {sensor_cfg["name"]} failure success rate: {rate:.1f} %'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFaultNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
