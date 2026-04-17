"""
exp7_pheromone_trail.py
========================
Experiment 7 — Virtual Pheromone Trail (TCRT5000 + MQ-135)

Sub-experiments A–D per thesis protocol.

Simulation mode:

Run:
  ros2 run formica_experiments exp7_pheromone --ros-args -p mock_sensors:=true -p auto_run:=true
"""

from __future__ import annotations

import math
import os
import statistics
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, String

from formica_experiments.data_logger import CsvLogger, ExperimentSummary

STRAIGHT_TRAIL_LEN_M = 3.0
CURVED_TRAIL_LEN_M = 4.0
SNR_THRESHOLD_DB = 6.0
STRAIGHT_LATERAL_TARGET_M = 0.015
CURVED_LATERAL_TARGET_M = 0.020
KP, KI, KD = 0.6, 0.02, 0.10
LINEAR_SPEED_MPS = 0.10
MAX_ANGULAR_RAD_S = 1.0
TRAIL_THRESHOLD_ADC = 1500.0
SWITCHOVER_LOG_PHRASE = 'SNR < 6 dB switching to chemical backup'

class PheromoneTrailNode(Node):
    def __init__(self) -> None:
        super().__init__('exp7_pheromone_trail')

        self.declare_parameter('mock_sensors', False)
        self.declare_parameter('auto_run', False)
        self.declare_parameter('num_straight_trials', 10)
        self.declare_parameter('num_curved_trials', 10)
        self.declare_parameter('num_snr_trials', 10)
        self.declare_parameter('chemical_threshold_adc', 180.0)
        self.declare_parameter('chemical_baseline_adc', 120.0)
        self.declare_parameter('gas_saturated_adc', 950.0)
        self.declare_parameter('start_delay_s', 3.0)

        self._mock = bool(self.get_parameter('mock_sensors').value)
        self._auto = bool(self.get_parameter('auto_run').value)
        self._num_straight = int(self.get_parameter('num_straight_trials').value)
        self._num_curved = int(self.get_parameter('num_curved_trials').value)
        self._num_snr = int(self.get_parameter('num_snr_trials').value)
        self._chem_thr = float(self.get_parameter('chemical_threshold_adc').value)
        self._chem_base = float(self.get_parameter('chemical_baseline_adc').value)
        self._gas_sat = float(self.get_parameter('gas_saturated_adc').value)
        start_delay = float(self.get_parameter('start_delay_s').value)

        self.get_logger().info(
            f'Experiment 7 — mock_sensors={self._mock} auto_run={self._auto} '
        )

        self._line_vals = [0.0, 0.0, 0.0, 0.0]
        self._gas_adc = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom: tuple[float, float] | None = None
        self._dist_travelled_m = 0.0

        self._optical_active = True
        self._chemical_active = False

        self._pid_integral = 0.0
        self._pid_prev_error = 0.0
        self._pid_last_time: float | None = None

        self._ambient_light = 0.0
        self._mock_snr_boost = 0.0
        self._mock_lat_error = 0.0
        self._last_optical_error = 0.0

        if not self._mock:
            self.create_subscription(Float32MultiArray, '/line_sensors', self._line_cb, 10)
            self.create_subscription(Float32, '/gas_sensor', self._gas_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Float32, '/ambient_light', self._ambient_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._log_pub = self.create_publisher(String, '/experiment_log', 10)

        self._csv = CsvLogger(
            'exp7_pheromone',
            [
                'sub_exp', 'trial', 'trail_type', 'dist_m', 'lateral_dev_m',
                'modality', 'snr_db', 'trail_lost_all_four', 'elapsed_s',
                'led_pwm', 's0', 's1', 's2', 's3', 'gas_adc', 'ambient_pct',
                'switchover_latency_s', 'continued_following_or_trail_any',
            ],
        )
        self._summary = ExperimentSummary('EXP 7 — Pheromone Trail')

        self._experiment_done = False
        self.create_timer(start_delay, self._start_experiment)

    def _line_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 4:
            self._line_vals = [float(x) for x in msg.data[:4]]

    def _gas_cb(self, msg: Float32) -> None:
        self._gas_adc = float(msg.data)

    def _ambient_cb(self, msg: Float32) -> None:
        self._ambient_light = max(0.0, min(1.0, float(msg.data)))

    def _odom_cb(self, msg: Odometry) -> None:
        if self._mock:
            return
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self._last_odom is not None:
            dx = pos.x - self._last_odom[0]
            dy = pos.y - self._last_odom[1]
            self._dist_travelled_m += math.hypot(dx, dy)
        self._last_odom = (pos.x, pos.y)
        self._odom_x, self._odom_y = pos.x, pos.y

    def _wait_enter(self, prompt: str) -> None:
        if self._auto:
            self.get_logger().info(f'[auto_run] {prompt}')
            time.sleep(0.3)
            return
        self.get_logger().info(prompt)
        try:
            input()
        except EOFError:
            time.sleep(0.5)

    def _start_experiment(self) -> None:
        if self._experiment_done:
            return
        self._experiment_done = True

        try:
            self.get_logger().info('=== Sub-experiment A: Straight trail ===')
            self._run_trail_trials('straight', STRAIGHT_TRAIL_LEN_M, self._num_straight, STRAIGHT_LATERAL_TARGET_M)

            self.get_logger().info('=== Sub-experiment B: Curved trail ===')
            self._run_trail_trials('curved', CURVED_TRAIL_LEN_M, self._num_curved, CURVED_LATERAL_TARGET_M)

            self.get_logger().info('=== Sub-experiment C: SNR / chemical switchover ===')
            self._run_snr_trials()

            self.get_logger().info('=== Sub-experiment D: Pheromone decay (LED PWM) ===')
            self._run_decay_simulation()

            self._summary.print_summary()
            self._csv.close()
            self._log_pub.publish(String(data='EXP7 complete — CSV + run postprocess for figures'))
            self.get_logger().info('EXP7 complete.')
        finally:
            self._cmd_pub.publish(Twist())
            def _deferred_shutdown() -> None:
                time.sleep(0.1)
                if rclpy.ok():
                    rclpy.shutdown()
            threading.Thread(target=_deferred_shutdown, daemon=True).start()

    def _update_mock_line_for_trial(self, trail_type: str, dist_m: float) -> None:
        err = self._mock_lat_error
        if trail_type == 'curved':
            err += 0.08 * math.sin(dist_m * 1.2)
        base = 2600.0
        self._line_vals = [
            base - 400 * err + 30 * math.sin(time.time() * 5),
            base - 100 * err,
            base + 100 * err,
            base + 400 * err + 30 * math.cos(time.time() * 5),
        ]

    def _run_trail_trials(
        self,
        trail_type: str,
        trail_len_m: float,
        num_trials: int,
        lateral_target_m: float,
    ) -> None:
        all_means: list[float] = []

        for trial in range(1, num_trials + 1):
            self._wait_enter(
                f'Trial {trial}/{num_trials} ({trail_type}): place robot at START, LED full; '
                f'confirm TCRT > {TRAIL_THRESHOLD_ADC:.0f} ADC. Press Enter ...'
            )

            self._reset_pid()
            self._dist_travelled_m = 0.0
            self._last_odom = None
            self._optical_active = True
            self._chemical_active = False
            self._mock_lat_error = 0.015 * (trial % 7) / 7.0
            self._mock_snr_boost = 0.0
            self._last_optical_error = 0.0

            lateral_samples: list[float] = []
            trail_loss_events = 0
            next_log_m = 0.0
            t_start = time.time()

            while self._dist_travelled_m < trail_len_m:
                rclpy.spin_once(self, timeout_sec=0.02)
                if self._mock:
                    self._dist_travelled_m += LINEAR_SPEED_MPS * 0.02
                    self._update_mock_line_for_trial(trail_type, self._dist_travelled_m)

                opt_err = self._compute_lateral_error_optical()
                if not self._mock and any(float(v) >= TRAIL_THRESHOLD_ADC for v in self._line_vals):
                    self._last_optical_error = opt_err

                error = self._lateral_error_for_control()
                lateral_dev_m = abs(error) * 0.02
                lateral_samples.append(lateral_dev_m)

                if self._all_sensors_below_trail():
                    trail_loss_events += 1

                snr_db = self._compute_snr()
                if snr_db < SNR_THRESHOLD_DB and self._optical_active:
                    self._optical_active = False
                    self._chemical_active = True
                    self.get_logger().warn(SWITCHOVER_LOG_PHRASE)
                    self.get_logger().warn(f'  (SNR={snr_db:.2f} dB, optical modality off)')

                cmd = self._pid_control(error)
                self._cmd_pub.publish(cmd)

                while self._dist_travelled_m >= next_log_m - 1e-6 and next_log_m <= trail_len_m + 1e-6:
                    self._csv.write_row([
                        'A_straight' if trail_type == 'straight' else 'B_curved',
                        trial,
                        trail_type,
                        round(min(next_log_m, self._dist_travelled_m), 3),
                        round(lateral_dev_m, 4),
                        'chemical' if self._chemical_active else 'optical',
                        round(snr_db, 2),
                        self._all_sensors_below_trail(),
                        round(time.time() - t_start, 2),
                        '',
                        *[round(v, 1) for v in self._line_vals],
                        round(self._gas_adc, 1),
                        round(self._ambient_light * 100.0, 1),
                        '',
                        '',
                    ])
                    next_log_m += 0.1

                if time.time() - t_start > 180.0:
                    self.get_logger().warn('Trail following timeout (180 s).')
                    break

            self._cmd_pub.publish(Twist())
            elapsed = time.time() - t_start
            mean_dev = statistics.mean(lateral_samples) if lateral_samples else 0.0
            all_means.append(mean_dev)
            passed = mean_dev <= lateral_target_m and trail_loss_events == 0
            self._summary.add(trial, f'{trail_type} mean lateral (m)', mean_dev, 'm')
            self._summary.add(trial, f'{trail_type} trail_loss_count', float(trail_loss_events), '')
            self._summary.add(trial, f'{trail_type} time_s', elapsed, 's')
            self.get_logger().info(
                f'Trial {trial}: mean_lat={mean_dev * 100:.2f} cm  trail_loss={trail_loss_events}  '
                f'time={elapsed:.1f}s  {"PASS" if passed else "FAIL"} '
                f'(target ≤ {lateral_target_m * 100:.1f} cm, no loss)'
            )

        if all_means:
            om = statistics.mean(all_means)
            os_ = statistics.stdev(all_means) if len(all_means) > 1 else 0.0
            self.get_logger().info(f'{trail_type} overall mean lateral: {om * 100:.2f} ± {os_ * 100:.2f} cm')

    def _run_snr_trials(self) -> None:
        for trial in range(1, self._num_snr + 1):
            self._wait_enter(
                f'SNR trial {trial}/{self._num_snr}: robot on trail, ambient LOW, MQ-135 primed; Enter ...'
            )

            self._optical_active = True
            self._chemical_active = False
            t_snr_breach: float | None = None
            t_chem_active: float | None = None
            ambient_at_breach = 0.0
            continued = False
            t0 = time.time()

            while time.time() - t0 < 120.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._mock:
                    self._mock_snr_boost = (time.time() - t0) * 0.12
                    self._gas_adc = self._chem_base + min(400.0, (time.time() - t0) * 35.0)
                    self._update_mock_line_for_trial('straight', 0.0)

                snr_db = self._compute_snr()

                if t_snr_breach is None and snr_db < SNR_THRESHOLD_DB:
                    t_snr_breach = time.time()
                    ambient_at_breach = self._ambient_light
                    self._optical_active = False
                    self.get_logger().warn(SWITCHOVER_LOG_PHRASE)
                    self.get_logger().warn(f'  (SNR={snr_db:.2f} dB)')

                if t_snr_breach is not None and not self._chemical_active:
                    if self._gas_adc >= self._chem_thr:
                        t_chem_active = time.time()
                        self._chemical_active = True
                        latency = t_chem_active - t_snr_breach
                        self.get_logger().info(
                            f'Chemical backup active: latency={latency:.3f} s  gas_adc={self._gas_adc:.1f}'
                        )
                        continued = True
                        break

            latency_s = -1.0
            if t_snr_breach is not None and t_chem_active is not None:
                latency_s = t_chem_active - t_snr_breach
            elif t_snr_breach is None:
                self.get_logger().warn('SNR never dropped below threshold in this trial.')

            self._csv.write_row([
                'C_snr',
                trial,
                'switchover',
                '',
                '',
                'chemical' if self._chemical_active else 'optical',
                round(self._compute_snr(), 2),
                '',
                round(time.time() - t0, 2),
                '',
                *[round(v, 1) for v in self._line_vals],
                round(self._gas_adc, 1),
                round(ambient_at_breach * 100.0, 1),
                round(latency_s, 4) if latency_s >= 0 else '',
                continued,
            ])
            self._summary.add(trial, 'C switchover_latency_s', float(latency_s), 's')
            self._summary.add(trial, 'C ambient_at_breach_frac', ambient_at_breach, '')

    def _run_decay_simulation(self) -> None:
        self._mock_snr_boost = 0.0
        led_pub = self.create_publisher(Float32, '/led_intensity', 10)
        time.sleep(0.3)

        self._wait_enter('Sub-exp D: robot centred on trail; Enter to step PWM 100%→10% ...')

        last_ok_pwm: float | None = None
        for step_i, pct in enumerate(range(100, 0, -10), start=1):
            intensity = pct / 100.0
            led_pub.publish(Float32(data=float(intensity)))
            self.get_logger().info(f'D step {step_i}: LED PWM = {intensity:.2f}')
            time.sleep(2.0)
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._mock:
                scale = intensity
                self._line_vals = [1800 * scale + 200] * 4

            s0, s1, s2, s3 = self._line_vals
            any_on = any(v > TRAIL_THRESHOLD_ADC for v in self._line_vals)
            all_off = all(v < TRAIL_THRESHOLD_ADC for v in self._line_vals)
            snr_db = self._compute_snr()

            if any_on:
                last_ok_pwm = intensity

            self._csv.write_row([
                'D_decay',
                step_i,
                'decay',
                '',
                '',
                'optical',
                round(snr_db, 2),
                all_off,
                '',
                round(intensity, 3),
                round(s0, 1),
                round(s1, 1),
                round(s2, 1),
                round(s3, 1),
                round(self._gas_adc, 1),
                '',
                '',
                any_on,
            ])
            self.get_logger().info(
                f'  ADC=[{s0:.0f},{s1:.0f},{s2:.0f},{s3:.0f}]  any>{TRAIL_THRESHOLD_ADC:.0f}={any_on}  '
                f'SNR={snr_db:.2f} dB'
            )

            if all_off and last_ok_pwm is not None:
                self.get_logger().info(
                    f'Minimum detectable PWM (last any-sensor-on step): {last_ok_pwm:.2f}'
                )
                break

        led_pub.publish(Float32(data=1.0))
        if last_ok_pwm is not None:
            self._summary.add(0, 'D min_detectable_pwm', last_ok_pwm, '')
            self.get_logger().info(
                f'Done decay: min detectable PWM ≈ {last_ok_pwm:.2f} '
                f'(link τ_evaporation=0.08/s model in Ch.3)'
            )

    def _all_sensors_below_trail(self) -> bool:
        return all(float(v) < TRAIL_THRESHOLD_ADC for v in self._line_vals)

    def _lateral_error_for_control(self) -> float:
        if self._chemical_active:
            if self._gas_adc >= self._gas_sat:
                return max(-1.0, min(1.0, self._last_optical_error))
            span = max(self._chem_thr - self._chem_base, 50.0)
            err = (self._gas_adc - self._chem_base) / span
            return max(-1.0, min(1.0, err))
        return self._compute_lateral_error_optical()

    def _compute_lateral_error_optical(self) -> float:
        weights = [-3.0, -1.0, 1.0, 3.0]
        total_w = 0.0
        wsum = 0.0
        for val, w in zip(self._line_vals, weights):
            activation = max(0.0, float(val) - TRAIL_THRESHOLD_ADC)
            wsum += activation * w
            total_w += activation
        if total_w < 1e-6:
            return 0.0
        return wsum / total_w

    def _pid_control(self, error: float) -> Twist:
        now = time.time()
        dt = (now - self._pid_last_time) if self._pid_last_time else 0.02
        dt = max(dt, 1e-4)
        self._pid_last_time = now

        self._pid_integral += error * dt
        derivative = (error - self._pid_prev_error) / dt
        self._pid_prev_error = error

        angular_z = -(KP * error + KI * self._pid_integral + KD * derivative)
        angular_z = max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, angular_z))

        cmd = Twist()
        cmd.linear.x = LINEAR_SPEED_MPS
        cmd.angular.z = angular_z
        return cmd

    def _compute_snr(self) -> float:
        values = [float(v) for v in self._line_vals]
        if self._mock:
            noise_floor = 5.0 + self._mock_snr_boost * 450.0 + self._ambient_light * 40.0
            signal = max(statistics.mean(values), 1.0)
            noise = max(statistics.stdev(values) if len(values) > 1 else 1.0, noise_floor)
        else:
            signal = max(statistics.mean(values), 1.0)
            noise = statistics.stdev(values) if len(values) > 1 else 1.0
            noise = max(noise, 1.0)
            noise = max(noise, self._ambient_light * 50.0)
        return 20.0 * math.log10(signal / noise)

    def _reset_pid(self) -> None:
        self._pid_integral = 0.0
        self._pid_prev_error = 0.0
        self._pid_last_time = None

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PheromoneTrailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and node.context.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
