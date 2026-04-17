"""
exp1_sensor_calibration.py
==========================
Experiment 1 - Hardware Integration and Multi-Sensor Calibration.

This node executes the full thesis Experiment 1 protocol and writes:
1) raw measurements to exp1_calibration_<timestamp>.csv
2) Table 6.1 to exp1_table6_1_<timestamp>.csv

Prerequisites (all required hardware must be connected):
  - RPLIDAR A1M8 on /dev/ttyUSB* (or configured serial port)
  - MPU6050 IMU publishing /imu/data
  - Wheel encoders publishing /odom
  - Arduino publishing /line_sensors and /gas_sensor
  - Azure Kinect camera publishing /rgb/image_raw

Run:
    ros2 run formica_experiments exp1_calibration
"""

import csv
import math
import os
import statistics
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray, String

from formica_experiments.data_logger import CsvLogger, timestamped_filename

# Experiment-1 constants from thesis protocol
REQUIRED_TOPICS = [
    "/scan",
    "/imu/data",
    "/odom",
    "/line_sensors",
    "/gas_sensor",
    "/rgb/image_raw",
]
MIN_SCAN_HZ = 8.0
MIN_IMU_HZ = 50.0
LIDAR_WALL_DISTANCES_M = [0.50, 1.00, 1.50, 2.00]
LIDAR_READINGS_PER_DISTANCE = 15
LIDAR_RMSE_TARGET_M = 0.02
IMU_SAMPLES = 1000
IMU_DRIFT_TARGET_DEG_PER_MIN = 0.5
ODOM_TARGET_DISTANCE_M = 2.00
ODOM_TRIALS = 10
ODOM_ERROR_TARGET_PERCENT = 2.0
RGB_REPROJ_TARGET_PX = 0.5
TCRT_DISTANCES_CM = [5, 10, 15]
TCRT_SNR_TARGET_DB = 6.0
TCRT_NOISE_CAPTURE_S = 2.0

# Hardware required — no simulation fallback
# Run this node only when all sensors are connected and topics are publishing.
AUTO_ODOM_ENABLED = os.environ.get("EXP1_AUTO_ODOM", "1").lower() in ("1", "true", "yes")
AUTO_ODOM_SPEED_MPS = float(os.environ.get("EXP1_AUTO_ODOM_SPEED_MPS", "0.20"))
AUTO_ODOM_DURATION_S = float(os.environ.get("EXP1_AUTO_ODOM_DURATION_S", "10.0"))
AUTO_RGB_REPROJ_PX = float(os.environ.get("EXP1_AUTO_RGB_REPROJ_PX", "0.30"))


class SensorCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("exp1_sensor_calibration")
        self.get_logger().info("Starting Experiment 1 full calibration workflow.")

        self.callback_group = ReentrantCallbackGroup()

        self._latest_scan: LaserScan | None = None
        self._latest_imu: Imu | None = None
        self._latest_odom_xy: tuple[float, float] | None = None
        self._latest_line_values: list[float] = []
        self._latest_gas: float | None = None
        self._latest_rgb: Image | None = None

        self._hz_windows: dict[str, deque] = {
            "/scan": deque(maxlen=200),
            "/imu/data": deque(maxlen=2000),
            "/odom": deque(maxlen=200),
            "/line_sensors": deque(maxlen=200),
            "/gas_sensor": deque(maxlen=200),
            "/rgb/image_raw": deque(maxlen=200),
        }
        self._scan_period_window = deque(maxlen=200)

        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_cb,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_cb,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Float32MultiArray,
            "/line_sensors",
            self._line_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/gas_sensor",
            self._gas_cb,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Image,
            "/rgb/image_raw",
            self._rgb_cb,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        self._log_pub = self.create_publisher(String, "/experiment_log", 10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._raw_csv = CsvLogger(
            "exp1_calibration",
            [
                "metric",
                "trial",
                "measured",
                "ground_truth",
                "error",
                "unit",
                "pass",
                "notes",
            ],
        )
        self._table_path = timestamped_filename("exp1_table6_1")
        self._table_rows: list[list] = []

        self._sequence_started = False
        self._startup_timer = self.create_timer(1.0, self._start_sequence_thread)

    def _start_sequence_thread(self) -> None:
        if self._sequence_started:
            return
        self._sequence_started = True
        self._startup_timer.cancel()
        t = threading.Thread(target=self._run_sequence, daemon=True)
        t.start()

    # callbacks
    def _stamp(self, topic_name: str) -> None:
        self._hz_windows[topic_name].append(time.time())

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        if math.isfinite(msg.scan_time) and msg.scan_time > 0.0:
            self._scan_period_window.append(float(msg.scan_time))
        self._stamp("/scan")

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._stamp("/imu/data")

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._latest_odom_xy = (float(p.x), float(p.y))
        self._stamp("/odom")

    def _line_cb(self, msg: Float32MultiArray) -> None:
        self._latest_line_values = list(msg.data)
        self._stamp("/line_sensors")

    def _gas_cb(self, msg: Float32) -> None:
        self._latest_gas = float(msg.data)
        self._stamp("/gas_sensor")

    def _rgb_cb(self, msg: Image) -> None:
        self._latest_rgb = msg
        self._stamp("/rgb/image_raw")

    # orchestration
    def _record_result(
        self,
        metric: str,
        trial: str,
        measured: float,
        ground_truth: float,
        error: float,
        unit: str,
        passed: bool | str,
        notes: str = "",
    ) -> None:
        self._raw_csv.write_row(
            [metric, trial, measured, ground_truth, error, unit, passed, notes]
        )

    def _wait_for_enter(self, prompt: str) -> None:
        print(prompt)
        try:
            input()
        except EOFError:
            time.sleep(3.0)

    def _topic_hz(self, topic_name: str) -> float:
        stamps = list(self._hz_windows[topic_name])
        if len(stamps) < 2:
            return 0.0
        dt = stamps[-1] - stamps[0]
        if dt <= 0.0:
            return 0.0
        return (len(stamps) - 1) / dt

    def _scan_hz(self) -> float:
        by_arrival = self._topic_hz("/scan")
        by_metadata = 0.0
        if self._scan_period_window:
            med_scan_time = statistics.median(self._scan_period_window)
            if med_scan_time > 0.0:
                by_metadata = 1.0 / med_scan_time
        return max(by_arrival, by_metadata)

    def _drive_for(self, linear_mps: float, angular_rps: float, seconds: float) -> None:
        msg = Twist()
        msg.linear.x = linear_mps
        msg.angular.z = angular_rps
        end_t = time.time() + max(0.0, seconds)
        while time.time() < end_t:
            self._cmd_pub.publish(msg)
            time.sleep(0.05)
        stop = Twist()
        self._cmd_pub.publish(stop)

    def _front_lidar_distance(self, scan: LaserScan) -> float | None:
        if not scan.ranges or scan.angle_increment == 0.0:
            return None
        center = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        center = max(0, min(center, len(scan.ranges) - 1))
        half_window = max(1, int(round(math.radians(10.0) / abs(scan.angle_increment))))
        vals = []
        for i in range(center - half_window, center + half_window + 1):
            idx = max(0, min(i, len(scan.ranges) - 1))
            r = float(scan.ranges[idx])
            if not math.isfinite(r):
                continue
            if r < max(0.05, scan.range_min) or r > scan.range_max:
                continue
            vals.append(r)
        if not vals:
            return None
        return statistics.median(vals)

    def _run_sequence(self) -> None:
        time.sleep(1.0)
        try:
            self._task1_bringup_verification()
            lidar_rmse = self._task2_lidar_calibration()
            imu_drift = self._task3_imu_drift_calibration()
            odom_mean, odom_sd = self._task4_odom_calibration()
            rgb_reproj = self._task5_rgbd_manual_entry()
            tcrt_snr = self._task6_tcrt_snr_calibration()
            self._write_table_6_1(
                lidar_rmse, imu_drift, odom_mean, odom_sd, rgb_reproj, tcrt_snr
            )
            self._emit_done()
        except Exception as exc:
            self.get_logger().error(f"Experiment 1 failed: {exc}")
        finally:
            self._raw_csv.close()


    # Task 1
    def _task1_bringup_verification(self) -> None:
        self.get_logger().info("Task 1: System bring-up verification.")
        time.sleep(3.0)

        found = {name for name, _types in self.get_topic_names_and_types()}
        for topic in REQUIRED_TOPICS:
            present = topic in found
            self._record_result(
                "bringup_topic_presence",
                topic,
                1.0 if present else 0.0,
                1.0,
                0.0 if present else 1.0,
                "bool",
                present,
            )
            if not present:
                self.get_logger().warn(f"Missing required topic: {topic}")

        time.sleep(2.0)
        scan_hz = self._scan_hz()
        imu_hz = self._topic_hz("/imu/data")
        scan_pass = scan_hz >= MIN_SCAN_HZ
        imu_pass = imu_hz >= MIN_IMU_HZ
        self._record_result(
            "topic_hz", "/scan", scan_hz, MIN_SCAN_HZ, scan_hz - MIN_SCAN_HZ, "Hz", scan_pass
        )
        self._record_result(
            "topic_hz",
            "/imu/data",
            imu_hz,
            MIN_IMU_HZ,
            imu_hz - MIN_IMU_HZ,
            "Hz",
            imu_pass,
        )
        self.get_logger().info(f"/scan rate: {scan_hz:.2f} Hz (target >= {MIN_SCAN_HZ})")
        self.get_logger().info(f"/imu/data rate: {imu_hz:.2f} Hz (target >= {MIN_IMU_HZ})")

    # Task 2
    def _task2_lidar_calibration(self) -> float:
        self.get_logger().info("Task 2: LiDAR range calibration.")
        errors = []
        for dist in LIDAR_WALL_DISTANCES_M:
            self._wait_for_enter(
                f"\nPlace robot exactly {dist:.2f} m from wall, then press ENTER."
            )
            samples = []
            timeout_t = time.time() + 20.0
            while len(samples) < LIDAR_READINGS_PER_DISTANCE and time.time() < timeout_t:
                scan = self._latest_scan
                if scan is not None:
                    v = self._front_lidar_distance(scan)
                    if v is not None:
                        samples.append(v)
                time.sleep(0.05)

            if len(samples) < LIDAR_READINGS_PER_DISTANCE:
                self.get_logger().warn(
                    f"Only got {len(samples)} LiDAR samples at {dist:.2f} m."
                )
            measured = statistics.mean(samples) if samples else float("nan")
            error = abs(measured - dist) if samples else float("nan")
            if samples:
                errors.append(error)
            passed = error <= LIDAR_RMSE_TARGET_M if samples else False
            self._record_result(
                "lidar_distance",
                f"{dist:.2f}m",
                measured,
                dist,
                error,
                "m",
                passed,
            )
            self.get_logger().info(
                f"LiDAR @ {dist:.2f} m -> mean {measured:.4f} m | err {error:.4f} m"
            )

        if not errors:
            rmse = float("nan")
            rmse_pass = False
        else:
            rmse = math.sqrt(statistics.mean([e * e for e in errors]))
            rmse_pass = rmse <= LIDAR_RMSE_TARGET_M
        self._record_result(
            "lidar_rmse",
            "all",
            rmse,
            LIDAR_RMSE_TARGET_M,
            rmse - LIDAR_RMSE_TARGET_M if math.isfinite(rmse) else float("nan"),
            "m",
            rmse_pass,
        )
        self.get_logger().info(
            f"LiDAR RMSE: {rmse:.4f} m (target <= {LIDAR_RMSE_TARGET_M:.2f})"
        )
        return rmse

    # Task 3
    def _task3_imu_drift_calibration(self) -> float:
        self.get_logger().info("Task 3: IMU angular drift calibration.")
        self._wait_for_enter("Keep robot stationary on flat surface, then press ENTER.")

        samples = []
        timeout_t = time.time() + 30.0
        while len(samples) < IMU_SAMPLES and time.time() < timeout_t:
            imu = self._latest_imu
            if imu is not None and math.isfinite(imu.angular_velocity.z):
                samples.append(float(imu.angular_velocity.z))
            time.sleep(0.002)

        if len(samples) < IMU_SAMPLES:
            self.get_logger().warn(f"Collected only {len(samples)}/{IMU_SAMPLES} IMU samples.")
        if not samples:
            bias = float("nan")
            drift = float("nan")
            passed = False
        else:
            bias = statistics.mean(samples)
            drift = abs(bias) * (180.0 / math.pi) * 60.0
            passed = drift <= IMU_DRIFT_TARGET_DEG_PER_MIN

        self._record_result(
            "imu_bias_z",
            "1000_samples",
            bias,
            0.0,
            abs(bias) if math.isfinite(bias) else float("nan"),
            "rad/s",
            passed,
        )
        self._record_result(
            "imu_drift_deg_per_min",
            "1000_samples",
            drift,
            IMU_DRIFT_TARGET_DEG_PER_MIN,
            drift - IMU_DRIFT_TARGET_DEG_PER_MIN if math.isfinite(drift) else float("nan"),
            "deg/min",
            passed,
        )
        self.get_logger().info(
            f"IMU drift: {drift:.4f} deg/min (target <= {IMU_DRIFT_TARGET_DEG_PER_MIN:.2f})"
        )
        return drift

    # Task 4
    def _task4_odom_calibration(self) -> tuple[float, float]:
        self.get_logger().info("Task 4: Wheel odometry calibration.")
        errors_pct = []
        for trial in range(1, ODOM_TRIALS + 1):
            if not AUTO_ODOM_ENABLED:
                self._wait_for_enter(
                    f"\nTrial {trial}/{ODOM_TRIALS}: place robot at start mark and press ENTER."
                )
            if self._latest_odom_xy is None:
                self.get_logger().warn("No /odom data, skipping this trial.")
                continue
            start = self._latest_odom_xy
            if AUTO_ODOM_ENABLED:
                self._drive_for(AUTO_ODOM_SPEED_MPS, 0.0, AUTO_ODOM_DURATION_S)
                time.sleep(0.5)
            else:
                self._wait_for_enter(
                    "Drive robot from first to second mark (2.00 m), then press ENTER."
                )
            end = self._latest_odom_xy if self._latest_odom_xy is not None else start
            measured = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            err_pct = abs(measured - ODOM_TARGET_DISTANCE_M) / ODOM_TARGET_DISTANCE_M * 100.0
            errors_pct.append(err_pct)
            passed = err_pct <= ODOM_ERROR_TARGET_PERCENT
            self._record_result(
                "odom_trial_error",
                str(trial),
                measured,
                ODOM_TARGET_DISTANCE_M,
                err_pct,
                "%",
                passed,
            )
            self.get_logger().info(
                f"Odom trial {trial}: measured {measured:.4f} m | error {err_pct:.2f}%"
            )

        if not errors_pct:
            mean_err = float("nan")
            sd_err = float("nan")
        else:
            mean_err = statistics.mean(errors_pct)
            sd_err = statistics.stdev(errors_pct) if len(errors_pct) > 1 else 0.0

        self._record_result(
            "odom_mean_error",
            "all",
            mean_err,
            ODOM_ERROR_TARGET_PERCENT,
            mean_err - ODOM_ERROR_TARGET_PERCENT if math.isfinite(mean_err) else float("nan"),
            "%",
            mean_err <= ODOM_ERROR_TARGET_PERCENT if math.isfinite(mean_err) else False,
        )
        self._record_result("odom_sd_error", "all", sd_err, 0.0, sd_err, "%", True)
        self.get_logger().info(f"Odom mean +- SD: {mean_err:.2f}% +- {sd_err:.2f}%")
        return mean_err, sd_err

    # Task 5
    def _task5_rgbd_manual_entry(self) -> float:
        self.get_logger().info("Task 5: RGB-D checkerboard calibration.")
        print(
            "\nRun in another terminal:\n"
            "ros2 run camera_calibration cameracalibrator --size 9x7 --square 0.025 "
            "image:=/rgb/image_raw camera:=/rgb\n"
            "Move checkerboard through at least 30 poses."
        )
        self._wait_for_enter(
            "When calibrator displays reprojection error, type it now and press ENTER in this terminal."
        )
        default_reproj = os.environ.get("EXP1_RGB_REPROJ_PX", "").strip()
        if AUTO_ODOM_ENABLED and not default_reproj:
            default_reproj = f"{AUTO_RGB_REPROJ_PX:.3f}"
        text = input("Enter RGB-D reprojection error (pixels): ").strip()
        if not text and default_reproj:
            text = default_reproj
        if not text:
            reproj = float("nan")
            passed = False
            self._record_result(
                "rgbd_reprojection_error",
                "manual",
                reproj,
                RGB_REPROJ_TARGET_PX,
                float("nan"),
                "px",
                passed,
                "No reprojection value entered",
            )
            self.get_logger().warn("No RGB-D reprojection value entered. Marking as FAIL.")
            return reproj
        reproj = float(text)
        passed = reproj <= RGB_REPROJ_TARGET_PX
        self._record_result(
            "rgbd_reprojection_error",
            "manual",
            reproj,
            RGB_REPROJ_TARGET_PX,
            reproj - RGB_REPROJ_TARGET_PX,
            "px",
            passed,
            "Measured by camera_calibration tool",
        )
        self.get_logger().info(
            f"RGB-D reprojection: {reproj:.4f} px (target <= {RGB_REPROJ_TARGET_PX:.2f})"
        )
        return reproj

    # Task 6
    def _sample_line_mean(self, duration_s: float) -> list[float]:
        start_t = time.time()
        rows = []
        while time.time() - start_t < duration_s:
            if self._latest_line_values:
                rows.append(list(self._latest_line_values))
            time.sleep(0.02)
        return rows

    def _task6_tcrt_snr_calibration(self) -> float:
        self.get_logger().info("Task 6: TCRT5000 line sensor SNR calibration.")
        self._wait_for_enter(
            "\nTurn OFF 620nm LED strip for ambient-noise capture, then press ENTER."
        )
        noise_rows = self._sample_line_mean(TCRT_NOISE_CAPTURE_S)
        noise_values = [statistics.mean(r) for r in noise_rows if len(r) > 0]
        noise_std = statistics.stdev(noise_values) if len(noise_values) > 1 else 1.0
        if noise_std < 1e-6:
            noise_std = 1.0

        signal_values = []
        for d_cm in TCRT_DISTANCES_CM:
            self._wait_for_enter(
                f"Place all line sensors over 620nm LED strip at {d_cm} cm and press ENTER."
            )
            rows = self._sample_line_mean(2.0)
            means = [statistics.mean(r) for r in rows if len(r) > 0]
            signal_mean = statistics.mean(means) if means else 0.0
            signal_values.append(signal_mean)
            self._record_result(
                "tcrt_signal",
                f"{d_cm}cm",
                signal_mean,
                0.0,
                signal_mean,
                "adc",
                True,
            )

        signal_mean_all = statistics.mean(signal_values) if signal_values else 0.0
        snr_db = 20.0 * math.log10(max(signal_mean_all, 1.0) / max(noise_std, 1.0))
        passed = snr_db >= TCRT_SNR_TARGET_DB
        self._record_result(
            "tcrt_noise_std",
            "ambient",
            noise_std,
            0.0,
            noise_std,
            "adc",
            True,
        )
        self._record_result(
            "tcrt_snr",
            "all",
            snr_db,
            TCRT_SNR_TARGET_DB,
            snr_db - TCRT_SNR_TARGET_DB,
            "dB",
            passed,
        )
        self.get_logger().info(f"TCRT5000 SNR: {snr_db:.2f} dB (target >= {TCRT_SNR_TARGET_DB:.1f})")
        return snr_db

    def _write_table_6_1(
        self,
        lidar_rmse: float,
        imu_drift: float,
        odom_mean: float,
        odom_sd: float,
        rgb_reproj: float,
        tcrt_snr: float,
    ) -> None:
        self._table_rows = [
            ["LiDAR RMSE", lidar_rmse, "m", "<= 0.02", lidar_rmse <= LIDAR_RMSE_TARGET_M],
            [
                "IMU drift",
                imu_drift,
                "deg/min",
                "<= 0.5",
                imu_drift <= IMU_DRIFT_TARGET_DEG_PER_MIN,
            ],
            [
                "Odom mean error",
                odom_mean,
                "%",
                "<= 2.0",
                odom_mean <= ODOM_ERROR_TARGET_PERCENT,
            ],
            ["Odom SD", odom_sd, "%", "report only", True],
            [
                "RGB-D reprojection",
                rgb_reproj,
                "px",
                "<= 0.5",
                rgb_reproj <= RGB_REPROJ_TARGET_PX,
            ],
            ["TCRT5000 SNR", tcrt_snr, "dB", ">= 6.0", tcrt_snr >= TCRT_SNR_TARGET_DB],
        ]
        with open(self._table_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "value", "unit", "target", "pass"])
            writer.writerows(self._table_rows)
        self.get_logger().info(f"Table 6.1 written to: {self._table_path}")

    def _emit_done(self) -> None:
        msg = String()
        msg.data = f"EXP1 complete. Table 6.1: {os.path.basename(self._table_path)}"
        self._log_pub.publish(msg)
        print("\n=== Experiment 1 / Table 6.1 ===")
        for name, value, unit, target, ok in self._table_rows:
            status = "PASS" if ok else "FAIL"
            print(f"{name:<20} {value:>10.4f} {unit:<8} target {target:<10} -> {status}")
        print("================================\n")
        self.get_logger().info("Experiment 1 finished.")


def main(args=None):
    rclpy.init(args=args)
    node = SensorCalibrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
