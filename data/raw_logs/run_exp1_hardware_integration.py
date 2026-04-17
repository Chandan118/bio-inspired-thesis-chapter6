#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray


OUT_DIR = Path("/home/jetson/exp1_logs")
REQUIRED_TOPICS = ["/scan", "/imu/data", "/odom", "/line_sensors", "/gas_sensor", "/rgb/image_raw"]


def now_s() -> float:
    return time.time()


def rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(sum(v * v for v in values) / len(values))


class Exp1HardwareNode(Node):
    def __init__(self, obstacle_stop_m: float):
        super().__init__("exp1_hardware_integration")
        self.obstacle_stop_m = obstacle_stop_m

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.topic_times = {t: [] for t in REQUIRED_TOPICS}
        self.scan_front_samples = []
        self.imu_z_samples = []
        self.odom_xy = []
        self.line_values = []
        self.last_front_min = float("inf")
        self.last_scan_t = 0.0

        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Float32MultiArray, "/line_sensors", self._line_cb, 20)
        self.create_subscription(Float32, "/gas_sensor", self._gas_cb, 20)
        self.create_subscription(Image, "/rgb/image_raw", self._rgb_cb, qos_profile_sensor_data)

    def _stamp(self, topic):
        self.topic_times[topic].append(now_s())

    def _scan_cb(self, msg: LaserScan):
        self._stamp("/scan")
        self.last_scan_t = now_s()
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        # 0 deg front index
        i0 = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        if 0 <= i0 < len(msg.ranges):
            r0 = msg.ranges[i0]
            if math.isfinite(r0) and msg.range_min <= r0 <= msg.range_max:
                self.scan_front_samples.append(float(r0))

        # Safety band +/- 20 deg for obstacle stop
        vals = []
        for deg in range(-20, 21):
            a = math.radians(deg)
            idx = int(round((a - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                v = msg.ranges[idx]
                if math.isfinite(v) and msg.range_min <= v <= msg.range_max:
                    vals.append(v)
        if vals:
            self.last_front_min = min(vals)

    def _imu_cb(self, msg: Imu):
        self._stamp("/imu/data")
        z = float(msg.angular_velocity.z)
        if math.isfinite(z):
            self.imu_z_samples.append(z)

    def _odom_cb(self, msg: Odometry):
        self._stamp("/odom")
        p = msg.pose.pose.position
        self.odom_xy.append((float(p.x), float(p.y), now_s()))
        if len(self.odom_xy) > 4000:
            self.odom_xy.pop(0)

    def _line_cb(self, msg: Float32MultiArray):
        self._stamp("/line_sensors")
        if msg.data:
            self.line_values = [float(v) for v in msg.data]

    def _gas_cb(self, _msg: Float32):
        self._stamp("/gas_sensor")

    def _rgb_cb(self, _msg: Image):
        self._stamp("/rgb/image_raw")

    def stop(self):
        self.cmd_pub.publish(Twist())

    def wait(self, seconds: float):
        end_t = now_s() + seconds
        while now_s() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def topic_rate(self, topic: str, window_s: float = 8.0) -> float:
        ts = self.topic_times.get(topic, [])
        if len(ts) < 2:
            return 0.0
        cutoff = now_s() - window_s
        recent = [t for t in ts if t >= cutoff]
        if len(recent) < 2:
            return 0.0
        dt = recent[-1] - recent[0]
        if dt <= 0:
            return 0.0
        return (len(recent) - 1) / dt

    def topic_seen(self, topic: str) -> bool:
        return len(self.topic_times.get(topic, [])) > 0

    def collect_lidar_front(self, n_samples: int, timeout_s: float = 10.0):
        start_i = len(self.scan_front_samples)
        end_t = now_s() + timeout_s
        while len(self.scan_front_samples) - start_i < n_samples and now_s() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)
        vals = self.scan_front_samples[start_i:start_i + n_samples]
        return vals

    def collect_imu_z(self, n_samples: int, timeout_s: float = 30.0):
        start_i = len(self.imu_z_samples)
        end_t = now_s() + timeout_s
        while len(self.imu_z_samples) - start_i < n_samples and now_s() < end_t:
            rclpy.spin_once(self, timeout_sec=0.02)
        vals = self.imu_z_samples[start_i:start_i + n_samples]
        return vals

    def collect_line_means(self, seconds: float):
        vals = []
        end_t = now_s() + seconds
        while now_s() < end_t:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.line_values:
                vals.append(statistics.mean(self.line_values))
        return vals

    def drive_straight_trial(self, speed_scale: float, target_m: float, max_time_s: float = 45.0):
        if not self.odom_xy:
            self.wait(1.0)
        if not self.odom_xy:
            return None

        x0, y0, _ = self.odom_xy[-1]
        v_nominal = 0.20
        v_target = max(0.0, min(v_nominal, v_nominal * speed_scale))
        v_cmd = 0.0
        ramp_up_mps2 = 0.08  # gentle acceleration for smooth motion
        end_t = now_s() + max_time_s

        while now_s() < end_t:
            if not rclpy.ok():
                break
            rclpy.spin_once(self, timeout_sec=0.02)
            cmd = Twist()
            lidar_fresh = (now_s() - self.last_scan_t) < 1.5

            # Safety policy requested by user: stop near obstacle.
            if lidar_fresh and self.last_front_min <= self.obstacle_stop_m:
                self.stop()
                v_cmd = 0.0
                time.sleep(0.05)
                continue

            # Slow down early before reaching stop threshold to avoid bumping.
            desired = v_target
            if lidar_fresh:
                if self.last_front_min <= (self.obstacle_stop_m + 0.20):
                    desired = min(desired, 0.04)
                elif self.last_front_min <= (self.obstacle_stop_m + 0.35):
                    desired = min(desired, 0.06)
                elif self.last_front_min <= (self.obstacle_stop_m + 0.50):
                    desired = min(desired, 0.08)

            # First-order ramp limit to keep wheel motion smooth.
            step = ramp_up_mps2 * 0.05
            if v_cmd < desired:
                v_cmd = min(desired, v_cmd + step)
            else:
                v_cmd = max(desired, v_cmd - step)

            cmd.linear.x = v_cmd
            try:
                self.cmd_pub.publish(cmd)
            except Exception:
                break
            time.sleep(0.05)

            x1, y1, _ = self.odom_xy[-1]
            d = math.hypot(x1 - x0, y1 - y0)
            if d >= target_m:
                break

        self.stop()
        self.wait(0.5)
        x1, y1, _ = self.odom_xy[-1]
        return math.hypot(x1 - x0, y1 - y0)


def main():
    parser = argparse.ArgumentParser(description="Experiment 1 Hardware Integration & Multi-Sensor Calibration")
    parser.add_argument("--speed-scale", type=float, default=0.37, help="Robot speed scale for odom trials")
    parser.add_argument("--obstacle-stop-m", type=float, default=0.30, help="Stop threshold from LiDAR front minimum")
    parser.add_argument("--clean-old", action="store_true", help="Delete older exp1/table6_1 outputs before run")
    parser.add_argument("--rgbd-reprojection", type=float, default=float("nan"), help="Manual reprojection error from camera_calibration output")
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if args.clean_old:
        for p in OUT_DIR.glob("table6_1*.csv"):
            p.unlink(missing_ok=True)
        for p in OUT_DIR.glob("exp1_*"):
            if p.is_file():
                p.unlink(missing_ok=True)

    run_tag = time.strftime("%Y%m%d_%H%M%S")
    table_path = OUT_DIR / f"table6_1_exp1_{run_tag}.csv"
    details_path = OUT_DIR / f"exp1_run_details_{run_tag}.txt"

    rclpy.init()
    node = Exp1HardwareNode(obstacle_stop_m=args.obstacle_stop_m)
    try:
        # Task 1: Bring-up verification and topic-rate checks
        node.get_logger().info("Task 1/6: Waiting 10s for sensor streams...")
        node.wait(10.0)
        topic_rows = []
        for topic in REQUIRED_TOPICS:
            hz = node.topic_rate(topic)
            seen = node.topic_seen(topic)
            topic_rows.append((topic, seen, hz))

        scan_hz = node.topic_rate("/scan")
        imu_hz = node.topic_rate("/imu/data")
        task1_pass = all(seen for _, seen, _ in topic_rows) and scan_hz >= 8.0 and imu_hz >= 50.0

        # Task 2: LiDAR calibration over 4 distances, 15 readings each
        lidar_gt = [0.50, 1.00, 1.50, 2.00]
        lidar_measured = []
        lidar_errors = []
        for d in lidar_gt:
            node.get_logger().info(f"Task 2/6: Place robot at {d:.2f} m from flat wall. Sampling in 6s...")
            node.wait(6.0)
            vals = node.collect_lidar_front(15, timeout_s=12.0)
            mean_v = statistics.mean(vals) if vals else float("nan")
            lidar_measured.append(mean_v)
            lidar_errors.append(abs(mean_v - d) if math.isfinite(mean_v) else float("nan"))
        valid_lidar_errs = [e for e in lidar_errors if math.isfinite(e)]
        lidar_rmse = rmse(valid_lidar_errs) if valid_lidar_errs else float("nan")

        # Task 3: IMU drift from 1000 angular_velocity.z samples
        node.get_logger().info("Task 3/6: Keep robot stationary. Collecting 1000 IMU samples...")
        imu_vals = node.collect_imu_z(1000, timeout_s=45.0)
        imu_bias = statistics.mean(imu_vals) if imu_vals else float("nan")
        imu_drift_deg_min = abs(imu_bias) * (180.0 / math.pi) * 60.0 if math.isfinite(imu_bias) else float("nan")

        # Task 4: Wheel odometry calibration 10 trials, 2.00 m each, user-selected speed
        node.get_logger().info(
            f"Task 4/6: Odom 10 trials at 2.00 m each, safety-stop active, speed_scale={args.speed_scale:.2f}."
        )
        odom_target = 2.0
        odom_errors_pct = []
        odom_measured = []
        for i in range(10):
            node.get_logger().info(f"Trial {i + 1}/10: place robot on start mark, run starts in 5s...")
            node.wait(5.0)
            d = node.drive_straight_trial(speed_scale=args.speed_scale, target_m=odom_target, max_time_s=60.0)
            if d is None:
                continue
            odom_measured.append(d)
            odom_errors_pct.append(abs(d - odom_target) / odom_target * 100.0)
            node.wait(2.0)
        odom_mean = statistics.mean(odom_errors_pct) if odom_errors_pct else float("nan")
        odom_sd = statistics.stdev(odom_errors_pct) if len(odom_errors_pct) > 1 else float("nan")

        # Task 5: RGB-D reprojection error (manual from camera_calibration output)
        rgbd_rep = args.rgbd_reprojection

        # Task 6: TCRT5000 SNR at 5, 10, 15 cm and ambient noise
        tcrt_signal_means = []
        for cm in [5, 10, 15]:
            node.get_logger().info(f"Task 6/6: Place line sensors over 620nm LED at {cm} cm, sampling in 5s...")
            node.wait(5.0)
            sig = node.collect_line_means(2.0)
            tcrt_signal_means.append(statistics.mean(sig) if sig else float("nan"))
        node.get_logger().info("Task 6/6: Turn LED OFF for ambient noise measurement (2s)...")
        node.wait(4.0)
        noise_samples = node.collect_line_means(2.0)
        noise_std = statistics.pstdev(noise_samples) if len(noise_samples) > 1 else float("nan")
        signal_mean = statistics.mean([v for v in tcrt_signal_means if math.isfinite(v)]) if any(
            math.isfinite(v) for v in tcrt_signal_means
        ) else float("nan")
        tcrt_snr_db = 20.0 * math.log10(signal_mean / noise_std) if (
            math.isfinite(signal_mean) and math.isfinite(noise_std) and noise_std > 0.0 and signal_mean > 0.0
        ) else float("nan")

        def pf(v, cond):
            return "PASS" if math.isfinite(v) and cond else "FAIL"

        with table_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "value", "unit", "target", "pass_fail", "notes"])
            w.writerow(["LiDAR_RMSE", f"{lidar_rmse:.6f}" if math.isfinite(lidar_rmse) else "", "m", "<=0.02",
                        pf(lidar_rmse, lidar_rmse <= 0.02), "4 distances: 0.5/1.0/1.5/2.0m, 15 front samples each"])
            w.writerow(["IMU_Drift", f"{imu_drift_deg_min:.6f}" if math.isfinite(imu_drift_deg_min) else "", "deg/min", "<=0.5",
                        pf(imu_drift_deg_min, imu_drift_deg_min <= 0.5), f"bias_rad_s={imu_bias:.8f}" if math.isfinite(imu_bias) else "insufficient imu"])
            w.writerow(["Odom_Mean_Error", f"{odom_mean:.4f}" if math.isfinite(odom_mean) else "", "%", "<=2.0",
                        pf(odom_mean, odom_mean <= 2.0), f"10 trials, speed_scale={args.speed_scale:.2f}"])
            w.writerow(["Odom_SD", f"{odom_sd:.4f}" if math.isfinite(odom_sd) else "", "%", "N/A", "N/A", "std of 10 odom errors"])
            w.writerow(["RGBD_Reprojection", f"{rgbd_rep:.4f}" if math.isfinite(rgbd_rep) else "", "px", "<=0.5",
                        pf(rgbd_rep, rgbd_rep <= 0.5), "from camera_calibration output"])
            w.writerow(["TCRT5000_SNR", f"{tcrt_snr_db:.4f}" if math.isfinite(tcrt_snr_db) else "", "dB", ">=6.0",
                        pf(tcrt_snr_db, tcrt_snr_db >= 6.0), "SNR=20*log10(signal_mean/noise_std), 5/10/15cm"])

        with details_path.open("w") as f:
            f.write("task1_topic_health\n")
            for topic, seen, hz in topic_rows:
                f.write(f"{topic},seen={seen},hz={hz:.3f}\n")
            f.write(f"task1_pass,{task1_pass}\n")
            f.write(f"scan_hz,{scan_hz:.3f}\n")
            f.write(f"imu_hz,{imu_hz:.3f}\n")
            f.write("\nlidar_per_distance\n")
            for gt, meas, err in zip(lidar_gt, lidar_measured, lidar_errors):
                f.write(f"gt_m={gt:.2f},measured_m={meas if math.isfinite(meas) else 'nan'},abs_err_m={err if math.isfinite(err) else 'nan'}\n")
            f.write("\nimu\n")
            f.write(f"samples,{len(imu_vals)}\n")
            f.write(f"bias_rad_s,{imu_bias if math.isfinite(imu_bias) else 'nan'}\n")
            f.write(f"drift_deg_per_min,{imu_drift_deg_min if math.isfinite(imu_drift_deg_min) else 'nan'}\n")
            f.write("\nodom_trials\n")
            for i, (d, e) in enumerate(zip(odom_measured, odom_errors_pct), start=1):
                f.write(f"trial_{i},dist_m={d:.4f},err_pct={e:.4f}\n")
            f.write("\ntcrt\n")
            f.write(f"signal_means,{tcrt_signal_means}\n")
            f.write(f"noise_std,{noise_std if math.isfinite(noise_std) else 'nan'}\n")
            f.write(f"snr_db,{tcrt_snr_db if math.isfinite(tcrt_snr_db) else 'nan'}\n")

        print(f"TABLE_6_1_CSV,{table_path}")
        print(f"DETAILS_TXT,{details_path}")
        print(f"TASK1_PASS,{task1_pass}")
        print(f"LIDAR_RMSE_M,{lidar_rmse if math.isfinite(lidar_rmse) else 'nan'}")
        print(f"IMU_DRIFT_DEG_PER_MIN,{imu_drift_deg_min if math.isfinite(imu_drift_deg_min) else 'nan'}")
        print(f"ODOM_MEAN_ERR_PCT,{odom_mean if math.isfinite(odom_mean) else 'nan'}")
        print(f"ODOM_SD_PCT,{odom_sd if math.isfinite(odom_sd) else 'nan'}")
        print(f"RGBD_REPROJ_PX,{rgbd_rep if math.isfinite(rgbd_rep) else 'nan'}")
        print(f"TCRT_SNR_DB,{tcrt_snr_db if math.isfinite(tcrt_snr_db) else 'nan'}")

    finally:
        try:
            node.stop()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
