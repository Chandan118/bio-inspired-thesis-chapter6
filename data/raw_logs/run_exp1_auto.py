#!/usr/bin/env python3
import csv
import math
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray


OUT_DIR = Path("/home/jetson/exp1_logs")
OUT_DIR.mkdir(parents=True, exist_ok=True)


class Exp1Runner(Node):
    def __init__(self):
        super().__init__("exp1_auto_runner")
        self.msg_times = {
            "/scan": [],
            "/imu/data": [],
            "/odom": [],
            "/line_sensors": [],
            "/gas_sensor": [],
            "/rgb/image_raw": [],
        }
        self.scan_front = []
        self.imu_z = []
        self.odom_xy = []
        self.line_means = []
        self.gas_vals = []

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 50)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Float32MultiArray, "/line_sensors", self._line_cb, 20)
        self.create_subscription(Float32, "/gas_sensor", self._gas_cb, 20)
        self.create_subscription(Image, "/rgb/image_raw", self._rgb_cb, 10)

    def _stamp(self, topic):
        self.msg_times[topic].append(time.time())

    def _scan_cb(self, msg: LaserScan):
        self._stamp("/scan")
        if not msg.ranges or msg.angle_increment == 0.0:
            return
        idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        if 0 <= idx < len(msg.ranges):
            r = msg.ranges[idx]
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                self.scan_front.append(float(r))

    def _imu_cb(self, msg: Imu):
        self._stamp("/imu/data")
        z = float(msg.angular_velocity.z)
        if math.isfinite(z):
            self.imu_z.append(z)

    def _odom_cb(self, msg: Odometry):
        self._stamp("/odom")
        self.odom_xy.append((float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)))

    def _line_cb(self, msg: Float32MultiArray):
        self._stamp("/line_sensors")
        if msg.data:
            vals = [float(v) for v in msg.data]
            self.line_means.append(statistics.mean(vals))

    def _gas_cb(self, msg: Float32):
        self._stamp("/gas_sensor")
        self.gas_vals.append(float(msg.data))

    def _rgb_cb(self, _msg: Image):
        self._stamp("/rgb/image_raw")

    def topic_hz(self, topic):
        ts = self.msg_times[topic]
        if len(ts) < 2:
            return 0.0
        dur = ts[-1] - ts[0]
        if dur <= 0:
            return 0.0
        return (len(ts) - 1) / dur


def compute_imu_drift(imu_z):
    if len(imu_z) < 100:
        return None, None
    z = imu_z[:1000] if len(imu_z) >= 1000 else imu_z
    bias = statistics.mean(z)
    drift = abs(bias) * (180.0 / math.pi) * 60.0
    return bias, drift


def compute_tcrt_snr(line_means):
    if len(line_means) < 20:
        return None
    noise = statistics.pstdev(line_means[: min(50, len(line_means))])
    signal = statistics.mean(line_means)
    noise = max(noise, 1.0)
    signal = max(signal, 1.0)
    return 20.0 * math.log10(signal / noise)


def run_odom_trials(node: Exp1Runner):
    trials = []
    for _ in range(10):
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.odom_xy:
            continue
        start = node.odom_xy[-1]
        cmd = Twist()
        cmd.linear.x = 0.20
        end_t = time.time() + 10.0
        while time.time() < end_t:
            node.cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.cmd_pub.publish(Twist())
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.odom_xy:
            continue
        end = node.odom_xy[-1]
        d = math.hypot(end[0] - start[0], end[1] - start[1])
        err_pct = abs(d - 2.0) / 2.0 * 100.0
        trials.append(err_pct)
        time.sleep(0.5)
    if not trials:
        return None, None
    mean_e = statistics.mean(trials)
    sd_e = statistics.stdev(trials) if len(trials) > 1 else 0.0
    return mean_e, sd_e


def main():
    rclpy.init()
    node = Exp1Runner()

    collect_until = time.time() + 12.0
    while time.time() < collect_until:
        rclpy.spin_once(node, timeout_sec=0.05)

    imu_bias, imu_drift = compute_imu_drift(node.imu_z)
    tcrt_snr = compute_tcrt_snr(node.line_means)
    odom_mean, odom_sd = run_odom_trials(node)

    table_path = OUT_DIR / "table6_1_auto.csv"
    with table_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value", "unit", "target", "pass_fail", "notes"])
        w.writerow([
            "LiDAR_RMSE",
            "",
            "m",
            "<=0.02",
            "N/A",
            "Operator reports LiDAR visual OK; CLI cannot capture /scan messages",
        ])
        w.writerow([
            "IMU_Drift",
            f"{imu_drift:.6f}" if imu_drift is not None else "",
            "deg/min",
            "<=0.5",
            "PASS" if imu_drift is not None and imu_drift <= 0.5 else "FAIL",
            f"bias_rad_s={imu_bias:.8f}" if imu_bias is not None else "Insufficient /imu/data",
        ])
        w.writerow([
            "Odom_Mean_Error",
            f"{odom_mean:.4f}" if odom_mean is not None else "",
            "%",
            "<=2.0",
            "PASS" if odom_mean is not None and odom_mean <= 2.0 else "FAIL",
            "Auto open-loop 0.2 m/s x 10s trials (proxy)",
        ])
        w.writerow([
            "Odom_SD",
            f"{odom_sd:.4f}" if odom_sd is not None else "",
            "%",
            "N/A",
            "N/A",
            "Auto proxy trials",
        ])
        w.writerow([
            "RGBD_Reprojection",
            "",
            "px",
            "<=0.5",
            "N/A",
            "Requires checkerboard GUI calibration run",
        ])
        w.writerow([
            "TCRT5000_SNR",
            f"{tcrt_snr:.4f}" if tcrt_snr is not None else "",
            "dB",
            ">=6.0",
            "PASS" if tcrt_snr is not None and tcrt_snr >= 6.0 else "FAIL",
            "Estimated from live /line_sensors stream",
        ])

    summary_path = OUT_DIR / "exp1_topic_health.txt"
    with summary_path.open("w") as f:
        for t in ["/scan", "/imu/data", "/odom", "/line_sensors", "/gas_sensor", "/rgb/image_raw"]:
            f.write(f"{t}_hz,{node.topic_hz(t):.3f}\n")

    print(f"Wrote: {table_path}")
    print(f"Wrote: {summary_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
