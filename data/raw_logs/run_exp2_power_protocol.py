#!/usr/bin/env python3
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray

from formica_experiments.exp2_postprocess import postprocess_exp2

OUT_DIR = Path("/home/jetson/exp1_logs")
DATA_DIR = Path("/home/jetson/formica_experiments/data")


class Exp2PowerNode(Node):
    def __init__(self, transit_speed: float, obstacle_stop_m: float):
        super().__init__("exp2_power_protocol_runner")
        self.transit_speed = transit_speed
        self.obstacle_stop_m = obstacle_stop_m
        self.power_msg = None
        self.power_times = []
        self.imu_seen = False
        self.line_seen = False
        self.last_scan_t = 0.0
        self.front_min_m = float("inf")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Float32MultiArray, "/power_monitor", self._power_cb, 20)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 20)
        self.create_subscription(Float32MultiArray, "/line_sensors", self._line_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 20)

    def _power_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.power_msg = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
            self.power_times.append(time.time())
            if len(self.power_times) > 2000:
                self.power_times.pop(0)

    def _imu_cb(self, _msg: Imu):
        self.imu_seen = True

    def _line_cb(self, _msg: Float32MultiArray):
        self.line_seen = True

    def _scan_cb(self, msg: LaserScan):
        self.last_scan_t = time.time()
        if not msg.ranges or msg.angle_increment == 0.0:
            return
        vals = []
        for deg in range(-20, 21):
            ang = math.radians(deg)
            idx = int(round((ang - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    vals.append(float(r))
        if vals:
            self.front_min_m = min(vals)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def lidar_fresh(self) -> bool:
        return (time.time() - self.last_scan_t) < 1.5 and math.isfinite(self.front_min_m)

    def set_transit(self):
        cmd = Twist()
        cmd.linear.x = self.transit_speed
        self.cmd_pub.publish(cmd)

    def turn_to_change_path(self):
        cmd = Twist()
        cmd.angular.z = 0.55
        self.cmd_pub.publish(cmd)

    def power_hz(self, window_s: float = 5.0) -> float:
        if len(self.power_times) < 2:
            return 0.0
        cutoff = time.time() - window_s
        recent = [t for t in self.power_times if t >= cutoff]
        if len(recent) < 2:
            return 0.0
        dt = recent[-1] - recent[0]
        return (len(recent) - 1) / dt if dt > 0 else 0.0


def mode_at(elapsed_s: float) -> str:
    # Exact cycle requested by protocol: 10.2s TRANSIT, 5.1s DECISION, 1.7s STANDBY.
    t = elapsed_s % 17.0
    if t < 10.2:
        return "TRANSIT"
    if t < 15.3:
        return "DECISION"
    return "STANDBY"


def main():
    parser = argparse.ArgumentParser(description="Experiment 2 strict power profiling protocol runner.")
    parser.add_argument("--duration-s", type=float, default=120.0, help="Mission duration in seconds (use 21600 for 6h).")
    parser.add_argument("--log-hz", type=float, default=1.0, help="CSV logging rate in Hz.")
    parser.add_argument("--transit-speed", type=float, default=0.20, help="TRANSIT linear cmd_vel (m/s), thesis 0.20.")
    parser.add_argument("--obstacle-stop-m", type=float, default=0.35, help="If obstacle is nearer than this, stop and change path.")
    parser.add_argument("--clean-old", action="store_true", help="Delete previous exp2/table6_2/figure6_3 outputs in exp1_logs.")
    args = parser.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    if args.clean_old:
        for pat in ("exp2_summary*.txt", "table6_2_power_profile*.csv", "Figure6_3_Power_Profile*.png"):
            for p in OUT_DIR.glob(pat):
                if p.is_file():
                    p.unlink(missing_ok=True)

    tag = time.strftime("%Y%m%d_%H%M%S")
    csv_path = DATA_DIR / f"exp2_power_{tag}.csv"

    rclpy.init()
    node = Exp2PowerNode(transit_speed=args.transit_speed, obstacle_stop_m=args.obstacle_stop_m)
    try:
        # Warmup and topic verification.
        warm_start = time.time()
        while time.time() - warm_start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        hz = node.power_hz(window_s=5.0)
        if node.power_msg is None:
            raise RuntimeError("No /power_monitor data received. Check INA219 wiring/topic.")
        if hz < 8.0:
            raise RuntimeError(f"/power_monitor rate too low: {hz:.2f} Hz (expected ~10 Hz).")

        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_s", "mode", "voltage_V", "current_A", "power_W"])
            start = time.time()
            next_log = start
            cmd_dt = 0.1
            next_cmd = start
            while rclpy.ok():
                now = time.time()
                elapsed = now - start
                if elapsed >= args.duration_s:
                    break
                rclpy.spin_once(node, timeout_sec=0.01)
                mode = mode_at(elapsed)

                # Mode actions.
                if now >= next_cmd:
                    if mode == "TRANSIT":
                        # Safety behavior requested by user:
                        # if obstacle appears, stop then change path.
                        if node.lidar_fresh() and node.front_min_m <= args.obstacle_stop_m:
                            node.stop_robot()
                            node.turn_to_change_path()
                        else:
                            node.set_transit()
                    else:
                        node.stop_robot()
                    next_cmd = now + cmd_dt

                if now >= next_log:
                    if node.power_msg is not None:
                        v, a, p = node.power_msg
                        if math.isfinite(v) and math.isfinite(a) and math.isfinite(p):
                            w.writerow([round(elapsed, 2), mode, v, a, p])
                    next_log = now + max(0.2, 1.0 / args.log_hz)

        node.stop_robot()
        out = postprocess_exp2(csv_path, tag, OUT_DIR)
        print(f"EXP2_CSV,{csv_path}")
        print(f"TABLE_6_2_CSV,{out['table']}")
        print(f"SUMMARY_TXT,{out['summary']}")
        print(f"FIGURE_6_3,{out['figure']}")
        print(f"OVERALL_MEAN_W,{out['overall_mean']:.6f}")
        print(f"ESTIMATED_RUNTIME_H,{out['estimated_h']:.6f}")
        print(f"TARGET_MEAN_LE_1.2W,{'PASS' if out['overall_mean'] <= 1.2 else 'FAIL'}")
    finally:
        try:
            node.stop_robot()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
