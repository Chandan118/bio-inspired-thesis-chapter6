#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from std_msgs.msg import Float32


@dataclass
class Health:
    last_scan: float = 0.0
    last_imu: float = 0.0
    last_cam: float = 0.0
    last_ultra: float = 0.0


class SmoothRunner(Node):
    def __init__(self):
        super().__init__("smooth_runner_30")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Imu, "/data", self.imu_cb, 50)
        self.create_subscription(Image, "/rgb/image_raw", self.cam_cb, 10)
        self.create_subscription(Float32, "/sensor/distance", self.ultra_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)

        self.h = Health()
        self.front_min = float("inf")
        self.ultra_cm = float("inf")
        self.steps = 0
        self.forward_steps = 0
        self.stops = 0
        self.turns = 0
        self.max_forward = 0.0
        self.min_linear_cmd = 1.0
        self.start_t = time.time()
        self.front_history = []
        self.ultra_history = []
        self.ultra_window = []
        self.ultra_filtered_cm = float("inf")
        self.ultra_low_count = 0
        self.lidar_fallback_steps = 0
        self.odom_xy = []
        self.last_odom_t = 0.0
        self.speed_scale_used = 0.0
        self.auto_ramp_activations = 0
        self.direction_sign = 1.0
        self.direction_checked = False
        self.direction_reason = "not_checked"

    def scan_cb(self, msg: LaserScan):
        self.h.last_scan = time.time()
        vals = []
        if msg.angle_increment == 0.0 or not msg.ranges:
            return
        # +/- 20 degrees around front
        for deg in range(-20, 21):
            ang = math.radians(deg)
            idx = int(round((ang - msg.angle_min) / msg.angle_increment))
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    vals.append(r)
        if vals:
            self.front_min = min(vals)

    def imu_cb(self, _msg: Imu):
        self.h.last_imu = time.time()

    def cam_cb(self, _msg: Image):
        self.h.last_cam = time.time()

    def ultra_cb(self, msg: Float32):
        self.h.last_ultra = time.time()
        self.ultra_cm = float(msg.data)
        # Clamp impossible spikes and smooth with median window.
        if math.isfinite(self.ultra_cm):
            v = self.ultra_cm
            if v < 2.0:
                v = 2.0
            if v > 400.0:
                v = 400.0
            self.ultra_window.append(v)
            if len(self.ultra_window) > 7:
                self.ultra_window.pop(0)
            self.ultra_filtered_cm = self._median(self.ultra_window)

    def odom_cb(self, msg: Odometry):
        self.last_odom_t = time.time()
        self.odom_xy.append((float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), self.last_odom_t))
        if len(self.odom_xy) > 600:
            self.odom_xy.pop(0)

    def fresh(self, t):
        now = time.time()
        return (
            now - self.h.last_scan < t and
            now - self.h.last_imu < t and
            now - self.h.last_cam < t and
            now - self.h.last_ultra < t
        )

    def core_fresh(self, t):
        now = time.time()
        return (
            now - self.h.last_imu < t and
            now - self.h.last_cam < t and
            now - self.h.last_ultra < t
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _median(self, vals):
        vals = [v for v in vals if math.isfinite(v)]
        if not vals:
            return float("nan")
        vals = sorted(vals)
        n = len(vals)
        if n % 2 == 1:
            return vals[n // 2]
        return 0.5 * (vals[n // 2 - 1] + vals[n // 2])

    def _sample_front_ultra(self, seconds=1.0):
        end_t = time.time() + seconds
        fronts = []
        ultras = []
        while time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.02)
            if math.isfinite(self.front_min):
                fronts.append(self.front_min)
            if math.isfinite(self.ultra_cm):
                ultras.append(self.ultra_cm)
        return self._median(fronts), self._median(ultras)

    def auto_detect_direction(self):
        # Needs at least ultrasonic freshness; LiDAR helps confidence.
        if (time.time() - self.h.last_ultra) > 1.5:
            self.direction_checked = True
            self.direction_reason = "ultrasonic_not_fresh"
            return

        # Baseline
        base_front, base_ultra = self._sample_front_ultra(1.0)
        # Probe command: nominal forward pulse
        probe = Twist()
        probe.linear.x = 0.06
        probe.angular.z = 0.0
        probe_end = time.time() + 2.0
        while time.time() < probe_end:
            self.cmd_pub.publish(probe)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.stop()
        time.sleep(0.3)
        end_front, end_ultra = self._sample_front_ultra(0.8)

        du = end_ultra - base_ultra if math.isfinite(end_ultra) and math.isfinite(base_ultra) else float("nan")
        df = end_front - base_front if math.isfinite(end_front) and math.isfinite(base_front) else float("nan")

        # If front distance increased clearly after "forward" pulse, likely motor polarity is reversed.
        if (math.isfinite(du) and du > 2.0) or (math.isfinite(df) and df > 0.05):
            self.direction_sign = -1.0
            self.direction_reason = f"reversed_detected_du={du:.2f}_df={df:.3f}"
            self.get_logger().warn("Direction check: detected reverse motion on forward pulse; flipping sign.")
        else:
            self.direction_sign = 1.0
            self.direction_reason = f"normal_du={du:.2f}_df={df:.3f}"
            self.get_logger().info("Direction check: forward pulse looks correct.")

        self.direction_checked = True

    def _moved_recently(self, window_s=8.0, min_dist_m=0.05):
        if len(self.odom_xy) < 2:
            return False
        now = time.time()
        pts = [p for p in self.odom_xy if now - p[2] <= window_s]
        if len(pts) < 2:
            return False
        x0, y0, _ = pts[0]
        x1, y1, _ = pts[-1]
        d = math.hypot(x1 - x0, y1 - y0)
        return d >= min_dist_m

    def run(
        self,
        duration_s=120.0,
        speed_scale=0.30,
        require_lidar=False,
        auto_ramp=False,
        auto_direction_check=False
    ):
        self.get_logger().info(
            f"Starting smooth run: {speed_scale*100:.0f}% speed, forward-only obstacle-safe mode."
        )
        if require_lidar:
            self.get_logger().info("LiDAR-required mode enabled: robot stops when LiDAR is stale.")
        if auto_ramp:
            self.get_logger().info(
                "Auto-ramp enabled: if no movement for 10s, increase speed by 2% until movement starts."
            )
        if auto_direction_check:
            self.get_logger().info("Auto direction check enabled (ultrasonic/LiDAR trend probe).")
            # Short settle before probing.
            settle_end = time.time() + 1.0
            while time.time() < settle_end:
                rclpy.spin_once(self, timeout_sec=0.02)
            self.auto_detect_direction()
        current_scale = speed_scale
        last_ramp_check = time.time()
        moved_latched = False
        rate_hz = 15.0
        dt = 1.0 / rate_hz
        while time.time() - self.start_t < duration_s:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.steps += 1

            cmd = Twist()
            # speed scale of 0.20 m/s nominal.
            v_fwd = max(0.0, min(0.20, 0.20 * current_scale))
            self.max_forward = max(self.max_forward, v_fwd)
            self.speed_scale_used = current_scale

            if auto_ramp and not moved_latched and (time.time() - last_ramp_check) >= 10.0:
                if self._moved_recently(window_s=8.0, min_dist_m=0.05):
                    moved_latched = True
                else:
                    old_scale = current_scale
                    current_scale = min(1.0, current_scale + 0.02)
                    self.auto_ramp_activations += 1
                    self.get_logger().warn(
                        f"No odom movement detected: increasing speed scale {old_scale:.2f} -> {current_scale:.2f}"
                    )
                    if current_scale >= 1.0:
                        moved_latched = True
                last_ramp_check = time.time()

            if not self.core_fresh(1.5):
                self.stops += 1
                self.min_linear_cmd = min(self.min_linear_cmd, 0.0)
                self.stop()
                time.sleep(dt)
                continue

            # Forward-only safety policy:
            # - Very near obstacle: stop + turn in place (no reverse).
            # - Near obstacle: keep small forward crawl + turn arc.
            lidar_ok = (time.time() - self.h.last_scan) < 1.5 and math.isfinite(self.front_min)
            if require_lidar and not lidar_ok:
                self.stops += 1
                self.min_linear_cmd = min(self.min_linear_cmd, 0.0)
                self.stop()
                time.sleep(dt)
                continue
            # Use filtered ultrasonic value to avoid false turn loops.
            ultra_safe = self.ultra_filtered_cm if math.isfinite(self.ultra_filtered_cm) else self.ultra_cm
            if ultra_safe < 45.0:
                self.ultra_low_count += 1
            else:
                self.ultra_low_count = max(0, self.ultra_low_count - 1)
            confirmed_ultra_near = self.ultra_low_count >= 3
            lidar_clear = math.isfinite(self.front_min) and self.front_min > 0.70
            if lidar_clear:
                # If LiDAR clearly sees open space, suppress ultrasonic-only near flags.
                confirmed_ultra_near = False

            if not lidar_ok:
                self.lidar_fallback_steps += 1
                # Ultrasonic-only fallback; stay cautious.
                if ultra_safe < 30.0 and confirmed_ultra_near:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.60
                    self.turns += 1
                elif ultra_safe < 50.0 and confirmed_ultra_near:
                    cmd.linear.x = max(0.02, 0.25 * v_fwd)
                    cmd.angular.z = 0.35
                    self.turns += 1
                else:
                    cmd.linear.x = min(v_fwd, 0.045)
                    cmd.angular.z = 0.0
                    self.forward_steps += 1
            elif self.front_min < 0.28 or (ultra_safe < 28.0 and confirmed_ultra_near):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.60
                self.turns += 1
            elif self.front_min < 0.50 or (ultra_safe < 45.0 and confirmed_ultra_near):
                cmd.linear.x = max(0.02, 0.35 * v_fwd)
                cmd.angular.z = 0.35
                self.turns += 1
            else:
                cmd.linear.x = v_fwd
                cmd.angular.z = 0.0
                self.forward_steps += 1

            if cmd.linear.x == 0.0 and cmd.angular.z == 0.0:
                self.stops += 1
            self.min_linear_cmd = min(self.min_linear_cmd, cmd.linear.x)
            # Enforce no reverse motion.
            cmd.linear.x = cmd.linear.x * self.direction_sign
            self.cmd_pub.publish(cmd)
            if math.isfinite(self.front_min):
                self.front_history.append(self.front_min)
            if math.isfinite(self.ultra_cm):
                self.ultra_history.append(self.ultra_cm)
            time.sleep(dt)

        self.stop()
        elapsed = time.time() - self.start_t
        print(f"elapsed_s,{elapsed:.2f}")
        print(f"steps,{self.steps}")
        print(f"turn_events,{self.turns}")
        print(f"stop_events,{self.stops}")
        print(f"forward_ratio,{(self.forward_steps / self.steps) if self.steps else 0.0:.4f}")
        print(f"lidar_fallback_ratio,{(self.lidar_fallback_steps / self.steps) if self.steps else 0.0:.4f}")
        print(f"max_forward_mps,{self.max_forward:.3f}")
        print(f"min_linear_cmd_mps,{self.min_linear_cmd:.3f}")
        print(f"final_speed_scale,{self.speed_scale_used:.2f}")
        print(f"auto_ramp_steps,{self.auto_ramp_activations}")
        print(f"direction_checked,{self.direction_checked}")
        print(f"direction_sign,{self.direction_sign:.1f}")
        print(f"direction_reason,{self.direction_reason}")
        print(f"last_front_min_m,{self.front_min:.3f}")
        print(f"last_ultra_cm,{self.ultra_cm:.1f}")
        print(f"last_ultra_filtered_cm,{self.ultra_filtered_cm:.1f}")
        print(f"ultra_low_count,{self.ultra_low_count}")
        if self.front_history:
            print(f"front_min_m,{min(self.front_history):.3f}")
            print(f"front_mean_m,{sum(self.front_history)/len(self.front_history):.3f}")
        if self.ultra_history:
            print(f"ultra_min_cm,{min(self.ultra_history):.1f}")
            print(f"ultra_mean_cm,{sum(self.ultra_history)/len(self.ultra_history):.1f}")
        print(f"imu_fresh,{time.time() - self.h.last_imu < 1.5}")
        print(f"camera_fresh,{time.time() - self.h.last_cam < 1.5}")
        print(f"lidar_fresh,{time.time() - self.h.last_scan < 1.5}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0, help="Run duration seconds")
    parser.add_argument("--speed-scale", type=float, default=0.30, help="Scale of nominal 0.20 m/s")
    parser.add_argument("--require-lidar", action="store_true", help="Stop unless LiDAR is fresh")
    parser.add_argument("--auto-ramp", action="store_true", help="Raise speed by 2% every 10s until movement detected")
    parser.add_argument("--auto-direction-check", action="store_true", help="Detect and auto-flip motor direction sign")
    args = parser.parse_args()
    rclpy.init()
    node = SmoothRunner()
    try:
        node.run(
            args.duration,
            args.speed_scale,
            args.require_lidar,
            args.auto_ramp,
            args.auto_direction_check
        )
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
