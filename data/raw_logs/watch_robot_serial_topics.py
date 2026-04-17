#!/usr/bin/env python3
"""Single ROS 2 node: subscribe to Arduino topics and print one line per message.

Avoids running multiple `ros2 topic echo` at once (they trigger Fast DDS SHM port
conflicts and noisy RTPS_TRANSPORT_SHM errors on the same host).

All-zero /line_sensors: prints one explanation, then stays quiet until values
change. Optional: WATCH_ROBOT_SERIAL_HEARTBEAT_SEC=30 to repeat [line] pings.
"""
from __future__ import annotations

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray

# Match typical robot publishers (reliable, small queue).
_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


class WatchRobotTopics(Node):
    def __init__(self) -> None:
        super().__init__("watch_robot_serial_topics")
        # None until first placeholder explanation (avoids hb if last_ts were 0.0).
        self._last_zero_placeholder_hb_ts: float | None = None
        self._shown_zero_explanation = False
        self.create_subscription(Float32MultiArray, "/line_sensors", self._line, _QOS)
        self.create_subscription(Float32, "sensor/distance", self._dist, _QOS)
        self.create_subscription(Float32, "/gas_sensor", self._gas, _QOS)

    def _line(self, msg: Float32MultiArray) -> None:
        vals = ",".join(f"{v:.3f}" for v in msg.data)
        data = list(msg.data)
        if data and all(v == 0.0 for v in data):
            if not self._shown_zero_explanation:
                self._shown_zero_explanation = True
                # Next heartbeat (if enabled) is allowed only after hb seconds from here.
                self._last_zero_placeholder_hb_ts = time.monotonic()
                print(
                    f"[line] {vals}\n"
                    "  → Placeholder from arduino_base until serial sends TCRT "
                    "(s,...,s0,s1,s2,s3) or legacy dist,s0,s1,s2,s3.\n"
                    "  → [dist]/[gas] appear when s,mq2,mq135,co2,dist is parsed; "
                    "if missing, check USB and arduino_base logs (RAW SERIAL).",
                    flush=True,
                )
                return
            hb = float(os.environ.get("WATCH_ROBOT_SERIAL_HEARTBEAT_SEC", "0") or "0")
            if hb <= 0:
                return
            now = time.monotonic()
            if self._last_zero_placeholder_hb_ts is None:
                self._last_zero_placeholder_hb_ts = now
                return
            if now - self._last_zero_placeholder_hb_ts < hb:
                return
            self._last_zero_placeholder_hb_ts = now
            print(f"[line] {vals}  (still placeholder)", flush=True)
            return
        self._shown_zero_explanation = False
        print(f"[line] {vals}", flush=True)

    def _dist(self, msg: Float32) -> None:
        print(f"[dist] {msg.data:.3f}", flush=True)

    def _gas(self, msg: Float32) -> None:
        print(f"[gas] {msg.data:.3f}", flush=True)


def main() -> None:
    rclpy.init()
    try:
        node = WatchRobotTopics()
        print(
            "[watch_robot_serial] Ctrl+C to stop. "
            "Topics: /line_sensors, sensor/distance, /gas_sensor",
            flush=True,
        )
        hb = float(os.environ.get("WATCH_ROBOT_SERIAL_HEARTBEAT_SEC", "0") or "0")
        if hb > 0:
            print(
                f"[watch_robot_serial] Heartbeat every {hb:g}s while line stays "
                "all-zero (placeholder).",
                flush=True,
            )
        else:
            print(
                "[watch_robot_serial] All-zero [line]: one explanation, then quiet "
                "until values change (optional: WATCH_ROBOT_SERIAL_HEARTBEAT_SEC=30).",
                flush=True,
            )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
