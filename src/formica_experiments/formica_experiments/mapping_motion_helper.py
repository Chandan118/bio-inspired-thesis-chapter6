#!/usr/bin/env python3
"""
Publish /cmd_vel while SLAM runs so the physical base explores the space.

Also publishes /cmd_vel_nav for stacks that relay Nav2 output to the base under that name.

Environment:
  MAPPING_MOTION_SEC   Total runtime in seconds (default: 50)
  MAPPING_MOTION_MAX_LIN   Max linear m/s (default: 0.18)
  MAPPING_MOTION_MAX_ANG   Max angular rad/s (default: 0.55)
  MAPPING_MOTION_SKIP_WAIT  Set to 1 to skip waiting for a subscriber
"""

from __future__ import annotations

import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

# Reliable keeps Nav2-style subscribers happy; RELIABLE pub + BEST_EFFORT sub (arduino_base) is compatible.
_CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def main(args: list[str] | None = None) -> None:
    duration = float(os.environ.get("MAPPING_MOTION_SEC", "50"))
    max_lin = float(os.environ.get("MAPPING_MOTION_MAX_LIN", "0.18"))
    max_ang = float(os.environ.get("MAPPING_MOTION_MAX_ANG", "0.55"))
    dt = 0.1

    segments: list[tuple[float, float, float]] = [
        (max_lin, 0.0, 5.0),
        (0.0, max_ang, 2.2),
        (max_lin, 0.0, 5.0),
        (0.0, -max_ang, 2.2),
        (max_lin, 0.0, 5.0),
        (0.0, 0.4, 2.0),
        (max_lin * 0.9, 0.0, 5.0),
    ]

    rclpy.init(args=args)
    node = rclpy.create_node("mapping_motion_helper")
    pub_cmd = node.create_publisher(Twist, "/cmd_vel", _CMD_VEL_QOS)
    pub_nav = node.create_publisher(Twist, "/cmd_vel_nav", _CMD_VEL_QOS)

    def _publish_twist(t: Twist) -> None:
        pub_cmd.publish(t)
        pub_nav.publish(t)

    skip_wait = os.environ.get("MAPPING_MOTION_SKIP_WAIT", "").strip() in ("1", "true", "yes")
    if not skip_wait:
        wait_deadline = time.time() + 25.0
        while time.time() < wait_deadline and rclpy.ok():
            n = pub_cmd.get_subscription_count() + pub_nav.get_subscription_count()
            if n > 0:
                node.get_logger().info(f"/cmd_vel (+nav) has {n} subscriber(s); starting motion.")
                break
            node.get_logger().info("Waiting for a cmd_vel subscriber (arduino_base / relay) ...")
            rclpy.spin_once(node, timeout_sec=0.25)
        else:
            node.get_logger().warn(
                "No cmd_vel subscriber after timeout — publishing anyway "
                "(check bringup / ROS_DOMAIN_ID)."
            )

    node.get_logger().info(
        f"Publishing cmd_vel for ~{duration:.0f}s (max lin={max_lin} m/s, max ang={max_ang} rad/s)"
    )

    deadline = time.time() + duration
    try:
        while time.time() < deadline and rclpy.ok():
            for lin, ang, seg_s in segments:
                if time.time() >= deadline:
                    break
                seg_end = min(time.time() + seg_s, deadline)
                while time.time() < seg_end and rclpy.ok():
                    t = Twist()
                    t.linear.x = float(lin)
                    t.angular.z = float(ang)
                    _publish_twist(t)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(dt)
    finally:
        z = Twist()
        _publish_twist(z)
        node.get_logger().info("Stopped cmd_vel (zero twist).")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
