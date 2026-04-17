#!/usr/bin/env python3
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image


OUT_PREFIX = Path("/home/jetson/formica_map")


class MapGrabber(Node):
    def __init__(self):
        super().__init__("map_topic_saver")
        self.map_msg = None
        self.create_subscription(OccupancyGrid, "/map", self.cb, 10)

    def cb(self, msg: OccupancyGrid):
        self.map_msg = msg


def main():
    rclpy.init()
    node = MapGrabber()
    end = time.time() + 15.0
    while time.time() < end and node.map_msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.map_msg is None:
        print("NO_MAP_RECEIVED")
        node.destroy_node()
        rclpy.shutdown()
        return

    msg = node.map_msg
    w = int(msg.info.width)
    h = int(msg.info.height)
    data = np.array(msg.data, dtype=np.int16).reshape((h, w))

    # nav2 map encoding: 0 free -> 254, 100 occupied -> 0, -1 unknown -> 205
    img = np.zeros((h, w), dtype=np.uint8)
    img[data == -1] = 205
    img[data == 0] = 254
    occ_mask = data > 0
    img[occ_mask] = np.clip(254 - (data[occ_mask] * 254 / 100.0), 0, 254).astype(np.uint8)
    img = np.flipud(img)

    pgm_path = OUT_PREFIX.with_suffix(".pgm")
    yaml_path = OUT_PREFIX.with_suffix(".yaml")
    Image.fromarray(img, mode="L").save(pgm_path)

    origin = msg.info.origin.position
    yaw = 0.0
    yaml_text = (
        f"image: {pgm_path.name}\n"
        "mode: trinary\n"
        f"resolution: {msg.info.resolution}\n"
        f"origin: [{origin.x}, {origin.y}, {yaw}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    yaml_path.write_text(yaml_text)

    print(f"MAP_PGM,{pgm_path}")
    print(f"MAP_YAML,{yaml_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
