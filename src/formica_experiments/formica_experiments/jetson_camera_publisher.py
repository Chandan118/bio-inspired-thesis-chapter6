#!/usr/bin/env python3
"""
Publish USB (or V4L2) camera frames to ROS as sensor_msgs/Image.

Most USB webcams on Jetson expose MJPEG only; a raw video/x-raw pipeline often fails
with v4l2src "Internal data stream error". Default is therefore MJPEG → jpegdec → BGR.
"""
from __future__ import annotations

import time
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# Legacy raw pipeline (fails on many MJPEG-only USB cameras)
USB_GSTREAMER_RAW_PIPELINE = (
    'v4l2src device=/dev/video0 ! '
    'video/x-raw, width=1280, height=720, framerate=30/1 ! '
    'videoconvert ! '
    'video/x-raw, format=BGR ! appsink drop=true'
)


def _mjpeg_gstreamer_pipeline(device: str, width: int, height: int, fps: int) -> str:
    return (
        f'v4l2src device={device} ! '
        f'image/jpeg,width={width},height={height},framerate={fps}/1 ! '
        f'jpegdec ! videoconvert ! video/x-raw,format=BGR ! '
        f'appsink drop=true sync=false'
    )


class JetsonCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('jetson_camera_publisher')

        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('use_mjpeg_gstreamer', True)
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('image_fps', 30)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('topic_name', 'camera/image_raw')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('capture_backend', 'gstreamer')
        # gstreamer | v4l2 — v4l2 uses cv2.VideoCapture(device, CAP_V4L2) + MJPG fourcc

        topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        frame_rate = float(self.get_parameter('frame_rate').value)

        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.bridge = CvBridge()

        self.cap: Optional[cv2.VideoCapture] = None
        self._warn_skip_until = 0.0
        self._opened_with: str = ''

        custom = str(self.get_parameter('gstreamer_pipeline').value).strip()
        use_mjpeg = bool(self.get_parameter('use_mjpeg_gstreamer').value)
        device = str(self.get_parameter('camera_device').value)
        w = int(self.get_parameter('image_width').value)
        h = int(self.get_parameter('image_height').value)
        fps_i = int(self.get_parameter('image_fps').value)
        backend = str(self.get_parameter('capture_backend').value).strip().lower()

        if backend == 'v4l2':
            self.cap = self._open_v4l2_mjpeg(device, w, h, fps_i)
        elif custom:
            self.cap = self._try_gstreamer(custom, 'custom pipeline')
        elif use_mjpeg:
            pipe = _mjpeg_gstreamer_pipeline(device, w, h, fps_i)
            self.cap = self._try_gstreamer(pipe, f'MJPEG {device} {w}x{h}@{fps_i}')
            if not self.cap or not self.cap.isOpened():
                self.get_logger().warn(
                    f'MJPEG GStreamer failed; trying 640x480@{fps_i} ...'
                )
                pipe2 = _mjpeg_gstreamer_pipeline(device, 640, 480, fps_i)
                self.cap = self._try_gstreamer(pipe2, f'MJPEG fallback 640x480')
        else:
            self.cap = self._try_gstreamer(USB_GSTREAMER_RAW_PIPELINE, 'legacy raw')

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error(
                'Could not open camera. Checks: (1) camera plugged in, '
                '(2) try -p camera_device:=/dev/video0 or /dev/video1, '
                '(3) v4l2-ctl -d /dev/video0 --list-formats-ext, '
                '(4) -p capture_backend:=v4l2'
            )
            self._timer = None
            return

        timer_period = 1.0 / max(frame_rate, 1.0)
        self._timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'Jetson camera publisher started ({self._opened_with}). topic={topic_name!r}'
        )

    def _try_gstreamer(self, pipeline: str, label: str) -> Optional[cv2.VideoCapture]:
        self.get_logger().info(f'Trying GStreamer ({label}): {pipeline}')
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            self._opened_with = label
            return cap
        cap.release()
        return None

    def _open_v4l2_mjpeg(
        self, device: str, width: int, height: int, fps: int
    ) -> Optional[cv2.VideoCapture]:
        self.get_logger().info(f'Trying OpenCV V4L2 + MJPG on {device}')
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return cap
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
        except Exception as exc:
            self.get_logger().warn(f'V4L2 property set failed: {exc}')
        if cap.isOpened():
            self._opened_with = f'V4L2 MJPG {device}'
        return cap

    def timer_callback(self) -> None:
        if self.cap is None or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if ret:
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = self.frame_id
            self.publisher_.publish(ros_image_msg)
            return
        now = time.monotonic()
        if now >= self._warn_skip_until:
            self._warn_skip_until = now + 5.0
            self.get_logger().warn(
                'Could not read frame (camera busy or disconnected). '
                'If this persists, try another camera_device or capture_backend:=v4l2.'
            )

    def on_shutdown(self) -> None:
        self.get_logger().info('Shutting down, releasing camera.')
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    camera_publisher = JetsonCameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.on_shutdown()
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
