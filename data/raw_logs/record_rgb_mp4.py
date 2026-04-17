#!/usr/bin/env python3
"""Record /dev/video0 to MP4 (V4L2 + OpenCV). Use when GStreamer jetson_camera fails."""
import argparse
import signal
import sys
import time

import cv2


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("output_mp4", help="Output file, e.g. /home/jetson/exp1_logs/exp4_rgb.mp4")
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("NO_CAMERA", file=sys.stderr)
        sys.exit(2)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    w = min(w, 1280)
    h = min(h, 720)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.output_mp4, fourcc, args.fps, (args.width, args.height))
    if not writer.isOpened():
        print("NO_WRITER", file=sys.stderr)
        cap.release()
        sys.exit(3)

    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    print(f"RECORDING_MP4,{args.output_mp4},{args.fps}fps", flush=True)
    t_next = time.time()
    period = 1.0 / max(args.fps, 1.0)
    while running:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        if frame.shape[1] != args.width or frame.shape[0] != args.height:
            frame = cv2.resize(frame, (args.width, args.height))
        writer.write(frame)
        t_next += period
        sleep = t_next - time.time()
        if sleep > 0:
            time.sleep(sleep)

    writer.release()
    cap.release()
    print("RECORDING_STOPPED", flush=True)


if __name__ == "__main__":
    main()
