#!/usr/bin/env python3
"""
hardware_checker.py - ROS 1 Version
=====================================
Quick serial health check for Formica stack (RPLidar + Arduino base).

Run:
    rosrun formica_experiments hardware_checker.py
    rosrun formica_experiments hardware_checker.py --quick
"""

from __future__ import print_function

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Run: pip install pyserial")


def scan_ports():
    if not SERIAL_AVAILABLE:
        return []
    ports = serial.tools.list_ports.comports()
    print("Found {0} serial ports:".format(len(ports)))
    for p in ports:
        print("  {0} - {1} (HWID: {2})".format(p.device, p.description, p.hwid))
    return [p.device for p in ports]


def probe_rplidar(port, read_timeout=0.35):
    print("\n--- RPLidar probe on {0} ---".format(port))
    try:
        ser = serial.Serial(port, 115200, timeout=read_timeout)
        ser.reset_input_buffer()
        ser.write(b"\xA5\x52")
        resp = ser.read(27)
        ser.close()
        if resp and len(resp) >= 2 and resp[0] == 0xA5 and resp[1] == 0x5A:
            print("  OK: RPLidar descriptor response on {0}".format(port))
            return True
        print("  No RPLidar descriptor header.")
    except Exception as e:
        print("  ERROR: {0}".format(e))
    return False


def probe_arduino_csv(port, boot_wait=2.0, io_timeout=5.0):
    print("\n--- Arduino / base (CSV) probe on {0} ---".format(port))
    try:
        ser = serial.Serial(port, 115200, timeout=io_timeout)
        time.sleep(boot_wait)
        ser.reset_input_buffer()
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        ser.close()
        if "," in line and len(line) < 500:
            print("  OK: CSV-like line from {0}: {1}".format(port, line[:120]))
            return True
        print("  No CSV line (got {0} chars)".format(len(line)))
    except Exception as e:
        print("  ERROR: {0}".format(e))
    return False


def classify_port(port, quick=False):
    if probe_rplidar(port):
        return "RPLidar"
    if probe_arduino_csv(port, boot_wait=0.5 if quick else 2.0):
        return "Arduino/Base"
    return "Unknown"


def main():
    parser = argparse.ArgumentParser(description="Formica hardware checker")
    parser.add_argument("--quick", action="store_true", help="Skip long waits")
    parser.add_argument("--port", type=str, help="Check specific port only")
    args = parser.parse_args()

    print("=" * 50)
    print("FormicaBot Hardware Checker (ROS 1 Noetic)")
    print("=" * 50)

    if not SERIAL_AVAILABLE:
        print("\nERROR: pyserial not installed.")
        print("Install with: pip install pyserial")
        sys.exit(1)

    if args.port:
        result = classify_port(args.port, quick=args.quick)
        print("\nResult: {0}".format(result))
    else:
        ports = scan_ports()
        print("\n--- Scanning all ports ---")
        for p in ports:
            result = classify_port(p, quick=args.quick)
            print("  {0} -> {1}".format(p, result))

    print("\n--- ROS Topic Check ---")
    import rospy
    rospy.init_node("hardware_checker", anonymous=True)
    
    topics = {
        "/scan": "RPLIDAR",
        "/imu/data": "IMU",
        "/odom": "Odometry",
        "/cmd_vel": "Velocity Command",
    }
    
    for topic, desc in topics.items():
        try:
            msg_class = rospy.wait_for_message(topic, rospy.AnyMsg, timeout=2.0)
            print("  {0} ({1}): OK".format(topic, desc))
        except rospy.ROSException:
            print("  {0} ({1}): NOT AVAILABLE".format(topic, desc))


if __name__ == "__main__":
    main()
