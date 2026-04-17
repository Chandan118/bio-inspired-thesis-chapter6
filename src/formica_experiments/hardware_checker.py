#!/usr/bin/env python3
"""
Quick serial health check for Formica stack (RPLidar + Arduino base on CP2102 / CH341 / ACM).

Older logic assumed ttyUSB0=lidar and ttyUSB1=IMU; many rigs use two CP2102 bridges
(lidar + Arduino). We probe each port in order: RPLidar descriptor → Arduino CSV → IMU-like binary.
"""

from __future__ import annotations

import argparse
import sys
import time

import serial
import serial.tools.list_ports


def scan_ports() -> list[str]:
    ports = serial.tools.list_ports.comports()
    print(f"Found {len(ports)} serial ports:")
    for p in ports:
        print(f"  {p.device} - {p.description} (HWID: {p.hwid})")
    return [p.device for p in ports]


def probe_rplidar(port: str, *, read_timeout: float = 0.35) -> bool:
    print(f"\n--- RPLidar probe on {port} ---")
    try:
        ser = serial.Serial(port, 115200, timeout=read_timeout)
        ser.reset_input_buffer()
        ser.write(b"\xA5\x52")
        resp = ser.read(27)
        ser.close()
        if resp and len(resp) >= 2 and resp[0] == 0xA5 and resp[1] == 0x5A:
            print(f"  OK: RPLidar descriptor response on {port}")
            return True
        print("  No RPLidar descriptor header.")
    except Exception as e:
        print(f"  ERROR: {e}")
    return False


def probe_arduino_csv(port: str, *, boot_wait: float, io_timeout: float) -> bool:
    print(f"\n--- Arduino / base (CSV) probe on {port} ---")
    try:
        ser = serial.Serial(port, 115200, timeout=io_timeout)
        time.sleep(boot_wait)
        ser.reset_input_buffer()
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        ser.close()
        if "," in line and len(line) < 500:
            print(f"  OK: CSV-like line from {port}: {line[:120]}{'...' if len(line) > 120 else ''}")
            return True
        print(f"  No CSV line (got {len(line)} chars): {line[:80]!r}")
    except Exception as e:
        print(f"  ERROR: {e}")
    return False


def probe_imu_binary(port: str, *, wait_s: float, io_timeout: float) -> bool:
    print(f"\n--- IMU-like binary probe on {port} ---")
    try:
        ser = serial.Serial(port, 115200, timeout=io_timeout)
        time.sleep(wait_s)
        data = ser.read(128)
        ser.close()
        if data and any(b > 0x7F for b in data):
            print(f"  OK: High-byte binary stream on {port} ({len(data)} bytes read)")
            return True
        if len(data) >= 8 and max(data) > 0:
            print(f"  Activity on {port} but not clearly IMU binary (max byte {max(data)})")
            return False
        print("  No useful data.")
    except Exception as e:
        print(f"  ERROR: {e}")
    return False


def classify_port(port: str, *, quick: bool) -> str:
    boot = 0.8 if quick else 2.0
    io_t = 0.25 if quick else 0.5
    lid_t = 0.25 if quick else 0.35
    imu_wait = 0.4 if quick else 1.0

    if probe_rplidar(port, read_timeout=lid_t):
        return "rplidar"
    if probe_arduino_csv(port, boot_wait=boot, io_timeout=io_t):
        return "arduino_csv"
    if probe_imu_binary(port, wait_s=imu_wait, io_timeout=io_t):
        return "imu_guess"
    return "unknown"


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="Formica USB serial health (lidar / base / IMU).")
    p.add_argument(
        "--quick",
        action="store_true",
        help="Shorter waits (~3–5s total for 2 ports) for a fast sanity check.",
    )
    args = p.parse_args(argv)

    devices = scan_ports()
    if not devices:
        print("\nNo serial devices — check USB cables / power.")
        return 1

    roles: dict[str, str] = {}
    for d in devices:
        roles[d] = classify_port(d, quick=args.quick)

    print("\n========== Summary ==========")
    for d, role in roles.items():
        print(f"  {d}: {role}")
    lidar_ok = any(r == "rplidar" for r in roles.values())
    base_ok = any(r == "arduino_csv" for r in roles.values())
    if lidar_ok and base_ok:
        print("\nOverall: RPLidar + Arduino/base both detected — good for mapping / nav bringup.")
        return 0
    if lidar_ok:
        print("\nOverall: RPLidar OK; Arduino/base not confirmed on CSV — check motor firmware port / baud.")
        return 2
    if base_ok:
        print("\nOverall: Arduino OK; RPLidar not found — check lidar USB / power.")
        return 3
    print("\nOverall: Could not confirm lidar or Arduino — inspect wiring and port order.")
    return 4


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
