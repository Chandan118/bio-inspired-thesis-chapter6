#!/usr/bin/env python3
"""
Detect stable /dev/serial/by-id paths for Formica dual-CP2102 setup.

- LIDAR: CP2102 device that answers the RPLidar GetDescriptor (0xA5 0x52).
- ARDUINO: the other Silicon Labs CP2102 symlink under /dev/serial/by-id (not the lidar).

Usage:
  python3 detect_formica_ports.py
  python3 detect_formica_ports.py --export-bash   # eval in shell: eval "$(python3 ... --export-bash)"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

def _cp2102_by_id_paths() -> list[Path]:
    base = Path("/dev/serial/by-id")
    if not base.is_dir():
        return []
    return sorted(base.glob("usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*"))


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--export-bash", action="store_true", help="Print export LIDAR_PORT=... ARDUINO_PORT=...")
    args = p.parse_args(argv)

    # Keep symlink paths (by-id), not resolve() → ttyUSB* , so USB order swaps do not break launch.
    devices = [str(x) for x in _cp2102_by_id_paths()]
    if not devices:
        print("No CP2102 devices under /dev/serial/by-id", file=sys.stderr)
        if args.export_bash:
            print("# No CP2102 by-id nodes found")
        return 1

    # Avoid active serial probing here: opening the lidar UART right before launch can destabilize startup.
    # Use stable by-id mapping on this Jetson: serial suffix "0001" is lidar, the other CP2102 is Arduino.
    lidar: str | None = None
    for dev in devices:
        if "0001" in dev:
            lidar = dev
            break
    if lidar is None and devices:
        lidar = devices[0]

    arduino: str | None = None
    for dev in devices:
        if lidar is None or dev != lidar:
            arduino = dev
            break

    if args.export_bash:
        if lidar:
            print(f"export LIDAR_PORT={lidar!s}")
        if arduino:
            print(f"export ARDUINO_PORT={arduino!s}")
        return 0

    print("LIDAR_PORT=" + (lidar or ""))
    print("ARDUINO_PORT=" + (arduino or ""))
    return 0 if (lidar and arduino) else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
