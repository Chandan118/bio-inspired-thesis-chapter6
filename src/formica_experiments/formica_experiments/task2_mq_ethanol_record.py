#!/usr/bin/env python3
"""
Task 2 — MQ-2 / MQ-135 ethanol pre-activation data logger
==========================================================
Protocol (matches experiment text):
  - Optional baseline (clean air).
  - Place ethanol-soaked cotton near both sensors for a fixed exposure window (default 60 s).
  - Remove cotton, then record residual vapour decay (default 120 s) for use on robots
    without gas sensors (replay / model from this CSV).

Requires Arduino firmware sending: s,<mq2>,<mq135>,0,<ultrasonic_cm> on Serial at 115200.

Run (interactive):
  python3 formica_experiments/task2_mq_ethanol_record.py

Non-interactive short test:
  python3 formica_experiments/task2_mq_ethanol_record.py --auto --baseline 2 --exposure 3 --residual 3

Output:
  ~/formica_experiments/data/exp7_task2_mq_ethanol_<UTC>_<phase>.csv
"""

from __future__ import annotations

import argparse
import csv
import datetime
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    print('Install pyserial: pip install pyserial')
    sys.exit(1)

DATA_DIR = os.path.expanduser('~/formica_experiments/data')


def _ensure_data_dir() -> str:
    os.makedirs(DATA_DIR, exist_ok=True)
    return DATA_DIR

BAUD = 115200


def _candidate_ports(explicit: str | None) -> list[str]:
    if explicit:
        return [explicit]
    env = os.environ.get('ARDUINO_PORT', '').strip()
    if env:
        return [env]
    fixed = [
        '/dev/ttyCH341USB0',
        '/dev/ttyCH341USB1',
        '/dev/ttyACM0',
        '/dev/ttyACM1',
        '/dev/ttyUSB0',
        '/dev/ttyUSB1',
    ]
    extra = sorted(glob.glob('/dev/ttyCH341USB*'))
    seen: set[str] = set()
    out: list[str] = []
    for p in fixed + extra:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def open_serial(port_hint: str | None) -> tuple[serial.Serial, str]:
    last = None
    tried: list[str] = []
    for port in _candidate_ports(port_hint):
        if not os.path.exists(port):
            continue
        tried.append(port)
        try:
            ser = serial.Serial(port, BAUD, timeout=0.25)
            time.sleep(2.0)
            return ser, port
        except OSError as e:
            last = e
    if not tried:
        raise OSError('No /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH341* found.')
    raise OSError(last)


def parse_s_line(line: str) -> tuple[int, int, int] | None:
    line = line.strip()
    if not line.startswith('s,') or line.count(',') < 4:
        return None
    parts = line.split(',')
    try:
        mq2 = int(float(parts[1]))
        mq135 = int(float(parts[2]))
        dist = int(float(parts[4]))
        return mq2, mq135, dist
    except (ValueError, IndexError):
        return None


def sample_once(ser: serial.Serial) -> tuple[int, int, int, str] | None:
    ser.reset_input_buffer()
    ser.write(b'V')
    deadline = time.time() + 2.0
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode('utf-8', errors='ignore').strip()
        if 'Ready' in text and 'Motor' in text:
            continue
        if text.startswith('FW:'):
            continue
        parsed = parse_s_line(text)
        if parsed:
            mq2, mq135, dist = parsed
            return mq2, mq135, dist, text
    return None


def wait_enter(msg: str, auto: bool, auto_delay: float) -> None:
    print(msg)
    if auto:
        print(f'  (--auto: continuing in {auto_delay:.0f} s)')
        time.sleep(auto_delay)
    else:
        input('  Press ENTER to continue... ')


def log_phase(
    ser: serial.Serial,
    path: str,
    phase: str,
    t0: float,
    duration_s: float,
    interval_s: float,
) -> int:
    headers = [
        'wall_time_iso',
        'elapsed_s',
        'phase',
        'mq2_raw',
        'mq135_raw',
        'ultrasonic_cm',
        'raw_line',
    ]
    n = 0
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(headers)
        end = time.time() + duration_s
        while time.time() < end:
            row_t = time.time()
            got = sample_once(ser)
            iso = datetime.datetime.now(datetime.timezone.utc).isoformat()
            elapsed = row_t - t0
            if got:
                mq2, mq135, dist, raw = got
                w.writerow([iso, f'{elapsed:.3f}', phase, mq2, mq135, dist, raw])
                n += 1
            else:
                w.writerow([iso, f'{elapsed:.3f}', phase, '', '', '', 'TIMEOUT'])
            f.flush()
            slip = interval_s - (time.time() - row_t)
            if slip > 0:
                time.sleep(slip)
    return n


def main() -> None:
    p = argparse.ArgumentParser(description='Task 2 MQ ethanol pre-activation logger')
    p.add_argument('--port', default=None, help='Serial device e.g. /dev/ttyCH341USB1')
    p.add_argument('--baseline', type=float, default=10.0, help='Seconds of baseline before cotton')
    p.add_argument('--exposure', type=float, default=60.0, help='Seconds with ethanol cotton near sensors')
    p.add_argument('--residual', type=float, default=120.0, help='Seconds after removal (residual trail)')
    p.add_argument('--interval', type=float, default=0.25, help='Seconds between samples')
    p.add_argument('--auto', action='store_true', help='No Enter prompts; pause auto_delay between phases')
    p.add_argument('--auto-delay', type=float, default=3.0, help='Seconds between auto phases')
    args = p.parse_args()

    if not sys.stdin.isatty():
        args.auto = True

    _ensure_data_dir()
    ser, used_port = open_serial(args.port)
    print(f'Connected: {used_port} @ {BAUD}')

    t0 = time.time()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S')
    base = os.path.join(_ensure_data_dir(), f'exp7_task2_mq_ethanol_{stamp}')

    print('\nTask 2 — MQ-2 (A0) & MQ-135 (A1) + ultrasonic')
    print('  Exposure: ethanol-soaked cotton near BOTH sensors for the timed window.')
    print('  After exposure: remove cotton before residual capture.\n')

    # Baseline
    if args.baseline > 0:
        path = f'{base}_baseline.csv'
        print(f'Baseline logging ({args.baseline:.0f} s) -> {path}')
        n = log_phase(ser, path, 'baseline', t0, args.baseline, args.interval)
        print(f'  rows: {n}')

    # Ethanol near sensors
    wait_enter(
        'Place ethanol-soaked cotton near MQ-2 and MQ-135. Press ENTER when it is in place.',
        args.auto,
        args.auto_delay,
    )
    path = f'{base}_ethanol_60s.csv'
    print(f'Exposure logging ({args.exposure:.0f} s) -> {path}')
    n = log_phase(ser, path, 'ethanol_preactivation', t0, args.exposure, args.interval)
    print(f'  rows: {n}')

    wait_enter(
        '\n*** REMOVE cotton now (end of 60 s exposure). ***\n'
        'Press ENTER after removal to start residual-trail capture.',
        args.auto,
        args.auto_delay,
    )
    path = f'{base}_residual.csv'
    print(f'Residual logging ({args.residual:.0f} s) -> {path}')
    n = log_phase(ser, path, 'residual_after_removal', t0, args.residual, args.interval)
    print(f'  rows: {n}')

    # Combined file for convenience
    combined = f'{base}_all.csv'
    parts = [
        f'{base}_baseline.csv',
        f'{base}_ethanol_60s.csv',
        f'{base}_residual.csv',
    ]
    merged = 0
    with open(combined, 'w', newline='') as out:
        w = csv.writer(out)
        header_written = False
        for part in parts:
            if not os.path.isfile(part):
                continue
            with open(part, newline='') as inf:
                r = csv.reader(inf)
                h = next(r, None)
                if not h:
                    continue
                if not header_written:
                    w.writerow(h)
                    header_written = True
                for row in r:
                    w.writerow(row)
                    merged += 1
    print(f'\nDone. Combined CSV: {combined}  (data rows: {merged})')
    ser.close()


if __name__ == '__main__':
    main()
