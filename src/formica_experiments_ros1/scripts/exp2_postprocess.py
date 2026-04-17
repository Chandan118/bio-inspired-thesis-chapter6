#!/usr/bin/env python3
"""
exp2_postprocess.py - ROS 1 Version
====================================
Experiment 2 - Post-processing and analysis of power profiling data.

Run:
    python scripts/exp2_postprocess.py --input exp2_power_*.csv --output Table6_2_summary.csv
"""

import argparse
import csv
import glob
import os
import statistics

from formica_experiments.data_logger import CsvLogger


def load_csv(filepath):
    """Load CSV and return (fieldnames, rows)."""
    with open(filepath) as f:
        reader = csv.DictReader(f)
        return reader.fieldnames, list(reader)


def compute_summary(rows):
    """Compute statistics from power data rows."""
    voltages = [float(r["voltage_v"]) for r in rows if r["voltage_v"]]
    currents = [float(r["current_ma"]) for r in rows if r["current_ma"]]
    powers = [float(r["power_w"]) for r in rows if r["power_w"]]

    if not voltages:
        return {}

    return {
        "avg_voltage_v": statistics.mean(voltages),
        "min_voltage_v": min(voltages),
        "max_voltage_v": max(voltages),
        "avg_current_ma": statistics.mean(currents),
        "avg_power_w": statistics.mean(powers),
        "max_power_w": max(powers),
        "min_power_w": min(powers),
        "samples": len(rows),
    }


def main():
    parser = argparse.ArgumentParser(description="Post-process Exp2 power data")
    parser.add_argument("-i", "--input", nargs='+', required=True,
                        help="Input CSV file(s)")
    parser.add_argument("-o", "--output", required=True,
                        help="Output summary CSV")
    args = parser.parse_args()

    all_rows = []
    for pattern in args.input:
        for filepath in glob.glob(pattern):
            fieldnames, rows = load_csv(filepath)
            all_rows.extend(rows)

    summary = compute_summary(all_rows)

    out_path = os.path.expanduser(args.output)
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        for key, val in sorted(summary.items()):
            writer.writerow([key, "{:.6f}".format(val) if isinstance(val, float) else val])

    print("Summary written to:", out_path)
    print("Average Power: {:.3f} W".format(summary.get("avg_power_w", 0)))
    print("Total Samples:", summary.get("samples", 0))


if __name__ == "__main__":
    main()
