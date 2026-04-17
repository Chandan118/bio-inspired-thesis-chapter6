#!/usr/bin/env python3
"""
exp7_postprocess.py - ROS 1 Version
====================================
Experiment 7 - Post-processing and analysis of pheromone trail data.

Run:
    python scripts/exp7_postprocess.py --input exp7_pheromone_*.csv --output Table6_6_7_summary.csv
"""

import argparse
import csv
import glob
import os
import statistics


def load_csv(filepath):
    """Load CSV and return (fieldnames, rows)."""
    with open(filepath) as f:
        reader = csv.DictReader(f)
        return reader.fieldnames, list(reader)


def compute_trail_statistics(rows):
    """Compute statistics from pheromone trail data."""
    ethanol_values = []
    yaw_values = []

    for r in rows:
        try:
            ethanol_values.append(float(r.get("ethanol_ppm", 0)))
            yaw_values.append(float(r.get("yaw_rad", 0)))
        except (ValueError, TypeError):
            continue

    if not ethanol_values:
        return {}

    return {
        "avg_ethanol_ppm": statistics.mean(ethanol_values),
        "max_ethanol_ppm": max(ethanol_values),
        "min_ethanol_ppm": min(ethanol_values),
        "std_ethanol_ppm": statistics.stdev(ethanol_values) if len(ethanol_values) > 1 else 0.0,
        "avg_yaw_rad": statistics.mean(yaw_values),
        "samples": len(ethanol_values),
    }


def main():
    parser = argparse.ArgumentParser(description="Post-process Exp7 trail data")
    parser.add_argument("-i", "--input", nargs='+', required=True,
                        help="Input CSV file(s)")
    parser.add_argument("-o", "--output", required=True,
                        help="Output summary CSV")
    args = parser.parse_args()

    all_rows = []
    for pattern in args.input:
        for filepath in glob.glob(pattern):
            _, rows = load_csv(filepath)
            all_rows.extend(rows)

    summary = compute_trail_statistics(all_rows)

    out_path = os.path.expanduser(args.output)
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    with open(out_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        for key, val in sorted(summary.items()):
            writer.writerow([key, "{:.6f}".format(val) if isinstance(val, float) else val])

    print("Summary written to:", out_path)
    print("Avg Ethanol: {:.1f} ppm".format(summary.get("avg_ethanol_ppm", 0)))


if __name__ == "__main__":
    main()
