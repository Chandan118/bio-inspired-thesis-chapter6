#!/usr/bin/env python3
import argparse
import csv
import os


def read_one(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[0] if rows else {}


def main():
    parser = argparse.ArgumentParser(description="Generate manuscript-ready markdown report.")
    parser.add_argument("--out", required=True, help="Output markdown file")
    args = parser.parse_args()

    base = os.path.dirname(args.out)
    field = read_one(os.path.join(base, "field_trials_summary.csv"))
    loc = read_one(os.path.join(base, "localization_summary.csv"))
    power = read_one(os.path.join(base, "power_summary.csv"))
    mem = read_one(os.path.join(base, "memory_summary.csv"))
    cmp = read_one(os.path.join(base, "baseline_summary.csv"))

    lines = []
    lines.append("# Appeal Experiment Results (Auto Summary)")
    lines.append("")
    lines.append("## 1) Structured Field Trials")
    lines.append(f"- Total trials: {field.get('total_trials', '[missing]')}")
    lines.append(f"- Success rate (%): {field.get('success_rate_percent', '[missing]')}")
    lines.append(f"- Avg duration (min): {field.get('avg_duration_min', '[missing]')}")
    lines.append(f"- By environment: {field.get('environment_breakdown', '[missing]')}")
    lines.append("")
    lines.append("## 2) Localization Accuracy")
    lines.append(f"- Samples: {loc.get('samples', '[missing]')}")
    lines.append(f"- RMSE (m): {loc.get('rmse_m', '[missing]')}")
    lines.append(f"- Mean absolute error (m): {loc.get('mean_abs_error_m', '[missing]')}")
    lines.append("")
    lines.append("## 3) Power Measurement")
    lines.append(f"- Avg power (W): {power.get('avg_power_w', '[missing]')}")
    lines.append(f"- Peak power (W): {power.get('peak_power_w', '[missing]')}")
    lines.append(f"- Energy (Wh): {power.get('energy_wh', '[missing]')}")
    lines.append("")
    lines.append("## 4) Compression Memory Test")
    lines.append(f"- Avg memory (GB): {mem.get('avg_memory_gb', '[missing]')}")
    lines.append(f"- Peak memory (GB): {mem.get('peak_memory_gb', '[missing]')}")
    lines.append(f"- Avg reduction (%): {mem.get('avg_reduction_percent', '[missing]')}")
    lines.append("")
    lines.append("## 5) Baseline Comparison")
    lines.append(
        f"- Success delta Loihi vs Jetson-only (%): {cmp.get('success_delta_percent_loihi_vs_jetson_only', '[missing]')}"
    )
    lines.append(
        f"- Collision delta Loihi vs Jetson-only (%): {cmp.get('collision_delta_percent_loihi_vs_jetson_only', '[missing]')}"
    )
    lines.append(
        f"- Time-to-goal delta Loihi vs Jetson-only (%): {cmp.get('ttg_delta_percent_loihi_vs_jetson_only', '[missing]')}"
    )
    lines.append("")
    lines.append("## Manuscript Insert Text")
    lines.append(
        "The revised manuscript includes additional structured field trials, localization benchmarking, direct power-meter measurements, compression memory profiling on Jetson, and baseline comparison under identical navigation tasks. Detailed protocols, sample counts, and summary metrics are reported in the updated Methods and Results sections."
    )
    lines.append("")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"Wrote report: {args.out}")


if __name__ == "__main__":
    main()
