#!/usr/bin/env python3
import argparse
from collections import defaultdict

from common import read_csv_rows, safe_float, summarize, write_single_row_csv


def main():
    parser = argparse.ArgumentParser(description="Compare baseline modes on same tasks.")
    parser.add_argument("--input", required=True, help="Baseline comparison CSV")
    parser.add_argument("--output", required=True, help="Output summary CSV")
    args = parser.parse_args()

    rows = read_csv_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in input CSV.")

    groups = defaultdict(list)
    for r in rows:
        mode = r.get("mode", "unknown").strip().lower()
        groups[mode].append(r)

    metrics = ["success", "collision_count", "time_to_goal_sec"]
    summary = {}
    for mode, grows in groups.items():
        summary[mode] = {}
        for m in metrics:
            vals = [safe_float(g.get(m)) for g in grows]
            mu, sd = summarize(vals)
            summary[mode][f"{m}_mean"] = mu
            summary[mode][f"{m}_std"] = sd

    # Expected mode labels: "jetson_only", "jetson_loihi"
    jo = summary.get("jetson_only", {})
    jl = summary.get("jetson_loihi", {})

    def delta(a, b):
        if a == 0:
            return 0.0
        return ((b - a) / abs(a)) * 100.0

    out = {
        "jetson_only_success_mean": round(jo.get("success_mean", 0.0), 6),
        "jetson_loihi_success_mean": round(jl.get("success_mean", 0.0), 6),
        "success_delta_percent_loihi_vs_jetson_only": round(
            delta(jo.get("success_mean", 0.0), jl.get("success_mean", 0.0)), 6
        ),
        "jetson_only_collision_mean": round(jo.get("collision_count_mean", 0.0), 6),
        "jetson_loihi_collision_mean": round(jl.get("collision_count_mean", 0.0), 6),
        "collision_delta_percent_loihi_vs_jetson_only": round(
            delta(jo.get("collision_count_mean", 0.0), jl.get("collision_count_mean", 0.0)), 6
        ),
        "jetson_only_ttg_mean_sec": round(jo.get("time_to_goal_sec_mean", 0.0), 6),
        "jetson_loihi_ttg_mean_sec": round(jl.get("time_to_goal_sec_mean", 0.0), 6),
        "ttg_delta_percent_loihi_vs_jetson_only": round(
            delta(jo.get("time_to_goal_sec_mean", 0.0), jl.get("time_to_goal_sec_mean", 0.0)), 6
        ),
    }
    write_single_row_csv(args.output, out)
    print(f"Wrote summary: {args.output}")


if __name__ == "__main__":
    main()
