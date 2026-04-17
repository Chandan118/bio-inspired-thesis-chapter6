#!/usr/bin/env python3
import argparse
from collections import defaultdict

from common import read_csv_rows, safe_float, summarize, write_single_row_csv


def main():
    parser = argparse.ArgumentParser(description="Summarize structured field trials.")
    parser.add_argument("--input", required=True, help="Input field trials CSV")
    parser.add_argument("--output", required=True, help="Output summary CSV")
    args = parser.parse_args()

    rows = read_csv_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in input CSV.")

    total = len(rows)
    success = sum(1 for r in rows if str(r.get("status", "")).lower() == "success")
    fail = total - success
    durations = [safe_float(r.get("duration_min")) for r in rows]
    avg_duration, std_duration = summarize(durations)

    by_env = defaultdict(lambda: {"total": 0, "success": 0})
    for r in rows:
        env = r.get("environment", "unknown")
        by_env[env]["total"] += 1
        if str(r.get("status", "")).lower() == "success":
            by_env[env]["success"] += 1

    env_breakdown = "; ".join(
        f"{k}: {v['success']}/{v['total']}" for k, v in sorted(by_env.items())
    )

    write_single_row_csv(
        args.output,
        {
            "total_trials": total,
            "success_trials": success,
            "failed_trials": fail,
            "success_rate_percent": round((success / total) * 100, 3) if total else 0.0,
            "avg_duration_min": round(avg_duration, 3),
            "std_duration_min": round(std_duration, 3),
            "environment_breakdown": env_breakdown,
        },
    )
    print(f"Wrote summary: {args.output}")


if __name__ == "__main__":
    main()
