#!/usr/bin/env python3
import argparse

from common import read_csv_rows, safe_float, summarize, write_single_row_csv


def main():
    parser = argparse.ArgumentParser(description="Summarize power meter logs.")
    parser.add_argument("--input", required=True, help="Power log CSV")
    parser.add_argument("--output", required=True, help="Output summary CSV")
    args = parser.parse_args()

    rows = read_csv_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in input CSV.")

    watts = [safe_float(r.get("power_w")) for r in rows]
    avg_w, std_w = summarize(watts)

    # Optional energy if timestamp_sec exists and monotonic
    ts = [safe_float(r.get("timestamp_sec")) for r in rows]
    energy_wh = None
    if len(ts) > 1 and all(ts[i] <= ts[i + 1] for i in range(len(ts) - 1)):
        joules = 0.0
        for i in range(1, len(rows)):
            dt = ts[i] - ts[i - 1]
            p_mid = 0.5 * (watts[i] + watts[i - 1])
            joules += p_mid * dt
        energy_wh = joules / 3600.0

    out = {
        "samples": len(rows),
        "avg_power_w": round(avg_w, 6),
        "std_power_w": round(std_w, 6),
        "peak_power_w": round(max(watts), 6) if watts else 0.0,
        "min_power_w": round(min(watts), 6) if watts else 0.0,
    }
    if energy_wh is not None:
        out["energy_wh"] = round(energy_wh, 6)
    else:
        out["energy_wh"] = ""

    write_single_row_csv(args.output, out)
    print(f"Wrote summary: {args.output}")


if __name__ == "__main__":
    main()
