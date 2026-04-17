#!/usr/bin/env python3
import argparse
import math

from common import read_csv_rows, safe_float, summarize, write_single_row_csv


def main():
    parser = argparse.ArgumentParser(description="Compute localization RMSE summary.")
    parser.add_argument("--input", required=True, help="Localization CSV")
    parser.add_argument("--output", required=True, help="Output summary CSV")
    args = parser.parse_args()

    rows = read_csv_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in input CSV.")

    errors = []
    for r in rows:
        x_pred = safe_float(r.get("x_pred"))
        y_pred = safe_float(r.get("y_pred"))
        x_gt = safe_float(r.get("x_gt"))
        y_gt = safe_float(r.get("y_gt"))
        err = math.sqrt((x_pred - x_gt) ** 2 + (y_pred - y_gt) ** 2)
        errors.append(err)

    rmse = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else 0.0
    mean_err, std_err = summarize(errors)

    write_single_row_csv(
        args.output,
        {
            "samples": len(errors),
            "rmse_m": round(rmse, 6),
            "mean_abs_error_m": round(mean_err, 6),
            "std_abs_error_m": round(std_err, 6),
        },
    )
    print(f"Wrote summary: {args.output}")


if __name__ == "__main__":
    main()
