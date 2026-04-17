#!/usr/bin/env python3
import argparse
import csv
import os
import sys


REQUIRED_SCHEMAS = {
    "field_trials.csv": [
        "trial_id",
        "environment",
        "mode",
        "start_time",
        "end_time",
        "duration_min",
        "status",
        "crash_flag",
        "notes",
    ],
    "localization_trials.csv": [
        "timestamp_sec",
        "trial_id",
        "x_pred",
        "y_pred",
        "x_gt",
        "y_gt",
    ],
    "power_log.csv": [
        "timestamp_sec",
        "trial_id",
        "stage",
        "power_w",
        "voltage_v",
        "current_a",
    ],
    "memory_log.csv": [
        "timestamp_sec",
        "trial_id",
        "memory_gb",
        "baseline_memory_gb",
        "compressed_memory_gb",
    ],
    "baseline_comparison.csv": [
        "trial_id",
        "task_id",
        "environment",
        "mode",
        "success",
        "collision_count",
        "time_to_goal_sec",
    ],
}

NUMERIC_COLUMNS = {
    "field_trials.csv": ["duration_min", "crash_flag"],
    "localization_trials.csv": ["timestamp_sec", "x_pred", "y_pred", "x_gt", "y_gt"],
    "power_log.csv": ["timestamp_sec", "power_w", "voltage_v", "current_a"],
    "memory_log.csv": [
        "timestamp_sec",
        "memory_gb",
        "baseline_memory_gb",
        "compressed_memory_gb",
    ],
    "baseline_comparison.csv": ["success", "collision_count", "time_to_goal_sec"],
}

OPTIONAL_COLUMNS = {
    "field_trials.csv": ["notes"],
}


def is_float(value):
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False


def validate_file(path, file_name, required_cols, numeric_cols):
    errors = []
    if not os.path.exists(path):
        errors.append(f"Missing file: {path}")
        return errors

    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        actual_cols = reader.fieldnames or []
        missing = [c for c in required_cols if c not in actual_cols]
        if missing:
            errors.append(f"{os.path.basename(path)} missing columns: {missing}")
            return errors

        rows = list(reader)
        if not rows:
            errors.append(f"{os.path.basename(path)} has no data rows")
            return errors

        for i, row in enumerate(rows, start=2):
            for col in required_cols:
                if col in OPTIONAL_COLUMNS.get(file_name, []):
                    continue
                if row.get(col, "") == "":
                    errors.append(f"{os.path.basename(path)} line {i}: empty value in '{col}'")
            for col in numeric_cols:
                if not is_float(row.get(col, "")):
                    errors.append(f"{os.path.basename(path)} line {i}: non-numeric '{col}'")

    return errors


def main():
    parser = argparse.ArgumentParser(description="Validate experiment CSV files.")
    parser.add_argument("--data-dir", required=True, help="Directory containing CSV files")
    args = parser.parse_args()

    all_errors = []
    for fname, schema in REQUIRED_SCHEMAS.items():
        path = os.path.join(args.data_dir, fname)
        errs = validate_file(path, fname, schema, NUMERIC_COLUMNS.get(fname, []))
        all_errors.extend(errs)

    if all_errors:
        print("CSV validation failed:")
        for e in all_errors:
            print(f"- {e}")
        sys.exit(1)
    print("CSV validation passed for all required files.")


if __name__ == "__main__":
    main()
