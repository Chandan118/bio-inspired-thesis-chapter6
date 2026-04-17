#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${1:-$ROOT_DIR/data/raw}"
OUT_DIR="$ROOT_DIR/outputs"

mkdir -p "$OUT_DIR"

echo "[1/7] Validating CSV files in: $DATA_DIR"
python3 "$ROOT_DIR/scripts/validate_csvs.py" --data-dir "$DATA_DIR"

echo "[2/7] Summarizing field trials"
python3 "$ROOT_DIR/scripts/summarize_trials.py" \
  --input "$DATA_DIR/field_trials.csv" \
  --output "$OUT_DIR/field_trials_summary.csv"

echo "[3/7] Analyzing localization"
python3 "$ROOT_DIR/scripts/analyze_localization.py" \
  --input "$DATA_DIR/localization_trials.csv" \
  --output "$OUT_DIR/localization_summary.csv"

echo "[4/7] Analyzing power"
python3 "$ROOT_DIR/scripts/analyze_power.py" \
  --input "$DATA_DIR/power_log.csv" \
  --output "$OUT_DIR/power_summary.csv"

echo "[5/7] Analyzing memory"
python3 "$ROOT_DIR/scripts/analyze_memory.py" \
  --input "$DATA_DIR/memory_log.csv" \
  --output "$OUT_DIR/memory_summary.csv"

echo "[6/7] Comparing baseline modes"
python3 "$ROOT_DIR/scripts/compare_baseline.py" \
  --input "$DATA_DIR/baseline_comparison.csv" \
  --output "$OUT_DIR/baseline_summary.csv"

echo "[7/7] Generating manuscript-ready report"
python3 "$ROOT_DIR/scripts/generate_report.py" \
  --out "$OUT_DIR/appeal_results_report.md"

echo "Done."
echo "Outputs in: $OUT_DIR"
