# Jetson Appeal Experiment Suite

This package gives you a complete, repeatable workflow for the five experiment blocks required for your appeal:

1. Structured field trials
2. Localization accuracy benchmark (RMSE)
3. Power measurement logging summary
4. Compression algorithm memory test on Jetson
5. Baseline comparison (Jetson-only vs neuromorphic/Loihi mode)

The scripts are built so you can:

- fill raw logs into CSV templates,
- run one command per analysis,
- generate publication-ready summary tables and a single report.

## Folder Layout

- `data/raw/`: put your raw logs here
- `data/templates/`: CSV formats to fill
- `outputs/`: generated results and summary CSV files
- `scripts/`: analysis scripts

## Quick Start

1) Fill the CSV templates in `data/templates/` and place completed files in `data/raw/` using these exact names:

- `field_trials.csv`
- `localization_trials.csv`
- `power_log.csv`
- `memory_log.csv`
- `baseline_comparison.csv`

2) Run everything in one command:

```bash
bash run_all.sh
```

Or pass a custom data directory:

```bash
bash run_all.sh /path/to/your/data
```

3) (Optional) Run analyses manually:

```bash
python3 scripts/analyze_localization.py --input data/raw/localization_trials.csv --output outputs/localization_summary.csv
python3 scripts/analyze_power.py --input data/raw/power_log.csv --output outputs/power_summary.csv
python3 scripts/analyze_memory.py --input data/raw/memory_log.csv --output outputs/memory_summary.csv
python3 scripts/compare_baseline.py --input data/raw/baseline_comparison.csv --output outputs/baseline_summary.csv
python3 scripts/summarize_trials.py --input data/raw/field_trials.csv --output outputs/field_trials_summary.csv
python3 scripts/generate_report.py --out outputs/appeal_results_report.md
```

4) Open `outputs/appeal_results_report.md` and paste into manuscript/response letter.

## Important

- This suite does not invent scientific data.
- Results are computed only from files you provide.
- Use real instrument logs (power meter, trajectory GT, memory logs).
- `run_all.sh` now validates CSV schemas before analysis to prevent bad manuscript outputs.

## Figure Generation (Figures 2-8)

Use the plot script after you fill figure CSV files:

```bash
python3 scripts/generate_figures.py \
  --data-dir data/templates \
  --out-dir outputs/figures
```

Required CSV files:

- `figure2_trajectory.csv`
- `figure3_rmse_curve.csv`
- `figure4_power_log.csv`
- `figure5_memory_usage.csv`
- `figure6_crash_vs_complexity.csv`
- `figure7_mission_duration.csv`
- `figure8_compression_results.csv`
