"""
Post-process Exp7 CSV: lateral deviation plots (A/B), SNR switchover table (C), LED curve (D).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

OUT_DIR_DEFAULT = Path('/home/jetson/exp1_logs')


def _mean_adc_row(row: pd.Series) -> float:
    vals = [row.get(f's{i}', float('nan')) for i in range(4)]
    nums = [float(x) for x in vals if pd.notna(x) and str(x) != '']
    return float(sum(nums) / len(nums)) if nums else float('nan')


def postprocess_exp7(csv_path: Path, out_dir: Path | None = None) -> dict[str, Path]:
    out_dir = out_dir or OUT_DIR_DEFAULT
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = csv_path.stem.replace('exp7_pheromone_', '')

    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError('Empty Exp7 CSV.')

    for col in ('dist_m', 'lateral_dev_m', 'snr_db', 'led_pwm', 's0', 's1', 's2', 's3'):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')

    outputs: dict[str, Path] = {}

    # --- A: straight ---
    a = df[df['sub_exp'].astype(str).str.startswith('A_straight')].copy()
    if not a.empty and a['dist_m'].notna().any():
        fig, ax = plt.subplots(figsize=(8, 4))
        for trial in sorted(a['trial'].dropna().unique()):
            sub = a[a['trial'] == trial].sort_values('dist_m')
            ax.plot(sub['dist_m'], sub['lateral_dev_m'] * 100.0, alpha=0.35, linewidth=1)
        ax.axhline(1.5, color='red', linestyle='--', label='Target 1.5 cm')
        ax.set_title('Exp7 Sub-A: Lateral deviation vs distance (straight)')
        ax.set_xlabel('Distance along trail (m)')
        ax.set_ylabel('Lateral deviation (cm)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        p = out_dir / f'figure7_A_straight_lateral_{tag}.png'
        fig.tight_layout()
        fig.savefig(p, dpi=150)
        plt.close(fig)
        outputs['A'] = p

    # --- B: curved ---
    b = df[df['sub_exp'].astype(str).str.startswith('B_curved')].copy()
    if not b.empty and b['dist_m'].notna().any():
        fig, ax = plt.subplots(figsize=(8, 4))
        for trial in sorted(b['trial'].dropna().unique()):
            sub = b[b['trial'] == trial].sort_values('dist_m')
            ax.plot(sub['dist_m'], sub['lateral_dev_m'] * 100.0, alpha=0.35, linewidth=1)
        ax.axhline(2.0, color='red', linestyle='--', label='Target 2.0 cm')
        ax.set_title('Exp7 Sub-B: Lateral deviation vs distance (curved)')
        ax.set_xlabel('Distance along trail (m)')
        ax.set_ylabel('Lateral deviation (cm)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        p = out_dir / f'figure7_B_curved_lateral_{tag}.png'
        fig.tight_layout()
        fig.savefig(p, dpi=150)
        plt.close(fig)
        outputs['B'] = p

    # --- C: switchover table ---
    c = df[df['sub_exp'].astype(str) == 'C_snr'].copy()
    if not c.empty:
        table = c[['trial', 'ambient_pct', 'switchover_latency_s', 'continued_following_or_trail_any']].copy()
        p = out_dir / f'table7_C_snr_switchover_{tag}.csv'
        table.to_csv(p, index=False)
        outputs['C'] = p

    # --- D: LED vs ADC ---
    d = df[df['sub_exp'].astype(str) == 'D_decay'].copy()
    if not d.empty:
        d['mean_adc'] = d.apply(_mean_adc_row, axis=1)
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.plot(d['led_pwm'], d['mean_adc'], 'o-', color='darkred')
        ax.axhline(1500, color='gray', linestyle='--', label='ADC threshold 1500')
        ax.set_title('Exp7 Sub-D: LED PWM vs mean TCRT ADC (decay)')
        ax.set_xlabel('LED PWM (0–1)')
        ax.set_ylabel('Mean ADC')
        ax.legend()
        ax.grid(True, alpha=0.3)
        p = out_dir / f'figure7_D_led_adc_decay_{tag}.png'
        fig.tight_layout()
        fig.savefig(p, dpi=150)
        plt.close(fig)
        outputs['D'] = p

    return outputs


def run_cli() -> int:
    ap = argparse.ArgumentParser(description='Exp7 post-process')
    ap.add_argument('csv', nargs='?', help='Path to exp7_pheromone_*.csv')
    ap.add_argument('--out-dir', type=Path, default=OUT_DIR_DEFAULT)
    args = ap.parse_args()
    if args.csv:
        csv_path = Path(args.csv)
    else:
        data = Path.home() / 'formica_experiments' / 'data'
        candidates = sorted(data.glob('exp7_pheromone_*.csv'), key=lambda p: p.stat().st_mtime, reverse=True)
        if not candidates:
            print('No exp7_pheromone_*.csv found.', file=sys.stderr)
            return 1
        csv_path = candidates[0]
        print(f'Using latest: {csv_path}')

    paths = postprocess_exp7(csv_path, args.out_dir)
    for k, p in paths.items():
        print(f'[{k}] {p}')
    return 0


def main() -> None:
    raise SystemExit(run_cli())


if __name__ == '__main__':
    raise SystemExit(run_cli())
