#!/usr/bin/env python3
"""
Filter Task 2 MQ CSV rows by phase (drop baseline, tests, or keep only residual).

Examples:
  # Chemical-backup trail only (after cotton removed):
  python3 task2_filter_csv.py -i exp7_task2_mq_ethanol_20260330_191306_all.csv \\
      -o FINAL_residual_only.csv --phases residual_after_removal

  # Exposure + residual (no baseline):
  python3 task2_filter_csv.py -i ..._all.csv -o FINAL_exposure_and_residual.csv \\
      --phases ethanol_preactivation residual_after_removal

  # Merge separate phase files from one run (no _all.csv):
  python3 task2_filter_csv.py -i ..._baseline.csv ..._ethanol_60s.csv ..._residual.csv \\
      -o merged.csv --phases residual_after_removal
"""

from __future__ import annotations

import argparse
import csv
import os
import sys


def main() -> None:
    ap = argparse.ArgumentParser(description='Filter Task 2 CSV by phase column')
    ap.add_argument(
        '-i', '--input', nargs='+', required=True, help='One or more input CSV paths'
    )
    ap.add_argument('-o', '--output', required=True, help='Output CSV path')
    ap.add_argument(
        '--phases',
        nargs='+',
        required=True,
        help='Keep rows where phase matches one of these (e.g. residual_after_removal)',
    )
    args = ap.parse_args()
    keep = set(args.phases)
    out_path = os.path.expanduser(args.output)
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    fieldnames: list[str] | None = None
    n_in, n_out = 0, 0
    with open(out_path, 'w', newline='') as outf:
        writer: csv.DictWriter | None = None
        for path in args.input:
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                print(f'Skip missing: {path}', file=sys.stderr)
                continue
            with open(path, newline='') as inf:
                r = csv.DictReader(inf)
                if not r.fieldnames or 'phase' not in r.fieldnames:
                    print(f'Skip (no phase column): {path}', file=sys.stderr)
                    continue
                if fieldnames is None:
                    fieldnames = list(r.fieldnames)
                    writer = csv.DictWriter(outf, fieldnames=fieldnames)
                    writer.writeheader()
                assert writer is not None
                for row in r:
                    n_in += 1
                    if row.get('phase') in keep:
                        writer.writerow(row)
                        n_out += 1
    print(f'Wrote {n_out} rows (from {n_in} read) -> {out_path}')


if __name__ == '__main__':
    main()
