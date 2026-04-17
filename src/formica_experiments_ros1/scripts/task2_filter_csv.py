#!/usr/bin/env python3
"""
task2_filter_csv.py - ROS 1 Version
=====================================
Filter Task 2 MQ CSV rows by phase (drop baseline, tests, or keep only residual).
"""

import argparse
import csv
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Filter Task 2 CSV by phase column')
    parser.add_argument('-i', '--input', nargs='+', required=True,
                        help='One or more input CSV paths')
    parser.add_argument('-o', '--output', required=True, help='Output CSV path')
    parser.add_argument('--phases', nargs='+', required=True,
                        help='Keep rows where phase matches (e.g. residual_after_removal)')
    args = parser.parse_args()

    keep = set(args.phases)
    out_path = os.path.expanduser(args.output)
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)

    fieldnames = None
    n_in, n_out = 0, 0
    with open(out_path, 'w') as outf:
        writer = None
        for path in args.input:
            path = os.path.expanduser(path)
            if not os.path.isfile(path):
                print('Skip missing: {0}'.format(path), file=sys.stderr)
                continue
            with open(path) as inf:
                r = csv.DictReader(inf)
                if not r.fieldnames or 'phase' not in r.fieldnames:
                    print('Skip (no phase column): {0}'.format(path), file=sys.stderr)
                    continue
                if fieldnames is None:
                    fieldnames = list(r.fieldnames)
                    writer = csv.DictWriter(outf, fieldnames=fieldnames)
                    writer.writeheader()
                for row in r:
                    n_in += 1
                    if row.get('phase') in keep:
                        writer.writerow(row)
                        n_out += 1
    print('Wrote {0} rows (from {1} read) -> {2}'.format(n_out, n_in, out_path))


if __name__ == '__main__':
    main()
