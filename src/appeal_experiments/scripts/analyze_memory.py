#!/usr/bin/env python3
import argparse

from common import read_csv_rows, safe_float, summarize, write_single_row_csv


def main():
    parser = argparse.ArgumentParser(description="Summarize memory usage during compression runs.")
    parser.add_argument("--input", required=True, help="Memory log CSV")
    parser.add_argument("--output", required=True, help="Output summary CSV")
    args = parser.parse_args()

    rows = read_csv_rows(args.input)
    if not rows:
        raise SystemExit("No rows found in input CSV.")

    mem_gb = [safe_float(r.get("memory_gb")) for r in rows]
    avg_mem, std_mem = summarize(mem_gb)
    peak_mem = max(mem_gb) if mem_gb else 0.0

    # Optional comparison: baseline_memory_gb and compressed_memory_gb columns
    base = [safe_float(r.get("baseline_memory_gb"), None) for r in rows]
    comp = [safe_float(r.get("compressed_memory_gb"), None) for r in rows]
    valid_pairs = [(b, c) for b, c in zip(base, comp) if b is not None and c is not None and b > 0]
    reduction = None
    if valid_pairs:
        reduction = sum((b - c) / b for b, c in valid_pairs) / len(valid_pairs) * 100.0

    out = {
        "samples": len(rows),
        "avg_memory_gb": round(avg_mem, 6),
        "std_memory_gb": round(std_mem, 6),
        "peak_memory_gb": round(peak_mem, 6),
        "avg_reduction_percent": round(reduction, 6) if reduction is not None else "",
    }
    write_single_row_csv(args.output, out)
    print(f"Wrote summary: {args.output}")


if __name__ == "__main__":
    main()
