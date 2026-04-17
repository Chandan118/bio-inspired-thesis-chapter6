"""
Experiment 2 post-processing: Table 6.2 + Figure 6.3 (thesis protocol).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

OUT_DIR_DEFAULT = Path("/home/jetson/exp1_logs")


def postprocess_exp2(
    csv_path: Path,
    tag: str,
    out_dir: Path | None = None,
    battery_wh: float = 55.08,
    target_mean_w: float = 1.2,
) -> dict:
    """Build Table 6.2 CSV, summary txt, Figure 6.3 PNG from Exp2 power CSV."""
    out_dir = out_dir or OUT_DIR_DEFAULT
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError("No rows logged in Exp2 CSV.")

    df["mode"] = df["mode"].astype(str).str.upper().str.strip()
    mode_order = ["TRANSIT", "DECISION", "STANDBY"]
    stats = df.groupby("mode")["power_W"].agg(["mean", "std", "max", "min", "count"]).reset_index()
    stats["mode"] = pd.Categorical(stats["mode"], categories=mode_order, ordered=True)
    stats = stats.sort_values("mode")

    overall_mean = float(df["power_W"].mean())
    overall_sd = float(df["power_W"].std(ddof=1)) if len(df) > 1 else 0.0
    overall_peak = float(df["power_W"].max())
    overall_min = float(df["power_W"].min())
    span_s = max(float(df["timestamp_s"].max() - df["timestamp_s"].min()), 1e-9)
    mission_hours = span_s / 3600.0
    energy_wh = overall_mean * mission_hours
    estimated_h = battery_wh / overall_mean if overall_mean > 0 else 0.0

    rows = []
    for _, r in stats.iterrows():
        rows.append({
            "row": f"{r['mode']} (this work)",
            "mean_W": round(float(r["mean"]), 4),
            "sd_W": round(float(r["std"]) if pd.notna(r["std"]) else 0.0, 4),
            "peak_W": round(float(r["max"]), 4),
            "min_W": round(float(r["min"]), 4),
        })
    rows.append({
        "row": "OVERALL (this work)",
        "mean_W": round(overall_mean, 4),
        "sd_W": round(overall_sd, 4),
        "peak_W": round(overall_peak, 4),
        "min_W": round(overall_min, 4),
    })
    rows.append({"row": "Kilobot (benchmark)", "mean_W": "N/A", "sd_W": "N/A", "peak_W": "N/A", "min_W": "N/A"})
    rows.append({"row": "EPFL Tribots (benchmark)", "mean_W": "N/A", "sd_W": "N/A", "peak_W": "N/A", "min_W": "N/A"})
    table = pd.DataFrame(rows)

    table_latest = out_dir / f"table6_2_power_profile_{tag}.csv"
    table_canonical = out_dir / "table6_2_power_profile.csv"
    table.to_csv(table_latest, index=False)
    table.to_csv(table_canonical, index=False)

    summary_latest = out_dir / f"exp2_summary_{tag}.txt"
    summary_canonical = out_dir / "exp2_summary.txt"
    summary_txt = "\n".join([
        f"csv_path,{csv_path}",
        f"overall_mean_W,{overall_mean:.6f}",
        f"overall_sd_W,{overall_sd:.6f}",
        f"overall_peak_W,{overall_peak:.6f}",
        f"overall_min_W,{overall_min:.6f}",
        f"energy_Wh,{energy_wh:.6f}",
        f"estimated_runtime_h,{estimated_h:.6f}",
        f"target_mean_le_{target_mean_w}W,{'PASS' if overall_mean <= target_mean_w else 'FAIL'}",
        "",
    ])
    summary_latest.write_text(summary_txt)
    summary_canonical.write_text(summary_txt)

    colors = {"TRANSIT": "#1f77b4", "DECISION": "#ff7f0e", "STANDBY": "#2ca02c"}
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(df["timestamp_s"], df["power_W"], color="black", linewidth=1.0, label="Power (W)")

    prev_mode = df.iloc[0]["mode"]
    prev_t = float(df.iloc[0]["timestamp_s"])
    for i in range(1, len(df)):
        m = df.iloc[i]["mode"]
        t = float(df.iloc[i]["timestamp_s"])
        if m != prev_mode:
            ax.axvspan(prev_t, t, color=colors.get(prev_mode, "#cccccc"), alpha=0.12)
            prev_mode = m
            prev_t = t
    ax.axvspan(prev_t, float(df["timestamp_s"].max()), color=colors.get(prev_mode, "#cccccc"), alpha=0.12)

    peak_idx = df["power_W"].idxmax()
    peak_t = float(df.loc[peak_idx, "timestamp_s"])
    peak_w = float(df.loc[peak_idx, "power_W"])
    ax.scatter([peak_t], [peak_w], color="red", zorder=5)
    ax.annotate(
        f"Peak {peak_w:.2f} W",
        (peak_t, peak_w),
        xytext=(peak_t + 0.04 * max(float(df["timestamp_s"].max()), 1.0), peak_w + 0.05),
        arrowprops=dict(arrowstyle="->", color="red"),
        color="red",
        fontsize=9,
    )

    ax.axhline(target_mean_w, color="red", linestyle="--", linewidth=1.2, label=f"Target {target_mean_w} W")
    ax.set_title("Figure 6.3 Power Profile with Mode Zones")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (W)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig_latest = out_dir / f"Figure6_3_Power_Profile_{tag}.png"
    fig_canonical = out_dir / "Figure6_3_Power_Profile.png"
    fig.savefig(fig_latest, dpi=180)
    fig.savefig(fig_canonical, dpi=180)
    plt.close(fig)

    return {
        "table": table_latest,
        "summary": summary_latest,
        "figure": fig_latest,
        "overall_mean": overall_mean,
        "estimated_h": estimated_h,
        "energy_wh": energy_wh,
    }
