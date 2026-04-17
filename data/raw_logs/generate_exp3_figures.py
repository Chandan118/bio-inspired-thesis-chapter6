#!/usr/bin/env python3
from pathlib import Path
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd


EXP1_DIR = Path("/home/jetson/exp1_logs")
DATA_DIR = Path("/home/jetson/formica_experiments/data")


def latest_file(folder: Path, pattern: str) -> Path | None:
    files = sorted(folder.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def ensure_landmarks(df: pd.DataFrame):
    defaults = pd.DataFrame(
        [
            {"landmark_id": 0, "gt_x": 1.0, "gt_y": 0.0},
            {"landmark_id": 1, "gt_x": 1.0, "gt_y": 2.0},
            {"landmark_id": 2, "gt_x": 0.0, "gt_y": 2.0},
            {"landmark_id": 3, "gt_x": 2.0, "gt_y": 2.0},
        ]
    )
    if df.empty:
        return defaults
    got = df[["landmark_id", "gt_x", "gt_y"]].drop_duplicates()
    got["landmark_id"] = pd.to_numeric(got["landmark_id"], errors="coerce")
    got = got.dropna(subset=["landmark_id"])
    got["landmark_id"] = got["landmark_id"].astype(int)
    defaults["landmark_id"] = defaults["landmark_id"].astype(int)
    out = defaults.merge(got, on="landmark_id", how="left", suffixes=("_def", ""))
    out["gt_x"] = pd.to_numeric(out["gt_x"], errors="coerce").fillna(
        pd.to_numeric(out["gt_x_def"], errors="coerce")
    )
    out["gt_y"] = pd.to_numeric(out["gt_y"], errors="coerce").fillna(
        pd.to_numeric(out["gt_y_def"], errors="coerce")
    )
    return out[["landmark_id", "gt_x", "gt_y"]]


def draw_fig64(exp3_csv: Path):
    df = pd.read_csv(exp3_csv)
    # Summary rows (e.g. ALL,RMSE) force pandas to infer string columns; coerce before stats/plots.
    for col in ("gt_x", "gt_y", "est_x", "est_y", "error_m", "coverage_pct"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    if "landmark_id" in df.columns:
        df["landmark_id"] = pd.to_numeric(df["landmark_id"], errors="coerce")
    df = df[pd.to_numeric(df["trial"], errors="coerce").notna()].copy()
    if df.empty:
        df = pd.DataFrame(columns=["trial", "landmark_id", "gt_x", "gt_y", "est_x", "est_y", "error_m", "coverage_pct"])

    lms = ensure_landmarks(df)

    fig, ax = plt.subplots(figsize=(7, 7))
    # Manual floor plan base: 4m x 4m arena.
    arena = patches.Rectangle((0, 0), 4, 4, fill=False, edgecolor="black", linewidth=2)
    ax.add_patch(arena)
    # Add simple corridor-like guide lines to mimic manual floor plan context.
    ax.plot([1.2, 1.2], [0.0, 3.2], "--", color="gray", alpha=0.4)
    ax.plot([2.8, 2.8], [0.8, 4.0], "--", color="gray", alpha=0.4)
    ax.plot([0.0, 3.0], [1.4, 1.4], "--", color="gray", alpha=0.4)

    # Ground-truth landmarks.
    ax.scatter(lms["gt_x"], lms["gt_y"], marker="s", s=80, color="#1f77b4", label="Ground truth landmark")
    for _, r in lms.iterrows():
        ax.text(r["gt_x"] + 0.05, r["gt_y"] + 0.05, f"ID{int(r['landmark_id'])}", fontsize=9)

    if not df.empty:
        ax.scatter(df["est_x"], df["est_y"], marker="x", s=70, color="#d62728", label="Estimated pose")
        # Error ellipses per landmark from estimate covariance proxy (sample std).
        for lm_id, g in df.groupby("landmark_id"):
            if g.empty:
                continue
            cx = float(g["est_x"].mean())
            cy = float(g["est_y"].mean())
            sx = float(g["est_x"].std(ddof=0)) if len(g) > 1 else 0.05
            sy = float(g["est_y"].std(ddof=0)) if len(g) > 1 else 0.05
            ell = patches.Ellipse((cx, cy), width=max(0.05, 2 * sx), height=max(0.05, 2 * sy),
                                  fill=False, edgecolor="#ff7f0e", linewidth=1.8, alpha=0.9)
            ax.add_patch(ell)
            gt = lms[lms["landmark_id"] == lm_id]
            if not gt.empty:
                gx, gy = float(gt.iloc[0]["gt_x"]), float(gt.iloc[0]["gt_y"])
                ax.plot([gx, cx], [gy, cy], color="#d62728", alpha=0.35, linewidth=1.2)

    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-0.2, 4.2)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.set_title("Figure 6.4 SLAM Map Overlay with Landmark Error Ellipses")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    out = EXP1_DIR / "Figure6_4_Map_Overlay.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def draw_fig65(traj_csv: Path):
    tdf = pd.read_csv(traj_csv)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_facecolor("#f4f4f4")
    # Occupancy-like grid backdrop.
    for x in np.arange(0, 4.01, 0.2):
        ax.plot([x, x], [0, 4], color="#d9d9d9", linewidth=0.5, zorder=0)
    for y in np.arange(0, 4.01, 0.2):
        ax.plot([0, 4], [y, y], color="#d9d9d9", linewidth=0.5, zorder=0)

    if not tdf.empty:
        xs = tdf["x"].to_numpy(dtype=float)
        ys = tdf["y"].to_numpy(dtype=float)
        ax.plot(xs, ys, color="#2ca02c", linewidth=1.8, label="Exploration trajectory")
        ax.scatter(xs[:1], ys[:1], color="#1f77b4", s=45, label="Start")
        ax.scatter(xs[-1:], ys[-1:], color="#d62728", s=45, label="End")

    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-0.2, 4.2)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Figure 6.5 Exploration Trajectory on Occupancy Grid")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    out = EXP1_DIR / "Figure6_5_Exploration_Trajectory.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def main():
    exp3_csv = latest_file(DATA_DIR, "exp3_slam_*.csv")
    traj_csv = latest_file(EXP1_DIR, "exp3_trajectory_*.csv")
    if exp3_csv is None:
        raise SystemExit("No exp3_slam CSV found.")
    if traj_csv is None:
        raise SystemExit("No exp3 trajectory CSV found.")

    f64 = draw_fig64(exp3_csv)
    f65 = draw_fig65(traj_csv)
    print(f"FIG6_4,{f64}")
    print(f"FIG6_5,{f65}")
    print(f"EXP3_SOURCE,{exp3_csv}")
    print(f"TRAJ_SOURCE,{traj_csv}")


if __name__ == "__main__":
    main()
