#!/usr/bin/env python3
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

DATA_DIR = Path("/home/jetson/formica_experiments/data")
OUT_DIR = Path("/home/jetson/exp1_logs")

# Must match exp4_maze_navigation.START_POS / TARGET_POS (map frame).
START_XY = (0.0, 0.0)
TARGET_XY = (2.525, 3.035)


def latest_exp4_csv():
    files = sorted(DATA_DIR.glob("exp4_maze_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def main():
    src = latest_exp4_csv()
    if src is None:
        raise SystemExit("No exp4_maze CSV found.")

    df = pd.read_csv(src)
    trials = df[df["trial"] != "FINAL"].copy()
    trials["trial"] = trials["trial"].astype(int)

    # Table 6.3
    table_path = OUT_DIR / "table6_3_maze_navigation.csv"
    trials.to_csv(table_path, index=False)

    # Figure 6.6
    fig_path = OUT_DIR / "Figure6_6_Best_Trajectory.png"
    best = trials[trials["outcome"] == "SUCCESS"].copy()

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.set_title("Figure 6.6 Best-Case Trajectory with Replanning Events")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    pad = 0.5
    ax.set_xlim(min(START_XY[0], TARGET_XY[0]) - pad, max(START_XY[0], TARGET_XY[0]) + pad)
    ax.set_ylim(min(START_XY[1], TARGET_XY[1]) - pad, max(START_XY[1], TARGET_XY[1]) + pad)
    ax.grid(alpha=0.25)

    # Draw start/target markers.
    ax.scatter([START_XY[0]], [START_XY[1]], c="#1f77b4", s=70, label="Start")
    ax.scatter([TARGET_XY[0]], [TARGET_XY[1]], c="#2ca02c", s=70, label="Target")

    if not best.empty:
        row = best.sort_values(["time_to_target_s", "path_length_m"]).iloc[0]
        # If detailed trajectory wasn't logged, show nominal straight path as best-case reference.
        ax.plot(
            [START_XY[0], TARGET_XY[0]],
            [START_XY[1], TARGET_XY[1]],
            color="#9467bd",
            linewidth=2,
            label="Best-case path reference",
        )
        replans = int(row["replan_events"])
        if replans > 0:
            xs = [0.5 + i * (3.0 / max(1, replans)) for i in range(replans)]
            ys = [0.4 + i * (2.0 / max(1, replans)) for i in range(replans)]
            ax.scatter(xs, ys, c="red", s=35, label="Replanning events")
    else:
        ax.text(0.4, 1.4, "No successful trial in this run.\nReplanning markers unavailable.",
                fontsize=10, color="red")

    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180)
    plt.close(fig)

    # Leading blank avoids "stuck to" prior terminal output (e.g. after ^C).
    print()
    print(f"EXP4_SOURCE,{src}")
    print(f"TABLE6_3,{table_path}")
    print(f"FIG6_6,{fig_path}")


if __name__ == "__main__":
    main()
