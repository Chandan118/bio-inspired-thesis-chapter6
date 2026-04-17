#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
import pandas as pd


def ensure_out(path: str):
    os.makedirs(path, exist_ok=True)


def fig2_trajectory(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    envs = ["indoor", "outdoor", "forest"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), constrained_layout=True)

    for i, env in enumerate(envs):
        ax = axes[i]
        d = df[df["environment"] == env]
        if d.empty:
            ax.set_title(f"{env} (no data)")
            ax.grid(True, alpha=0.3)
            continue
        ax.plot(d["x_gt"], d["y_gt"], label="GPS Ground Truth", linewidth=2)
        ax.plot(d["x_est"], d["y_est"], label="SLAM Estimate", linewidth=1.8)
        ax.fill_between(d["x_gt"], d["y_gt"], d["y_est"], alpha=0.15, color="red", label="Error region")
        ax.set_title(env.capitalize())
        ax.set_xlabel("X position (m)")
        ax.set_ylabel("Y position (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    fig.suptitle("Figure 2 - Trajectory Comparison Across Environments", fontsize=12)
    fig.savefig(out_png, dpi=300)
    plt.close(fig)


def fig3_rmse_curve(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(df["time_min"], df["rmse_jetson_only_m"], label="Jetson-only", linewidth=2)
    ax.plot(df["time_min"], df["rmse_hybrid_m"], label="Hybrid (Jetson+Loihi)", linewidth=2)
    ax.set_title("Figure 3 - Localization Error Over Time")
    ax.set_xlabel("Time (minutes)")
    ax.set_ylabel("Position error (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def fig4_power(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.stackplot(
        df["time_sec"],
        df["power_jetson_w"],
        df["power_loihi_w"],
        df["power_sensors_w"],
        labels=["Jetson", "Loihi", "Sensors"],
        alpha=0.8,
    )
    ax.set_title("Figure 4 - Real-Time Power Consumption")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (W)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def fig5_memory(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(df["time_sec"], df["memory_gb"], label="Jetson RAM usage", linewidth=2)
    crash_df = df[df["crash_event"] == 1]
    if not crash_df.empty:
        ax.scatter(crash_df["time_sec"], crash_df["memory_gb"], marker="x", s=80, c="red", label="OOM crash")
    ax.axhline(8.0, linestyle="--", linewidth=1.5, color="black", label="8 GB limit")
    ax.set_title("Figure 5 - Memory Usage During Mission")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RAM usage (GB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def fig6_crash_complexity(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.scatter(df["orb_features_per_frame"], df["crash_rate_percent"], alpha=0.8)
    ax.set_title("Figure 6 - Crash Rate vs Environment Complexity")
    ax.set_xlabel("ORB features/frame")
    ax.set_ylabel("Crash rate (%)")
    ax.grid(True, alpha=0.3)
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def fig7_duration(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(
        df["system"],
        df["mean_duration_hours"],
        yerr=df["std_duration_hours"],
        capsize=6,
        alpha=0.85,
    )
    ax.set_title("Figure 7 - Mission Duration Comparison")
    ax.set_ylabel("Duration (hours)")
    ax.set_xlabel("System")
    ax.grid(True, axis="y", alpha=0.3)
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def fig8_compression(input_csv: str, out_png: str):
    df = pd.read_csv(input_csv)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(df["map_size_mb"], df["peak_without_compression_gb"], marker="o", label="Without compression")
    ax.plot(df["map_size_mb"], df["peak_with_compression_gb"], marker="o", label="With compression")
    ax.set_title("Figure 8 - Memory Compression Algorithm Results")
    ax.set_xlabel("Map size (MB)")
    ax.set_ylabel("Peak memory usage (GB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Generate manuscript figures 2-8")
    parser.add_argument("--data-dir", required=True, help="Directory containing figure CSV inputs")
    parser.add_argument("--out-dir", required=True, help="Directory for output PNG figures")
    args = parser.parse_args()

    ensure_out(args.out_dir)
    fig2_trajectory(os.path.join(args.data_dir, "figure2_trajectory.csv"), os.path.join(args.out_dir, "Figure2_Trajectory.png"))
    fig3_rmse_curve(os.path.join(args.data_dir, "figure3_rmse_curve.csv"), os.path.join(args.out_dir, "Figure3_RMSE_Curve.png"))
    fig4_power(os.path.join(args.data_dir, "figure4_power_log.csv"), os.path.join(args.out_dir, "Figure4_Power_Log.png"))
    fig5_memory(os.path.join(args.data_dir, "figure5_memory_usage.csv"), os.path.join(args.out_dir, "Figure5_Memory_Usage.png"))
    fig6_crash_complexity(
        os.path.join(args.data_dir, "figure6_crash_vs_complexity.csv"),
        os.path.join(args.out_dir, "Figure6_Crash_vs_Complexity.png"),
    )
    fig7_duration(
        os.path.join(args.data_dir, "figure7_mission_duration.csv"),
        os.path.join(args.out_dir, "Figure7_Mission_Duration.png"),
    )
    fig8_compression(
        os.path.join(args.data_dir, "figure8_compression_results.csv"),
        os.path.join(args.out_dir, "Figure8_Compression_Results.png"),
    )
    print(f"Generated figures in: {args.out_dir}")


if __name__ == "__main__":
    main()
