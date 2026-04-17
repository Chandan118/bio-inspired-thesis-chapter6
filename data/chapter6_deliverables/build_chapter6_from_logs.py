#!/usr/bin/env python3
"""
Build Chapter 6 tables and figures from FormicaBot logs on this machine.

Every output file lists its source CSV in PROVENANCE.md (generated).
This script does not invent measurements: thresholds and pass/fail are computed
from the data you actually logged.

Subfigures (b) for map-related figures 6.2, 6.4, 6.5, 6.6:
  (b) is built ONLY from a second map file, never from the same path as formica_map.pgm (otherwise
  it would duplicate subfigure (a)). After a new SLAM run on the robot, save the new grid as:
    /home/jetson/formica_map_b.pgm  (+ formica_map_b.yaml)
  Optional:
  - FORMICA_MAP_B — override path for the (b) occupancy .pgm
  - EXP3_SLAM_CSV_B — SLAM landmark CSV for Figure_6_4b (else: second-latest exp3_slam_*.csv if any)
  - EXP3_TRAJECTORY_B — pose CSV for Figure_6_5b (else: exp1_logs/exp3_trajectory_b.csv if present)

Frozen subfigures (a) under Figure_*_a_* are not overwritten by this script.
"""

from __future__ import annotations

import ast
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BASE = Path("/home/jetson/chapter6_formica_deliverables")
DATA = Path("/home/jetson/formica_experiments/data")
EXP1 = Path("/home/jetson/exp1_logs")
MAP_YAML = Path("/home/jetson/formica_map.yaml")
MAP_PGM = Path("/home/jetson/formica_map.pgm")
TRAJ = EXP1 / "exp3_trajectory_20260402_072320.csv"
# Subfigure (b): must be a different on-disk map from MAP_PGM (saved after a new robot SLAM run).
MAP_PGM_B_DEFAULT = Path("/home/jetson/formica_map_b.pgm")
TRAJ_B_DEFAULT = EXP1 / "exp3_trajectory_b.csv"
MAP_PGM_B = Path(os.environ["FORMICA_MAP_B"]).expanduser() if os.environ.get("FORMICA_MAP_B") else MAP_PGM_B_DEFAULT
TRAJ_B = Path(os.environ["EXP3_TRAJECTORY_B"]).expanduser() if os.environ.get("EXP3_TRAJECTORY_B") else TRAJ_B_DEFAULT

VARIANT_B_FIGURE_PNGS = [
    "Figure_6_2b_Map_From_LiDAR_SLAM.png",
    "Figure_6_4b_Map_Overlay.png",
    "Figure_6_5b_Exploration_Trajectory.png",
    "Figure_6_6b_Maze_Protocol_Overlay.png",
]


def _variant_b_map_ready() -> bool:
    """True only if map B exists and is not the same file as the canonical map (no duplicate a/b)."""
    if not MAP_PGM.is_file() or not MAP_PGM_B.is_file():
        return False
    try:
        return MAP_PGM_B.resolve() != MAP_PGM.resolve()
    except OSError:
        return False


def _variant_b_missing_instructions() -> str:
    return (
        "Figure (b) is not generated until you save a NEW occupancy map from the robot.\n\n"
        "Subfigure (a) stays frozen; (b) must use a different file than formica_map.pgm.\n\n"
        "After SLAM, save the new map as:\n"
        f"  {MAP_PGM_B_DEFAULT}\n"
        "with matching YAML:\n"
        f"  {MAP_PGM_B_DEFAULT.with_suffix('.yaml')}\n\n"
        "Override:  export FORMICA_MAP_B=/path/to/new_map.pgm\n\n"
        "Optional for overlays:\n"
        f"  export EXP3_SLAM_CSV_B=/home/jetson/formica_experiments/data/exp3_slam_<timestamp>.csv\n"
        f"  export EXP3_TRAJECTORY_B=/home/jetson/exp1_logs/exp3_trajectory_<timestamp>.csv\n"
        f"Or save trajectory as: {TRAJ_B_DEFAULT}\n\n"
        f"Then: python3 {BASE}/build_chapter6_from_logs.py\n"
    )


def _scrub_variant_b_figure_files(figures: Path, provenance: dict) -> None:
    """Remove stale (b) PNGs and record why (b) is absent — avoids identical a/b panels."""
    msg = _variant_b_missing_instructions()
    for name in VARIANT_B_FIGURE_PNGS:
        (figures / name).unlink(missing_ok=True)
        (figures / name.replace(".png", "_MISSING.txt")).write_text(msg)
    stub = "MISSING — save new robot map; see corresponding Figure_*_b_*_MISSING.txt in figures/"
    provenance["Figure_6_2b_Map_From_LiDAR_SLAM.png"] = stub
    provenance["Figure_6_4b_Map_Overlay.png"] = stub
    provenance["Figure_6_5b_Exploration_Trajectory.png"] = stub
    provenance["Figure_6_6b_Maze_Protocol_Overlay.png"] = stub


def _clear_variant_b_missing_txts(figures: Path) -> None:
    for name in VARIANT_B_FIGURE_PNGS:
        (figures / name.replace(".png", "_MISSING.txt")).unlink(missing_ok=True)


def _read_map_origin_resolution(map_pgm: Path) -> tuple[float, float, float]:
    """Parse resolution and origin x,y from map_name.yaml next to the .pgm (Nav2 style)."""
    ox, oy, res = -5.9, -2.99, 0.05
    yaml_path = map_pgm.with_suffix(".yaml")
    if not yaml_path.is_file():
        return ox, oy, res
    for line in yaml_path.read_text().splitlines():
        s = line.strip()
        if s.startswith("resolution:"):
            try:
                res = float(s.split(":", 1)[1].strip())
            except ValueError:
                pass
        if s.startswith("origin:"):
            try:
                rest = s.split(":", 1)[1].strip()
                arr = ast.literal_eval(rest)
                ox, oy = float(arr[0]), float(arr[1])
            except (ValueError, SyntaxError, IndexError, TypeError):
                pass
    return ox, oy, res


def _load_occupancy_display(map_pgm: Path) -> tuple[np.ndarray, list[float]]:
    if not map_pgm.is_file():
        raise FileNotFoundError(map_pgm)
    img = plt.imread(str(map_pgm))
    if img.ndim == 3:
        img = img[:, :, 0]
    h, w = img.shape
    ox, oy, res = _read_map_origin_resolution(map_pgm)
    extent = [ox, ox + w * res, oy, oy + h * res]
    return img, extent


def _latest_exp3_slam_csv() -> Path | None:
    for csv_path in sorted(DATA.glob("exp3_slam_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        if not df.empty and "gt_x" in df.columns:
            return csv_path
    return None


def _slam_csv_for_variant_b(lm_canonical: Path | None) -> Path | None:
    """SLAM CSV for 6.4b: explicit env, else a file different from the canonical overlay (if any)."""
    env = os.environ.get("EXP3_SLAM_CSV_B", "").strip()
    if env:
        p = Path(env).expanduser()
        if not p.is_file():
            return None
        try:
            df = pd.read_csv(p)
        except Exception:
            return None
        if df.empty or "gt_x" not in df.columns:
            return None
        return p
    candidates: list[Path] = []
    for csv_path in sorted(DATA.glob("exp3_slam_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        if not df.empty and "gt_x" in df.columns:
            candidates.append(csv_path)
    for p in candidates:
        if lm_canonical is None or p.resolve() != lm_canonical.resolve():
            return p
    return None


PHOTO_CANDIDATES = [
    Path("/home/jetson/my_photo-1.jpg"),
    Path("/home/jetson/my_photo.jpg"),
    Path("/home/jetson/formica_robot.jpg"),
    Path("/home/jetson/formica_robot.png"),
]
AGENT_CAMERA_CANDIDATES: dict[str, list[Path]] = {
    "a": [
        Path("/home/jetson/agent_camera_a.jpg"),
        Path("/home/jetson/agent_camera_a.png"),
        Path("/home/jetson/camera_a.jpg"),
        Path("/home/jetson/camera_a.png"),
    ],
    "b": [
        Path("/home/jetson/agent_camera_b.jpg"),
        Path("/home/jetson/agent_camera_b.png"),
        Path("/home/jetson/camera_b.jpg"),
        Path("/home/jetson/camera_b.png"),
    ],
    "c": [
        Path("/home/jetson/agent_camera_c.jpg"),
        Path("/home/jetson/agent_camera_c.png"),
        Path("/home/jetson/camera_c.jpg"),
        Path("/home/jetson/camera_c.png"),
    ],
    "d": [
        Path("/home/jetson/agent_camera_d.jpg"),
        Path("/home/jetson/agent_camera_d.png"),
        Path("/home/jetson/camera_d.jpg"),
        Path("/home/jetson/camera_d.png"),
    ],
}
EXACT_AGENT_CAMERA_PATHS: dict[str, Path | None] = {
    "a": Path(os.environ["EXP1_CAM_A"]).expanduser() if os.environ.get("EXP1_CAM_A") else None,
    "b": Path(os.environ["EXP1_CAM_B"]).expanduser() if os.environ.get("EXP1_CAM_B") else None,
    "c": Path(os.environ["EXP1_CAM_C"]).expanduser() if os.environ.get("EXP1_CAM_C") else None,
    "d": Path(os.environ["EXP1_CAM_D"]).expanduser() if os.environ.get("EXP1_CAM_D") else None,
}

sys.path.insert(0, str(Path("/home/jetson/formica_experiments")))
from formica_experiments.exp2_postprocess import postprocess_exp2  # noqa: E402
from formica_experiments.exp7_postprocess import postprocess_exp7  # noqa: E402


def _latest_csv(pattern: str) -> Path | None:
    cands = sorted(DATA.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    for p in cands:
        try:
            df = pd.read_csv(p)
        except Exception:
            continue
        if len(df) > 0:
            return p
    return None


def _all_nonempty_csvs(pattern: str) -> list[Path]:
    out = []
    for p in sorted(DATA.glob(pattern)):
        try:
            if len(pd.read_csv(p)) > 0:
                out.append(p)
        except Exception:
            continue
    return out


def ensure_dirs() -> tuple[Path, Path, Path]:
    tables = BASE / "tables"
    figures = BASE / "figures"
    meta = BASE / "meta"
    tables.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)
    meta.mkdir(parents=True, exist_ok=True)
    return tables, figures, meta


def build_table_6_1(tables: Path, provenance: dict) -> None:
    """Prefer the latest non-empty exp1 Table 6.1 CSV from exp1_logs."""
    src = None
    for p in sorted(EXP1.glob("table6_1_exp1_*.csv"), key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            df = pd.read_csv(p)
        except Exception:
            continue
        if len(df) > 0 and {"metric", "value", "unit", "target"} <= set(df.columns):
            src = p
            break
    if src is None:
        # Fallback to legacy calibration CSV if no formatted table is available.
        src = DATA / "exp1_calibration_20260401_200107.csv"
    dst = tables / "Table_6_1_Sensor_Calibration.csv"
    shutil.copy2(src, dst)
    provenance["Table_6_1_Sensor_Calibration.csv"] = str(src.resolve())


def build_exp2(tables: Path, figures: Path, provenance: dict) -> None:
    p = _latest_csv("exp2_power_*.csv")
    if not p:
        (tables / "Table_6_2_Power_Profile_MISSING.txt").write_text(
            "No non-empty exp2_power_*.csv under formica_experiments/data.\n"
        )
        provenance["Table_6_2"] = "MISSING"
        return
    tag = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    res = postprocess_exp2(p, tag=tag, out_dir=BASE / "_exp2_scratch")
    # Copy canonical outputs from scratch (postprocess writes to _exp2_scratch)
    shutil.copy2(BASE / "_exp2_scratch" / "table6_2_power_profile.csv", tables / "Table_6_2_Power_Profile.csv")
    shutil.copy2(BASE / "_exp2_scratch" / "Figure6_3_Power_Profile.png", figures / "Figure_6_3_Power_Profile.png")
    shutil.copy2(BASE / "_exp2_scratch" / "exp2_summary.txt", BASE / "meta" / "exp2_summary.txt")
    provenance["Table_6_2_Power_Profile.csv"] = str(p.resolve())
    provenance["Figure_6_3_Power_Profile.png"] = str(p.resolve())


def build_exp3_slam_summary(tables: Path, figures: Path, provenance: dict) -> None:
    rows = []
    for csv_path in sorted(DATA.glob("exp3_slam_*.csv")):
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            continue
        if df.empty or "error_m" not in df.columns:
            continue
        for _, r in df.iterrows():
            rows.append({"source_file": csv_path.name, **r.to_dict()})
    summary_path = tables / "Table_6_3a_SLAM_Landmark_Errors_All_Logged.csv"
    if not rows:
        summary_path.write_text("source_file,trial,landmark_id,gt_x,gt_y,est_x,est_y,error_m,coverage_pct\n")
        (tables / "Table_6_3a_SLAM_Summary.txt").write_text(
            "No non-empty exp3_slam rows with error_m. RMSE: N/A\n"
        )
        provenance["Table_6_3a"] = "MISSING"
    else:
        all_df = pd.DataFrame(rows)
        all_df.to_csv(summary_path, index=False)
        err = pd.to_numeric(all_df["error_m"], errors="coerce").dropna()
        rmse = float(np.sqrt((err**2).mean())) if len(err) else float("nan")
        (tables / "Table_6_3a_SLAM_Summary.txt").write_text(
            f"landmark_rows,{len(err)}\n"
            f"RMSE_m,{rmse:.6f}\n"
            f"target_RMSE_le_0.15_m,{'PASS' if rmse <= 0.15 else 'FAIL'}\n"
        )
        provenance["Table_6_3a_SLAM_Landmark_Errors_All_Logged.csv"] = "merged exp3_slam_*.csv"

    # Figures 6.4–6.5 (+ subfigure b): occupancy + trajectory
    if not MAP_PGM.is_file():
        (figures / "Figure_6_4_Map_Overlay_MISSING.txt").write_text("formica_map.pgm not found.\n")
        provenance["Figure_6_4"] = "MISSING"
        return

    lm_csv = _latest_exp3_slam_csv()
    lm_csv_b = _slam_csv_for_variant_b(lm_csv)

    def _plot_landmarks(ax, lm: Path | None) -> None:
        if lm is None:
            return
        df = pd.read_csv(lm)
        seen = set()
        for _, r in df.iterrows():
            gt_x = pd.to_numeric(r.get("gt_x"), errors="coerce")
            gt_y = pd.to_numeric(r.get("gt_y"), errors="coerce")
            est_x = pd.to_numeric(r.get("est_x"), errors="coerce")
            est_y = pd.to_numeric(r.get("est_y"), errors="coerce")
            if not (pd.notna(gt_x) and pd.notna(gt_y)):
                continue
            key = (float(gt_x), float(gt_y))
            if key in seen:
                continue
            seen.add(key)
            ax.plot(float(gt_x), float(gt_y), "g^", markersize=10, label="GT landmark" if len(seen) == 1 else "")
            if pd.notna(est_x) and pd.notna(est_y):
                ax.plot(float(est_x), float(est_y), "rx", markersize=8)
        if seen:
            ax.legend(loc="upper right")

    def _save_fig_6_4(map_pgm: Path, lm: Path | None, title: str, fname: str, prov_key: str) -> None:
        img, extent = _load_occupancy_display(map_pgm)
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.imshow(np.flipud(img), cmap="gray_r", extent=extent, origin="lower", alpha=0.9)
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        _plot_landmarks(ax, lm)
        if lm is not None:
            provenance[prov_key] = f"{map_pgm} + {lm.name}"
        else:
            provenance[prov_key] = str(map_pgm)
        fig.tight_layout()
        fig.savefig(figures / fname, dpi=160)
        plt.close(fig)

    def _save_fig_6_5(map_pgm: Path, traj_p: Path, title: str, fname: str, prov_key: str) -> None:
        img, extent = _load_occupancy_display(map_pgm)
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.imshow(np.flipud(img), cmap="gray_r", extent=extent, origin="lower", alpha=0.9)
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        if traj_p.is_file():
            tdf = pd.read_csv(traj_p)
            if not tdf.empty and {"x", "y"} <= set(tdf.columns):
                ax.plot(tdf["x"], tdf["y"], "b-", linewidth=1.2, alpha=0.85)
                provenance[prov_key] = f"{map_pgm} + {traj_p.name}"
            else:
                provenance[prov_key] = f"{map_pgm} (no valid x,y in {traj_p.name})"
        else:
            provenance[prov_key] = f"{map_pgm} (map only; trajectory not found: {traj_p})"
        fig.tight_layout()
        fig.savefig(figures / fname, dpi=160)
        plt.close(fig)

    _save_fig_6_4(
        MAP_PGM,
        lm_csv,
        "Figure 6.4 Occupancy map (saved formica_map) + landmarks (from logged SLAM CSV)",
        "Figure_6_4_Map_Overlay.png",
        "Figure_6_4_Map_Overlay.png",
    )
    if _variant_b_map_ready():
        _save_fig_6_4(
            MAP_PGM_B,
            lm_csv_b,
            "Figure 6.4(b) Occupancy map + landmarks (new map / log; subfigure b)",
            "Figure_6_4b_Map_Overlay.png",
            "Figure_6_4b_Map_Overlay.png",
        )

    _save_fig_6_5(
        MAP_PGM,
        TRAJ,
        "Figure 6.5 Exploration pose trail (logged odometry / pose)",
        "Figure_6_5_Exploration_Trajectory.png",
        "Figure_6_5_Exploration_Trajectory.png",
    )
    if _variant_b_map_ready():
        _save_fig_6_5(
            MAP_PGM_B,
            TRAJ_B,
            "Figure 6.5(b) Exploration pose trail (new map / trajectory; subfigure b)",
            "Figure_6_5b_Exploration_Trajectory.png",
            "Figure_6_5b_Exploration_Trajectory.png",
        )


def build_table_6_3_maze(tables: Path, provenance: dict) -> None:
    """Merge every non-empty exp4 maze log into one thesis table."""
    parts = []
    for p in sorted(DATA.glob("exp4_maze_*.csv"), key=lambda p: p.stat().st_mtime):
        df = pd.read_csv(p)
        if df.empty:
            continue
        df = df.copy()
        df["log_file"] = p.name
        parts.append(df)
    dst = tables / "Table_6_3_Maze_Navigation_All_Trials_Logged.csv"
    if not parts:
        dst.write_text("trial,outcome,path_length_m,time_to_target_s,replan_events,failure_mode,efficiency_pct,log_file\n")
        (tables / "Table_6_3_Maze_Summary.txt").write_text("No maze trial rows logged.\n")
        provenance["Table_6_3_Maze"] = "MISSING"
        return
    merged = pd.concat(parts, ignore_index=True)
    merged.to_csv(dst, index=False)
    trial_rows = merged[merged["trial"].astype(str).str.isdigit()].copy()
    if trial_rows.empty:
        (tables / "Table_6_3_Maze_Summary.txt").write_text("No per-trial rows (numeric trial id).\n")
    else:
        n = len(trial_rows)
        passes = (trial_rows["outcome"].astype(str).str.upper() == "PASS").sum()
        pct = 100.0 * passes / n if n else 0.0
        (tables / "Table_6_3_Maze_Summary.txt").write_text(
            f"total_trial_rows,{n}\n"
            f"pass_count,{int(passes)}\n"
            f"success_rate_pct,{pct:.2f}\n"
            f"target_ge_89_pct,{'PASS' if pct >= 89 else 'FAIL'}\n"
        )
    provenance["Table_6_3_Maze_Navigation_All_Trials_Logged.csv"] = "merged exp4_maze_*.csv"


def build_figure_6_2_lidar_schematic(figures: Path, provenance: dict) -> None:
    """Map-only view (honest caption: occupancy from saved map; not a CAD floor plan)."""
    if not MAP_PGM.is_file():
        return
    img, extent = _load_occupancy_display(MAP_PGM)
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.imshow(np.flipud(img), cmap="bone", extent=extent, origin="lower")
    ax.set_title("Figure 6.2 Occupancy grid from mapping (use CAD overlay in thesis if required)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(figures / "Figure_6_2_Map_From_LiDAR_SLAM.png", dpi=160)
    plt.close(fig)
    provenance["Figure_6_2_Map_From_LiDAR_SLAM.png"] = str(MAP_PGM)

    if _variant_b_map_ready():
        img_b, extent_b = _load_occupancy_display(MAP_PGM_B)
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.imshow(np.flipud(img_b), cmap="bone", extent=extent_b, origin="lower")
        ax.set_title("Figure 6.2(b) Occupancy grid (new SLAM map; subfigure b)")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        fig.tight_layout()
        fig.savefig(figures / "Figure_6_2b_Map_From_LiDAR_SLAM.png", dpi=160)
        plt.close(fig)
        provenance["Figure_6_2b_Map_From_LiDAR_SLAM.png"] = str(MAP_PGM_B)


def build_figure_6_1_photo(figures: Path, provenance: dict) -> None:
    # Prefer a 2x2 panel for the four agent camera views (a,b,c,d).
    panel_sources: dict[str, Path | None] = {}
    for tag, cands in AGENT_CAMERA_CANDIDATES.items():
        exact = EXACT_AGENT_CAMERA_PATHS.get(tag)
        if exact is not None and exact.is_file():
            panel_sources[tag] = exact
        else:
            panel_sources[tag] = next((p for p in cands if p.is_file()), None)

    fig, axes = plt.subplots(2, 2, figsize=(10.5, 7.6))
    axes_flat = axes.flatten()
    labels = ["a", "b", "c", "d"]
    used_sources: list[str] = []
    for idx, tag in enumerate(labels):
        ax = axes_flat[idx]
        src = panel_sources[tag]
        if src is not None:
            img = plt.imread(str(src))
            ax.imshow(img)
            used_sources.append(str(src.resolve()))
        else:
            ax.set_facecolor("#eceff1")
            ax.text(
                0.5,
                0.5,
                f"Robot camera view {tag.upper()}\nDifferent angle demo",
                ha="center",
                va="center",
                fontsize=12,
            )
        ax.text(
            0.03,
            0.95,
            f"({tag})",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=16,
            fontweight="bold",
            color="white" if src is not None else "black",
            bbox={"facecolor": "black" if src is not None else "white", "alpha": 0.68, "pad": 2},
        )
        ax.set_title(f"Viewpoint ({tag})", fontsize=12, pad=8)
        ax.axis("off")

    fig.suptitle("Figure 6.1 Robot camera views from different angles", y=0.985, fontsize=15, fontweight="bold")
    fig.text(
        0.5,
        0.015,
        "Subfigures: (a), (b), (c), (d) represent different agent viewpoints",
        ha="center",
        va="bottom",
        fontsize=11,
    )
    fig.tight_layout(rect=[0.02, 0.04, 0.98, 0.95], pad=1.0)
    out_photo = figures / "Figure_6_1_Robot_Photo.png"
    out_placeholder = figures / "Figure_6_1_Robot_Photo_PLACEHOLDER.png"
    fig.savefig(out_photo, dpi=160)
    plt.close(fig)
    out_placeholder.unlink(missing_ok=True)
    provenance[out_photo.name] = (
        "; ".join(used_sources) if used_sources else "generated 2x2 camera panel placeholders"
    )
    return

    # Fallback to a single robot photograph when camera panel images do not exist.
    chosen = next((p for p in PHOTO_CANDIDATES if p.is_file()), None)
    out_photo = figures / "Figure_6_1_Robot_Photo.png"
    out_placeholder = figures / "Figure_6_1_Robot_Photo_PLACEHOLDER.png"

    if chosen is not None:
        img = plt.imread(str(chosen))
        fig, ax = plt.subplots(figsize=(8, 5))
        ax.imshow(img)
        ax.axis("off")
        ax.set_title("Figure 6.1 FormicaBot hardware photograph")
        fig.tight_layout()
        fig.savefig(out_photo, dpi=160)
        plt.close(fig)
        out_placeholder.unlink(missing_ok=True)
        provenance[out_photo.name] = str(chosen.resolve())
        return

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.axis("off")
    ax.text(
        0.5,
        0.55,
        "Figure 6.1 — FormicaBot hardware photograph\n(placeholder: add your annotated photo in thesis)",
        ha="center",
        va="center",
        fontsize=12,
    )
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    fig.savefig(out_placeholder, dpi=120)
    plt.close(fig)
    out_photo.unlink(missing_ok=True)
    provenance[out_placeholder.name] = "generated placeholder (no photo on disk)"


def build_exp5_exp6_notes(tables: Path, figures: Path, provenance: dict) -> None:
    table64 = tables / "Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt"
    exp5_files = _all_nonempty_csvs("exp5_fault_*.csv") + _all_nonempty_csvs("exp5_obstacle_fault_*.csv")
    if not exp5_files:
        baseline_pct = "N/A"
        maze_summary = tables / "Table_6_3_Maze_Summary.txt"
        if maze_summary.is_file():
            for line in maze_summary.read_text().splitlines():
                if line.startswith("success_rate_pct,"):
                    baseline_pct = line.split(",", 1)[1].strip()
                    break
        table64.write_text(
            "\n".join(
                [
                    "metric,value,target,pass_or_status,notes",
                    f"baseline_exp4_success_pct,{baseline_pct},reference,LOGGED,from Table_6_3_Maze_Summary",
                    "condition_A_reroute_success_pct,N/A,>=80%,MISSING_DATA,no exp5_fault rows",
                    "condition_A_mean_detect_latency_s,N/A,report,MISSING_DATA,no exp5_fault rows",
                    "condition_B_overall_success_pct,N/A,>=60%,MISSING_DATA,no exp5_fault rows",
                    "condition_B_lidar_success_pct,N/A,report,MISSING_DATA,no exp5_fault rows",
                    "condition_B_camera_success_pct,N/A,report,MISSING_DATA,no exp5_fault rows",
                    "condition_B_line_sensor_success_pct,N/A,report,MISSING_DATA,no exp5_fault rows",
                    "condition_B_mean_recovery_time_s,N/A,<=30s preferred,MISSING_DATA,no exp5_fault rows",
                ]
            )
            + "\n"
        )
        provenance["Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt"] = "template generated (no exp5_fault_*.csv found)"
    else:
        parts = []
        for p in exp5_files:
            df = pd.read_csv(p)
            if df.empty:
                continue
            df = df.copy()
            df["log_file"] = p.name
            parts.append(df)
        if not parts:
            table64.write_text("metric,value,target,pass_or_status,notes\nexp5_data_status,EMPTY,required,FAIL,exp5 files contain headers only\n")
            provenance["Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt"] = "exp5 files exist but empty"
        else:
            df = pd.concat(parts, ignore_index=True)
            # Normalize fields
            cond = df["condition"].astype(str).str.lower() if "condition" in df.columns else pd.Series([], dtype=str)
            out = df["outcome"].astype(str).str.lower() if "outcome" in df.columns else pd.Series([], dtype=str)
            is_success = out.eq("success")
            is_valid_outcome = out.isin(["success", "fail"])
            detect_latency = pd.to_numeric(df.get("detect_latency_s"), errors="coerce")
            recovery_time = pd.to_numeric(df.get("recovery_time_s"), errors="coerce")
            perturb = df.get("perturbation", pd.Series([""] * len(df))).astype(str).str.lower()

            a_mask = cond.str.startswith("a")
            b_mask = cond.str.startswith("b")
            a_total = int(a_mask.sum())
            b_total = int(b_mask.sum())
            a_valid = a_mask & is_valid_outcome
            b_valid = b_mask & is_valid_outcome
            a_valid_n = int(a_valid.sum())
            b_valid_n = int(b_valid.sum())
            a_invalid_n = a_total - a_valid_n
            b_invalid_n = b_total - b_valid_n

            a_succ_pct = (100.0 * is_success[a_valid].mean()) if a_valid_n else float("nan")
            b_succ_pct = (100.0 * is_success[b_valid].mean()) if b_valid_n else float("nan")
            a_lat_mean = float(detect_latency[a_valid].dropna().mean()) if a_valid_n else float("nan")
            b_rec_mean = float(recovery_time[b_valid].dropna().mean()) if b_valid_n else float("nan")

            def pct_for_sensor(tag: str) -> float:
                m = b_valid & perturb.str.contains(tag)
                return (100.0 * is_success[m].mean()) if int(m.sum()) else float("nan")

            lidar_pct = pct_for_sensor("lidar")
            cam_pct = pct_for_sensor("camera")
            line_pct = pct_for_sensor("line")

            def fmt(v: float) -> str:
                return f"{v:.2f}" if pd.notna(v) else "N/A"

            def rate_status(value: float, threshold: float, total_n: int, valid_n: int) -> str:
                if total_n == 0:
                    return "MISSING_DATA"
                if valid_n == 0:
                    return "INVALID_RUN"
                return "PASS" if pd.notna(value) and value >= threshold else "FAIL"

            a_rate_status = rate_status(a_succ_pct, 80.0, a_total, a_valid_n)
            b_no_rows_status = "INCOMPLETE_RUN" if (b_total == 0 and a_total > 0) else "MISSING_DATA"
            b_rate_status = rate_status(b_succ_pct, 60.0, b_total, b_valid_n) if b_total > 0 else b_no_rows_status
            if b_total == 0:
                rec_status = b_no_rows_status
            elif b_valid_n == 0:
                rec_status = "INVALID_RUN"
            else:
                rec_status = "PASS" if pd.notna(b_rec_mean) and b_rec_mean <= 30 else "CHECK"
            a_lat_status = "LOGGED" if pd.notna(a_lat_mean) else ("MISSING_DATA" if a_total == 0 else "INVALID_RUN")

            a_rate_notes = f"rows={a_total}, valid={a_valid_n}, invalid={a_invalid_n}"
            b_rate_notes = f"rows={b_total}, valid={b_valid_n}, invalid={b_invalid_n}"
            sensor_note_suffix = f"(from valid B rows={b_valid_n})"

            table64.write_text(
                "\n".join(
                    [
                        "metric,value,target,pass_or_status,notes",
                        f"condition_A_reroute_success_pct,{fmt(a_succ_pct)},>=80%,{a_rate_status},{a_rate_notes}",
                        f"condition_A_mean_detect_latency_s,{fmt(a_lat_mean)},report,{a_lat_status},from detect_latency_s (valid A rows only)",
                        f"condition_B_overall_success_pct,{fmt(b_succ_pct)},>=60%,{b_rate_status},{b_rate_notes}",
                        f"condition_B_lidar_success_pct,{fmt(lidar_pct)},report,{'LOGGED' if pd.notna(lidar_pct) else (b_no_rows_status if b_total == 0 else 'INVALID_RUN')},perturbation contains lidar {sensor_note_suffix}",
                        f"condition_B_camera_success_pct,{fmt(cam_pct)},report,{'LOGGED' if pd.notna(cam_pct) else (b_no_rows_status if b_total == 0 else 'INVALID_RUN')},perturbation contains camera {sensor_note_suffix}",
                        f"condition_B_line_sensor_success_pct,{fmt(line_pct)},report,{'LOGGED' if pd.notna(line_pct) else (b_no_rows_status if b_total == 0 else 'INVALID_RUN')},perturbation contains line {sensor_note_suffix}",
                        f"condition_B_mean_recovery_time_s,{fmt(b_rec_mean)},<=30s preferred,{rec_status},from recovery_time_s (valid B rows only)",
                    ]
                )
                + "\n"
            )
            provenance["Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt"] = "merged exp5_fault_*.csv"

    cnn_files = _all_nonempty_csvs("exp6_cnn_*.csv")
    if not cnn_files:
        (tables / "Table_6_5_CNN_Detection.txt").write_text(
            "No non-empty exp6_cnn_*.csv found. mAP / confusion matrix not computed.\n"
        )
        (figures / "Figure_6_8_Confusion_Matrix_MISSING.txt").write_text("No CNN trial rows.\n")
        provenance["Table_6_5"] = "MISSING"
        provenance["Figure_6_8"] = "MISSING"
        return
    # If any file has data rows beyond header, merge
    parts = []
    for p in cnn_files:
        df = pd.read_csv(p)
        if len(df) < 1:
            continue
        df = df.copy()
        df["log_file"] = p.name
        parts.append(df)
    if not parts:
        (tables / "Table_6_5_CNN_Detection.txt").write_text("exp6 files exist but contain headers only.\n")
        provenance["Table_6_5"] = "EMPTY_HEADERS"
        return
    merged = pd.concat(parts, ignore_index=True)
    merged.to_csv(tables / "Table_6_5_CNN_Detection_All_Logged.csv", index=False)
    provenance["Table_6_5_CNN_Detection_All_Logged.csv"] = "merged exp6_cnn_*.csv"
    # Clear stale placeholders from earlier missing-data runs.
    (tables / "Table_6_5_CNN_Detection.txt").unlink(missing_ok=True)
    (figures / "Figure_6_8_Confusion_Matrix_MISSING.txt").unlink(missing_ok=True)

    # Figure 6.8: AP heatmap by lighting condition and class.
    # If AP is entirely zero in the logs, fall back to miss-rate (%) heatmap
    # from TP/FN so the figure still carries useful information.
    ap_df = merged.copy()
    ap_df["condition"] = ap_df["condition"].astype(str)
    ap_df["class_name"] = ap_df["class_name"].astype(str)
    ap_df["AP"] = pd.to_numeric(ap_df["AP"], errors="coerce")
    ap_df = ap_df.dropna(subset=["AP"])
    ap_df = ap_df[~ap_df["condition"].str.upper().eq("OVERALL")]

    if ap_df.empty:
        (figures / "Figure_6_8_Confusion_Matrix_MISSING.txt").write_text("No valid AP rows in exp6 data.\n")
        provenance["Figure_6_8"] = "MISSING_AP_ROWS"
        return

    heat = ap_df.pivot_table(index="condition", columns="class_name", values="AP", aggfunc="mean").sort_index()
    vals = heat.fillna(0.0).to_numpy(dtype=float)

    fig, ax = plt.subplots(figsize=(7.8, 5.0))
    if float(np.nanmax(vals)) <= 1e-9:
        miss_df = merged.copy()
        miss_df["condition"] = miss_df["condition"].astype(str)
        miss_df["class_name"] = miss_df["class_name"].astype(str)
        miss_df["TP"] = pd.to_numeric(miss_df.get("TP"), errors="coerce")
        miss_df["FN"] = pd.to_numeric(miss_df.get("FN"), errors="coerce")
        miss_df = miss_df[~miss_df["condition"].str.upper().eq("OVERALL")]
        denom = (miss_df["TP"].fillna(0.0) + miss_df["FN"].fillna(0.0)).replace(0.0, np.nan)
        miss_df["miss_pct"] = 100.0 * miss_df["FN"].fillna(0.0) / denom
        miss_heat = miss_df.pivot_table(index="condition", columns="class_name", values="miss_pct", aggfunc="mean").sort_index()
        miss_vals = miss_heat.fillna(0.0).to_numpy(dtype=float)

        im = ax.imshow(miss_vals, cmap="magma", vmin=0.0, vmax=100.0)
        ax.set_title("Figure 6.8 Exp6 Detection Miss Rate (%) by Condition and Class")
        ax.set_xlabel("Class")
        ax.set_ylabel("Condition")
        ax.set_xticks(np.arange(len(miss_heat.columns)))
        ax.set_xticklabels(list(miss_heat.columns), rotation=20, ha="right")
        ax.set_yticks(np.arange(len(miss_heat.index)))
        ax.set_yticklabels(list(miss_heat.index))
        for i in range(miss_vals.shape[0]):
            for j in range(miss_vals.shape[1]):
                ax.text(j, i, f"{miss_vals[i, j]:.1f}%", ha="center", va="center", color="white", fontsize=9)
        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("Miss rate (%)")
        ax.text(
            0.5,
            -0.16,
            "Fallback used because AP is zero across all logged Exp6 trials.",
            transform=ax.transAxes,
            ha="center",
            va="top",
            fontsize=9,
        )
        provenance["Figure_6_8_Confusion_Matrix.png"] = "miss-rate fallback from merged exp6_cnn_*.csv (AP all zero)"
    else:
        im = ax.imshow(vals, cmap="viridis", vmin=0.0, vmax=1.0)
        ax.set_title("Figure 6.8 Exp6 AP Heatmap by Condition and Class")
        ax.set_xlabel("Class")
        ax.set_ylabel("Condition")
        ax.set_xticks(np.arange(len(heat.columns)))
        ax.set_xticklabels(list(heat.columns), rotation=20, ha="right")
        ax.set_yticks(np.arange(len(heat.index)))
        ax.set_yticklabels(list(heat.index))
        for i in range(vals.shape[0]):
            for j in range(vals.shape[1]):
                ax.text(j, i, f"{vals[i, j]:.2f}", ha="center", va="center", color="white", fontsize=9)
        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label("AP")
        provenance["Figure_6_8_Confusion_Matrix.png"] = "merged exp6_cnn_*.csv"

    fig.tight_layout()
    fig.savefig(figures / "Figure_6_8_Confusion_Matrix.png", dpi=160)
    plt.close(fig)


def build_exp7(tables: Path, figures: Path, provenance: dict) -> None:
    p = _latest_csv("exp7_pheromone_*.csv")
    all_p = _all_nonempty_csvs("exp7_pheromone_*.csv")
    if not p or not all_p:
        (tables / "Table_6_6_Pheromone_Trials.txt").write_text("No non-empty exp7_pheromone_*.csv\n")
        provenance["exp7"] = "MISSING"
        return

    parts = []
    for src in all_p:
        df_src = pd.read_csv(src)
        if df_src.empty:
            continue
        df_src = df_src.copy()
        df_src["log_file"] = src.name
        parts.append(df_src)
    merged = pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()

    # Figures 6.9 / 6.10 from merged A/B rows.
    def _plot_lateral(sub_tag: str, title: str, out_name: str) -> None:
        if merged.empty:
            return
        sub = merged[merged["sub_exp"].astype(str).str.startswith(sub_tag)].copy()
        if sub.empty:
            return
        sub["dist_m"] = pd.to_numeric(sub.get("dist_m"), errors="coerce")
        sub["dev_cm"] = pd.to_numeric(sub.get("lateral_dev_m"), errors="coerce").abs() * 100.0
        sub = sub.dropna(subset=["dist_m", "dev_cm"])
        if sub.empty:
            return
        trend = sub.groupby("dist_m", as_index=False)["dev_cm"].mean().sort_values("dist_m")
        q95 = float(sub["dev_cm"].quantile(0.95))

        fig, ax = plt.subplots(figsize=(8.6, 4.8))
        ax.scatter(sub["dist_m"], sub["dev_cm"], s=18, alpha=0.28, color="#1f77b4", label="Samples")
        ax.plot(trend["dist_m"], trend["dev_cm"], color="#d62728", linewidth=2.0, label="Mean trend")
        ax.fill_between(
            trend["dist_m"].to_numpy(dtype=float),
            0.0,
            trend["dev_cm"].to_numpy(dtype=float),
            color="#d62728",
            alpha=0.08,
        )
        ax.axhline(q95, color="#ff7f0e", linestyle="--", linewidth=1.2, label=f"95th pct = {q95:.3f} cm")
        ax.set_title(title)
        ax.set_xlabel("Distance along trail (m)")
        ax.set_ylabel("Absolute lateral deviation (cm)")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", framealpha=0.95)
        ax.text(
            0.01,
            0.01,
            f"merged rows={len(sub)}, files={len(all_p)}",
            transform=ax.transAxes,
            ha="left",
            va="bottom",
            fontsize=8,
        )
        fig.tight_layout()
        fig.savefig(figures / out_name, dpi=170)
        plt.close(fig)
        provenance[out_name] = "merged exp7_pheromone_*.csv"

    _plot_lateral(
        "A_straight",
        "Figure 6.9 Pheromone Straight Trail: Lateral Deviation",
        "Figure_6_9_Pheromone_Straight_Lateral_Deviation.png",
    )
    _plot_lateral(
        "B_curved",
        "Figure 6.10 Pheromone Curved Trail: Lateral Deviation",
        "Figure_6_10_Pheromone_Curved_Lateral_Deviation.png",
    )

    # Figure 6.12 from merged D_decay rows.
    if not merged.empty:
        d = merged[merged["sub_exp"].astype(str).str.startswith("D_decay")].copy()
        if not d.empty:
            d["led_pwm"] = pd.to_numeric(d.get("led_pwm"), errors="coerce")
            for ch in ["s0", "s1", "s2", "s3"]:
                d[ch] = pd.to_numeric(d.get(ch), errors="coerce")
            d["mean_adc"] = d[["s0", "s1", "s2", "s3"]].mean(axis=1)
            d = d.dropna(subset=["led_pwm", "mean_adc"])
            if not d.empty:
                agg = d.groupby("led_pwm", as_index=False)["mean_adc"].mean().sort_values("led_pwm")
                fig, ax = plt.subplots(figsize=(7.8, 4.6))
                ax.scatter(d["led_pwm"], d["mean_adc"], s=22, alpha=0.30, color="#2ca02c", label="Samples")
                ax.plot(agg["led_pwm"], agg["mean_adc"], color="#9467bd", linewidth=2.2, label="Mean by PWM")
                if len(agg) >= 2:
                    coef = np.polyfit(agg["led_pwm"].to_numpy(dtype=float), agg["mean_adc"].to_numpy(dtype=float), 1)
                    xx = np.linspace(float(agg["led_pwm"].min()), float(agg["led_pwm"].max()), 80)
                    yy = coef[0] * xx + coef[1]
                    ax.plot(xx, yy, color="#d62728", linestyle="--", linewidth=1.4, label="Linear fit")
                ax.set_title("Figure 6.12 LED PWM vs Mean ADC (Merged Decay Trials)")
                ax.set_xlabel("LED PWM duty ratio (0-1)")
                ax.set_ylabel("Mean ADC (channels s0-s3)")
                ax.grid(True, alpha=0.25)
                ax.legend(loc="best", framealpha=0.95)
                ax.text(
                    0.01,
                    0.01,
                    f"merged rows={len(d)}, files={len(all_p)}",
                    transform=ax.transAxes,
                    ha="left",
                    va="bottom",
                    fontsize=8,
                )
                fig.tight_layout()
                fig.savefig(figures / "Figure_6_12_LED_PWM_vs_Mean_ADC.png", dpi=170)
                plt.close(fig)
                provenance["Figure_6_12_LED_PWM_vs_Mean_ADC.png"] = "merged exp7_pheromone_*.csv"
    # Table 6.7: merge all logged switchover rows across exp7 files.
    if not merged.empty:
        c_rows = merged[merged["sub_exp"].astype(str).str.startswith("C_snr")].copy()
        if not c_rows.empty:
            out_c = c_rows[["trial", "ambient_pct", "switchover_latency_s", "continued_following_or_trail_any", "log_file"]].copy()
            out_c["switchover_latency_s"] = pd.to_numeric(out_c["switchover_latency_s"], errors="coerce")
            out_c = out_c.sort_values(["log_file", "trial"]).reset_index(drop=True)
            out_c.to_csv(tables / "Table_6_7_SNR_Switchover_Logged.csv", index=False)
            provenance["Table_6_7_SNR_Switchover_Logged.csv"] = "merged exp7_pheromone_*.csv (C_snr rows)"

    # Table 6.6: richer summary with quality targets.
    summ_lines = [f"sources_count,{len(all_p)}", f"latest_source,{p.name}", ""]
    for tag, title, target in [("A_straight", "straight", 3.0), ("B_curved", "curved", 5.0)]:
        sub = merged[merged["sub_exp"].astype(str).str.startswith(tag)] if not merged.empty else pd.DataFrame()
        if sub.empty:
            continue
        dev_cm = pd.to_numeric(sub["lateral_dev_m"], errors="coerce").abs() * 100.0
        mean_cm = float(dev_cm.mean())
        max_cm = float(dev_cm.max())
        status = "PASS" if mean_cm <= target else "FAIL"
        summ_lines.append(f"{title}_samples,{int(dev_cm.notna().sum())}")
        summ_lines.append(f"{title}_mean_abs_lateral_dev_cm,{mean_cm:.4f}")
        summ_lines.append(f"{title}_max_abs_lateral_dev_cm,{max_cm:.4f}")
        summ_lines.append(f"{title}_target_mean_le_{target:.1f}_cm,{status}")

    if not merged.empty:
        optical = merged[merged["modality"].astype(str).str.lower().eq("optical")]
        if not optical.empty:
            snr_db = pd.to_numeric(optical["snr_db"], errors="coerce")
            summ_lines.append(f"optical_snr_mean_db,{snr_db.mean():.2f}")
            summ_lines.append(f"optical_snr_min_db,{snr_db.min():.2f}")

    (tables / "Table_6_6_Pheromone_Lateral_Summary.txt").write_text("\n".join(summ_lines) + "\n")
    provenance["exp7_source"] = "merged exp7_pheromone_*.csv"


def build_figure_6_6_maze(figures: Path, provenance: dict) -> None:
    """Best-effort: map + straight line S→T (no pose log available)."""
    if not MAP_PGM.is_file():
        return

    def _save_maze(map_pgm: Path, title: str, fname: str, prov_key: str) -> None:
        img, extent = _load_occupancy_display(map_pgm)
        fig, ax = plt.subplots(figsize=(8, 7))
        ax.imshow(np.flipud(img), cmap="gray_r", extent=extent, origin="lower", alpha=0.9)
        S = (0.0, 0.0)
        T = (3.5, 2.5)
        ax.plot([S[0], T[0]], [S[1], T[1]], "c--", linewidth=2, label="Euclidean S→T (protocol)")
        ax.plot(S[0], S[1], "go", markersize=12, label="Start S")
        ax.plot(T[0], T[1], "r*", markersize=16, label="Target T")
        ax.set_title(title)
        ax.legend(loc="upper right")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        fig.tight_layout()
        fig.savefig(figures / fname, dpi=160)
        plt.close(fig)
        provenance[prov_key] = f"{map_pgm} + protocol S,T"

    _save_maze(
        MAP_PGM,
        "Figure 6.6 Map with protocol start / goal (add measured path when Nav2 logs exist)",
        "Figure_6_6_Maze_Protocol_Overlay.png",
        "Figure_6_6_Maze_Protocol_Overlay.png",
    )
    if _variant_b_map_ready():
        _save_maze(
            MAP_PGM_B,
            "Figure 6.6(b) Map with protocol S→T (new map; subfigure b)",
            "Figure_6_6b_Maze_Protocol_Overlay.png",
            "Figure_6_6b_Maze_Protocol_Overlay.png",
        )


def build_figure_6_7_note(figures: Path, provenance: dict) -> None:
    exp5_files = _all_nonempty_csvs("exp5_fault_*.csv") + _all_nonempty_csvs("exp5_obstacle_fault_*.csv")
    parts = []
    for p in exp5_files:
        df = pd.read_csv(p)
        if df.empty:
            continue
        df = df.copy()
        df["log_file"] = p.name
        parts.append(df)

    if parts:
        df = pd.concat(parts, ignore_index=True)
        df["condition"] = df.get("condition", pd.Series([""] * len(df))).astype(str).str.lower()
        df["outcome"] = df.get("outcome", pd.Series(["unknown"] * len(df))).astype(str).str.lower()
        df["trial_num"] = pd.to_numeric(df.get("trial"), errors="coerce")
        df["seq"] = np.arange(1, len(df) + 1)
        df["perturbation_norm"] = df.get("perturbation", pd.Series(["unknown"] * len(df))).astype(str).str.lower()

        outcome_colors = {
            "success": "#2E7D32",
            "fail": "#D32F2F",
            "goal_rejected": "#616161",
            "timeout": "#EF6C00",
        }

        fig, axes = plt.subplots(2, 1, figsize=(11.5, 6.2), sharex=True, gridspec_kw={"height_ratios": [1, 1.2]})

        a_rows = df[df["condition"].str.startswith("a")].copy()
        if not a_rows.empty:
            for out, grp in a_rows.groupby("outcome"):
                axes[0].scatter(
                    grp["seq"],
                    grp["trial_num"],
                    s=40,
                    marker="s",
                    color=outcome_colors.get(out, "#9E9E9E"),
                    alpha=0.9,
                )
            axes[0].set_ylabel("A trial #")
            axes[0].set_title("Condition A: Dynamic Obstacle Trials")
            axes[0].set_ylim(0.5, max(10.5, float(a_rows["trial_num"].max()) + 0.5))
            axes[0].grid(True, axis="x", alpha=0.25)
        else:
            axes[0].text(0.5, 0.5, "No Condition A rows", transform=axes[0].transAxes, ha="center", va="center")
            axes[0].set_yticks([])

        b_rows = df[df["condition"].str.startswith("b")].copy()
        if not b_rows.empty:
            order = ["lidar", "camera", "linesensor", "dynamic_box", "inject", "unknown"]
            pert_vals = [p for p in order if (b_rows["perturbation_norm"] == p).any()]
            for p in sorted(set(b_rows["perturbation_norm"]) - set(pert_vals)):
                pert_vals.append(p)
            y_map = {p: i for i, p in enumerate(pert_vals)}
            b_rows["pert_y"] = b_rows["perturbation_norm"].map(y_map).fillna(len(y_map))
            for out, grp in b_rows.groupby("outcome"):
                axes[1].scatter(
                    grp["seq"],
                    grp["pert_y"],
                    s=40,
                    marker="s",
                    color=outcome_colors.get(out, "#9E9E9E"),
                    alpha=0.9,
                )
            axes[1].set_yticks(list(y_map.values()))
            axes[1].set_yticklabels([k.title() for k in y_map.keys()])
            axes[1].set_ylabel("B perturbation")
            axes[1].set_title("Condition B: Sensor-Failure Trials")
            axes[1].grid(True, axis="x", alpha=0.25)
        else:
            axes[1].text(0.5, 0.5, "No Condition B rows", transform=axes[1].transAxes, ha="center", va="center")
            axes[1].set_yticks([])

        axes[1].set_xlabel("Chronological event index across merged Exp5 logs")
        fig.suptitle("Figure 6.7 Dynamic Obstacle / Fault-Tolerance Timeline (Logged Trials)", y=0.98)

        legend_order = [k for k in ["success", "fail", "goal_rejected", "timeout"] if (df["outcome"] == k).any()]
        for k in legend_order:
            axes[0].scatter([], [], s=50, marker="s", color=outcome_colors[k], label=k.upper())
        if legend_order:
            axes[0].legend(loc="upper right", framealpha=0.9, title="Outcome")

        succ_n = int((df["outcome"] == "success").sum())
        fail_n = int((df["outcome"] == "fail").sum())
        rej_n = int((df["outcome"] == "goal_rejected").sum())
        fig.text(
            0.01,
            0.01,
            f"Merged logs: files={len(exp5_files)}, rows={len(df)}, success={succ_n}, fail={fail_n}, goal_rejected={rej_n}",
            ha="left",
            va="bottom",
            fontsize=8,
        )

        fig.tight_layout()
        fig.savefig(figures / "Figure_6_7_Dynamic_Obstacle_Timeline.png", dpi=170)
        plt.close(fig)
        (figures / "Figure_6_7_Dynamic_Obstacle_Timeline_PLACEHOLDER.txt").unlink(missing_ok=True)
        provenance["Figure_6_7_Dynamic_Obstacle_Timeline.png"] = "timeline from merged exp5_fault_*.csv"
        return

    # Fallback template when no exp5 rows are available.
    fig, ax = plt.subplots(figsize=(10, 3.8))
    ax.set_title("Figure 6.7 Dynamic obstacle / fault-tolerance timeline")
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("State")
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.grid(True, axis="x", alpha=0.25)
    segments = [
        ("Normal navigation", 0, 20, "#4CAF50"),
        ("Obstacle encounter", 20, 35, "#FFC107"),
        ("Replan/recovery", 35, 55, "#03A9F4"),
        ("Fault event (motor/sensor)", 55, 70, "#F44336"),
        ("Recovered mission", 70, 100, "#4CAF50"),
    ]
    for label, x0, x1, color in segments:
        ax.axvspan(x0, x1, ymin=0.35, ymax=0.65, color=color, alpha=0.25)
        ax.text((x0 + x1) / 2.0, 0.5, label, ha="center", va="center", fontsize=9)
    ax.set_xlim(0, 100)
    ax.text(
        0.01,
        0.06,
        "Template figure: replace with measured exp5 timeline when obstacle/fault logs are available.",
        transform=ax.transAxes,
        fontsize=8,
        ha="left",
        va="bottom",
    )
    fig.tight_layout()
    fig.savefig(figures / "Figure_6_7_Dynamic_Obstacle_Timeline.png", dpi=160)
    plt.close(fig)
    (figures / "Figure_6_7_Dynamic_Obstacle_Timeline_PLACEHOLDER.txt").unlink(missing_ok=True)
    provenance["Figure_6_7_Dynamic_Obstacle_Timeline.png"] = "generated template (no exp5 timeline CSV found)"


def write_provenance(meta: Path, provenance: dict) -> None:
    lines = [
        "# Chapter 6 deliverables — data provenance",
        "",
        "Generated (UTC): " + datetime.now(timezone.utc).isoformat(),
        "",
        "Each output was produced from local logs only. Rows in tables reflect what was actually recorded.",
        "",
        "## File → source",
        "",
    ]
    for k, v in sorted(provenance.items()):
        lines.append(f"- **{k}**: {v}")
    (meta / "PROVENANCE.md").write_text("\n".join(lines) + "\n")

    (meta / "sources_index.json").write_text(json.dumps(provenance, indent=2))


def main() -> int:
    tables, figures, meta = ensure_dirs()
    provenance: dict[str, str] = {}

    if _variant_b_map_ready():
        _clear_variant_b_missing_txts(figures)
    else:
        _scrub_variant_b_figure_files(figures, provenance)

    build_figure_6_1_photo(figures, provenance)
    build_figure_6_2_lidar_schematic(figures, provenance)
    build_table_6_1(tables, provenance)
    build_exp2(tables, figures, provenance)
    build_exp3_slam_summary(tables, figures, provenance)
    build_table_6_3_maze(tables, provenance)
    build_figure_6_6_maze(figures, provenance)
    build_figure_6_7_note(figures, provenance)
    build_exp5_exp6_notes(tables, figures, provenance)
    build_exp7(tables, figures, provenance)

    write_provenance(meta, provenance)

    # Clean scratch
    for d in (BASE / "_exp2_scratch", BASE / "_exp7_scratch"):
        if d.is_dir():
            shutil.rmtree(d, ignore_errors=True)

    print(f"Wrote deliverables under {BASE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
