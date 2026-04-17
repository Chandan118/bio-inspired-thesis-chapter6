#!/usr/bin/env python3
"""
regenerate_tables_real.py
=========================
只用真实传感器数据生成论文表格。

从exp1_run_details_*.txt和exp2_summary.txt提取真实测量值。
没有真实数据的实验，标注为 DATA_MISSING 并说明原因。

真实数据来源：
  /home/jetson/exp1_logs/exp1_run_details_20260409_075758.txt  — 最佳运行
  /home/jetson/exp1_logs/exp1_run_details_20260409_073214.txt  — 第二次运行
  /home/jetson/exp1_logs/exp1_run_details_20260402_060735.txt  — 第三次运行（LiDAR数据异常）
  /home/jetson/exp1_logs/exp2_summary.txt  — 功率测量（运行6h）

Run:
    python3 /home/jetson/exp1_logs/regenerate_tables_real.py
"""

import csv
import datetime
import math
import os
import shutil
import statistics

OUT_DIR = '/home/jetson/exp1_logs'
os.makedirs(OUT_DIR, exist_ok=True)
TAG = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    print(f'Written: {path}')


# --------------------------------------------------------------------------
# 真实数据（从日志提取）
# --------------------------------------------------------------------------

# Exp1 里程计数据 — 两次运行的真实测量
ODOM_TRIALS_RUN1 = [5.3810, 5.6523, 5.6520, 6.0761, 5.6486,
                     5.8294, 5.4857, 5.8260, 5.6827, 5.7811]
ODOM_TRIALS_RUN2 = [5.4390, 5.6414, 5.4361, 5.6804, 5.4200,
                     5.1057, 5.4600, 5.8028, 5.4257, 5.6130]
ODOM_MEAN_RUN1 = statistics.mean(ODOM_TRIALS_RUN1)   # 5.7015%
ODOM_SD_RUN1   = statistics.stdev(ODOM_TRIALS_RUN1)  # 0.1932%
ODOM_MEAN_RUN2 = statistics.mean(ODOM_TRIALS_RUN2)   # 5.5024%
ODOM_SD_RUN2   = statistics.stdev(ODOM_TRIALS_RUN2)  # 0.1926%

# Exp1 IMU漂移 — 两次运行的真实测量
IMU_DRIFT_RUN1 = 0.01411881  # deg/min (best run, 20260409_075758)
IMU_DRIFT_RUN2 = 0.00494385  # deg/min (20260409_073214)

# Exp1 Odom — 运行1的10次试验数据
ODOM_RUN1_DIST  = [2.1076, 2.1130, 2.1130, 2.1215, 2.1130,
                   2.1166, 2.1097, 2.1165, 2.1137, 2.1156]
ODOM_TARGET_M   = 2.0000

# Exp1 LiDAR — 运行3 (20260402) 的真实测量
# 机器人始终在0.728m处（错误位置），所有4个距离都测到相同值
# gt=0.50 → measured=0.728  err=0.228
# gt=1.00 → measured=0.728  err=0.272
# gt=1.50 → measured=0.728  err=0.772
# gt=2.00 → measured=0.728  err=1.272
LIDAR_L3_ERRORS = [0.2283, 0.2720, 0.7719, 1.2721]
LIDAR_L3_RMSE   = math.sqrt(statistics.mean([e*e for e in LIDAR_L3_ERRORS]))

# Exp2 功率 — 真实INA219测量（运行6h，数据文件 exp2_power_20260409_000628.csv）
# overall_mean_W,overall_sd_W,overall_peak_W,overall_min_W
# TRANSIT mode mean ≈ 5.64 W (motors + RPLIDAR)
# DECISION mode mean ≈ 6.70 W (+ camera + CNN)
# STANDBY mode mean ≈ 6.20 W (Jetson idle + IMU)
# OVERALL mean = 6.012 W

# Exp2 各模式真实功率数据（从 exp2_summary.txt + 日志分析）
# 注意：Thesis目标 ≤1.2W只适用于纯STANDBY（无ROS2栈）
# 实际FormicaBot在ROS2运行：STANDBY ≈ 6.2W（Jetson Nano满载）
TRANSIT_MEAN  = 5.639
TRANSIT_SD    = 0.3343
TRANSIT_PEAK   = 7.3066
TRANSIT_MIN    = 5.3747

DECISION_MEAN = 6.6956
DECISION_SD   = 0.4554
DECISION_PEAK  = 7.7486
DECISION_MIN   = 5.7468

STANDBY_MEAN  = 6.1963
STANDBY_SD    = 0.5441
STANDBY_PEAK  = 7.513
STANDBY_MIN   = 5.6228

OVERALL_MEAN  = 6.0122
OVERALL_SD    = 0.6179
OVERALL_PEAK  = 7.7486
OVERALL_MIN   = 5.3747

# Exp4 迷宫导航 — 所有运行都失败，nav2无法生成有效路径
# 原因：AMCLLocalisation位置与地图不匹配（-20m外），
# 导致 "None of the X points of the global plan were in the local costmap"


# --------------------------------------------------------------------------
# Table 6.1: 传感器标定（只用真实测量）
# --------------------------------------------------------------------------

def generate_table6_1():
    # 使用最佳运行的数据（20260409_075758）
    # 里程计用两次运行的平均值
    odom_mean_combined = statistics.mean([ODOM_MEAN_RUN1, ODOM_MEAN_RUN2])
    # SD取两次运行的标准差的pooled估计
    pooled_sd = math.sqrt((ODOM_SD_RUN1**2 + ODOM_SD_RUN2**2) / 2)

    rows = [
        ['metric', 'value', 'unit', 'target', 'pass_fail', 'notes'],
        ['LiDAR_RMSE', '', 'm', '<= 0.02', 'DATA_MISSING',
         'Robot not placed at wall; all 4 distances returned 0.728 m (L3 run). Re-run with robot correctly positioned.'],
        ['IMU_Drift', round(IMU_DRIFT_RUN1, 6), 'deg/min', '<= 0.5', 'PASS',
         f'bias_rad_s=-4.11e-06, 1000 samples, run 20260409_075758'],
        ['Odom_Mean_Error', round(odom_mean_combined, 4), '%', '<= 2.0', 'FAIL',
         f'{len(ODOM_TRIALS_RUN1)+len(ODOM_TRIALS_RUN2)} trials combined, wheel circumference needs recalibration'],
        ['Odom_SD', round(pooled_sd, 4), '%', 'report only', 'N/A',
         'pooled SD of all 20 odom trials'],
        ['RGBD_Reprojection', '', 'px', '<= 0.5', 'DATA_MISSING',
         'Azure Kinect camera not connected. Re-run ros2 run camera_calibration with /rgb/image_raw active.'],
        ['TCRT5000_SNR', '', 'dB', '>= 6.0', 'DATA_MISSING',
         'LED strip not powered on; sensor returned 0. Power on 620nm LED strip and re-run Task 6.'],
    ]

    path = f'{OUT_DIR}/table6_1_exp1_real_{TAG}.csv'
    write_csv(path, rows)
    return path


# --------------------------------------------------------------------------
# Table 6.2: 功率配置（真实INA219测量）
# --------------------------------------------------------------------------

def generate_table6_2():
    rows = [
        ['row', 'mean_W', 'sd_W', 'peak_W', 'min_W'],
        ['TRANSIT (this work)',    TRANSIT_MEAN,  TRANSIT_SD,   TRANSIT_PEAK,  TRANSIT_MIN],
        ['DECISION (this work)',   DECISION_MEAN, DECISION_SD,  DECISION_PEAK, DECISION_MIN],
        ['STANDBY (this work)',    STANDBY_MEAN,  STANDBY_SD,   STANDBY_PEAK,  STANDBY_MIN],
        ['OVERALL (this work)',     OVERALL_MEAN,  OVERALL_SD,   OVERALL_PEAK,  OVERALL_MIN],
        ['Kilobot (benchmark)',     'N/A', 'N/A', 'N/A', 'N/A'],
        ['EPFL Tribots (benchmark)','N/A', 'N/A', 'N/A', 'N/A'],
    ]
    path = f'{OUT_DIR}/table6_2_power_profile_real_{TAG}.csv'
    write_csv(path, rows)
    return path


# --------------------------------------------------------------------------
# Table 6.3: 迷宫导航（无有效数据）
# --------------------------------------------------------------------------

def generate_table6_3():
    rows = [
        ['trial', 'outcome', 'path_length_m', 'time_to_target_s',
         'replan_events', 'failure_mode', 'efficiency_pct'],
        ['1', 'DATA_MISSING', '', '', '',
         'nav2_status_6: AMCL pose (-20.7,-7.8) far from map origin; '
         'global planner generates 0 poses. Robot requires manual relocation to map origin for AMCL initialisation.', ''],
        ['2', 'DATA_MISSING', '', '', '',
         'nav2_status_6: same localisation failure; all 20 trials in this run affected.', ''],
        ['3', 'DATA_MISSING', '', '', '',
         'nav2_status_6: same localisation failure.', ''],
    ]
    path = f'{OUT_DIR}/table6_3_maze_navigation_real_{TAG}.csv'
    write_csv(path, rows)
    return path


# --------------------------------------------------------------------------
# Table 7-C: 信息素SNR切换（无有效数据）
# --------------------------------------------------------------------------

def generate_table7_C():
    rows = [
        ['trial', 'ambient_pct', 'switchover_latency_s', 'continued_following_or_trail_any'],
        ['1', 'DATA_MISSING', 'DATA_MISSING', 'DATA_MISSING',
         'Exp7 was not completed. TCRT5000 returned 0 (LED strip off); '
         'MQ-135 gas_sensor topic not active (/gas_sensor seen=False). '
         'Re-run with LED strip powered and MQ-135 Arduino firmware uploaded.'],
    ]
    path = f'{OUT_DIR}/table7_C_snr_switchover_real_{TAG}.csv'
    write_csv(path, rows)
    return path


# --------------------------------------------------------------------------
# 运行
# --------------------------------------------------------------------------

def main():
    print('=' * 70)
    print('  Regenerating thesis tables — REAL DATA ONLY')
    print(f'  Tag: {TAG}')
    print('=' * 70)
    print()
    print('真实数据来源:')
    print('  Exp1: exp1_run_details_20260409_075758.txt (best run)')
    print('         exp1_run_details_20260409_073214.txt (second run)')
    print('         exp1_run_details_20260402_060735.txt (L3 — LiDAR position error)')
    print('  Exp2: exp2_summary.txt + exp2_power_20260409_000628.csv (INA219, 6h run)')
    print('  Exp4: All nav4 logs — nav2 could not localise (AMCL pose -20m from map)')
    print('  Exp7: No valid trial data — LED strip off, gas_sensor inactive')
    print()
    print('=' * 70)

    t61 = generate_table6_1()
    print(f'  Table 6.1  → {t61}')
    print()
    t62 = generate_table6_2()
    print(f'  Table 6.2  → {t62}')
    print()
    t63 = generate_table6_3()
    print(f'  Table 6.3  → {t63}')
    print()
    t7c = generate_table7_C()
    print(f'  Table 7-C  → {t7c}')
    print()
    print('=' * 70)
    print('  完成 — 只使用真实传感器数据')
    print('=' * 70)

    # Also overwrite the canonical files
    canonical = {
        'table6_1_exp1_real.csv': t61,
        'table6_2_power_profile_real.csv': t62,
        'table6_3_maze_navigation_real.csv': t63,
        'table7_C_snr_switchover_real.csv': t7c,
    }
    print()
    print('Canonical copies:')
    for canon_name, src in canonical.items():
        dst = os.path.join(OUT_DIR, canon_name)
        shutil.copy2(src, dst)
        print(f'  {dst}')


if __name__ == '__main__':
    main()
