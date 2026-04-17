# Chapter 6 Experiment Audit (Auto-Checked)

Generated from current logs and deliverables under `/home/jetson/chapter6_formica_deliverables`.

## Status Summary (Latest Batch)

- **Experiment 1 (Hardware/Sensor calibration):** PARTIAL/FAIL
- **Experiment 2 (Power profiling):** DATA PRESENT, TARGET FAIL
- **Experiment 3 (SLAM mapping):** PASS
- **Experiment 4 (Maze navigation):** FAIL
- **Experiment 5 (Dynamic obstacle/fault):** MISSING REAL DATA
- **Experiment 6 (CNN recognition):** DATA PRESENT, AP CURRENTLY 0.0
- **Experiment 7 (Virtual pheromone):** DATA PRESENT (latest batch used quick mock mode)

## Per-Experiment Detail

### Experiment 1

- Source: `table6_1_exp1_20260409_075758.csv`
- LiDAR RMSE = missing (`nan`) -> FAIL
- IMU drift = `0.014119 deg/min` (target `<= 0.5`) -> PASS
- Odom mean error = `5.7015%` (target `<= 2%`) -> FAIL
- RGB-D reprojection = missing -> FAIL
- TCRT5000 SNR = missing (`nan`) -> FAIL
- **Conclusion:** Not thesis-valid; rerun with full physical protocol + checkerboard reprojection value.

### Experiment 2

- Source: `exp2_power_20260409_000628.csv`
- Overall mean power = `6.0122 W` (target `<= 1.2 W`) -> FAIL
- Figure generated: `Figure_6_3_Power_Profile.png`
- **Conclusion:** Data collected, but target not met.

### Experiment 3

- Source: `Table_6_3a_SLAM_Summary.txt`
- RMSE = `0.086822 m` (target `<= 0.15 m`) -> PASS
- Figures present: `Figure_6_2`, `Figure_6_4`, `Figure_6_5`
- **Conclusion:** Passes RMSE threshold with current logs.

### Experiment 4

- Source: `Table_6_3_Maze_Summary.txt`
- Success rate = `0.00%` (target `>= 89%`) -> FAIL
- Latest added run included 3 new trials; all failed (`nav2_status_6`).
- **Conclusion:** Not thesis-valid; requires successful full 20-trial run.

### Experiment 5

- Source: `Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt`
- No exp5 obstacle/fault CSV logs found.
- Figure present: `Figure_6_7_Dynamic_Obstacle_Timeline.png` (template only)
- **Conclusion:** Real experiment data still missing.

### Experiment 6

- Source: `Table_6_5_CNN_Detection_All_Logged.csv`
- Rows exist and figure is generated, but AP values are currently `0.0`.
- Figure present: `Figure_6_8_Confusion_Matrix.png`
- **Conclusion:** Pipeline works, but result is not thesis-valid until full protocol with correct object/lighting setup succeeds.

### Experiment 7

- Sources: `exp7_pheromone_20260409_001352.csv`, `Table_6_6_Pheromone_Lateral_Summary.txt`, `Table_6_7_SNR_Switchover_Logged.csv`
- Straight mean abs lateral deviation: `0.0406 cm`
- Curved mean abs lateral deviation: `0.0720 cm`
- SNR switchover latency row logged (`0.0037 s`)
- **Conclusion:** Data and figures updated; latest batch was quick mock mode (1 trial each), so perform full physical trial counts for final thesis claim.

## Figure 6.1 Update

- `Figure_6_1_Robot_Photo.png` now uses real image: `/home/jetson/my_photo-1.jpg`
- Placeholder file is no longer used when a real photo exists.

## What Must Be Re-Run for Strict Final Thesis Claims

1. **Exp1** complete physical calibration (LiDAR RMSE, RGB-D reprojection, TCRT SNR).
2. **Exp2** achieve lower mean power (target `<= 1.2 W`) or revise claim.
3. **Exp4** full 20-trial success validation to reach target efficiency.
4. **Exp5** run dynamic obstacle + sensor-failure trials and produce logs.
5. **Exp6** full 45-condition capture with valid detections/mAP.
6. **Exp7** full non-mock trial counts (10/10/10 + full decay session) for final report.

## Notes

- This audit is intentionally strict: figures can exist even when protocol targets are not yet met.
- Provenance mapping remains in `meta/PROVENANCE.md`.
