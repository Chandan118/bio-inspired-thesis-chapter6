# Chapter 6 deliverables — data provenance (UPDATED)

Generated (UTC): 2026-04-16T11:38:06.654620
Last Updated: 2026-04-16 (Response to MAJOR REVISION)

## Major Changes in This Revision

### Simulation B (Scalability) - NEW
- **Source**: simulation_B_scalability_*.csv (appeal_experiments/data/raw/)
- **Fix**: Corrected trial-duration from 6000 to 36000 steps
- **Results**: Meaningful foraging output demonstrated

### Simulation C (Fault Tolerance) - UPDATED
- **Source**: simulation_C_fault_tolerance_*.csv (appeal_experiments/data/raw/)
- **Fix**: Corrected trial-duration from 6000 to 36000 steps
- **Results**: Valid fault tolerance metrics now available

### Experiment 7D (Pheromone) - HARDWARE FIX
- **Change**: 620nm LEDs replaced with 940nm LEDs
- **Validation**: exp7D_940nm_*.csv (appeal_experiments/data/raw/)
- **Results**: Spectral mismatch issue resolved

## Original File → source

### Figures
- **Figure_6_10_Pheromone_Curved_Lateral_Deviation.png**: merged exp7_pheromone_*.csv
- **Figure_6_12_LED_PWM_vs_Mean_ADC.png**: exp7D_940nm_validation_*.png (NEW)
- **Figure_6_1_Robot_Photo.png**: /home/jetson/agent_camera_*.jpg
- **Figure_6_2_Map_From_LiDAR_SLAM.png**: /home/jetson/formica_map.pgm
- **Figure_6_3_Power_Profile.png**: exp2_power_20260409_000628.csv
- **Figure_6_4_Map_Overlay.png**: formica_map.pgm + exp3_slam_*.csv
- **Figure_6_5_Exploration_Trajectory.png**: formica_map.pgm + exp3_trajectory_*.csv
- **Figure_6_7_Dynamic_Obstacle_Timeline.png**: timeline from merged exp5_fault_*.csv
- **Figure_6_8_Confusion_Matrix.png**: miss-rate from merged exp6_cnn_*.csv

### Tables
- **Table_6_1_Sensor_Calibration.csv**: /home/jetson/exp1_logs/table6_1_exp1_20260409_075758.csv
- **Table_6_2_Power_Profile.csv**: /home/jetson/formica_experiments/data/exp2_power_20260409_000628.csv
- **Table_6_3_Maze_Navigation_All_Trials_Logged.csv**: merged exp4_maze_*.csv
- **Table_6_3a_SLAM_Landmark_Errors_All_Logged.csv**: merged exp3_slam_*.csv
- **Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt**: UPDATED with valid results
- **Table_6_4b_Scalability_Simulation_B.txt**: NEW (2026-04-16)
- **Table_6_5_CNN_Detection_All_Logged.csv**: merged exp6_cnn_*.csv
- **Table_6_6_Pheromone_Lateral_Summary.txt**: UPDATED with 940nm validation
- **Table_6_7_SNR_Switchover_Logged.csv**: merged exp7_pheromone_*.csv

### Appeal Experiment Data (NEW)
- /home/jetson/appeal_experiments/data/raw/simulation_B_scalability_*.csv
- /home/jetson/appeal_experiments/data/raw/simulation_C_fault_tolerance_*.csv
- /home/jetson/appeal_experiments/data/raw/exp7D_940nm_results_*.csv
- /home/jetson/appeal_experiments/data/raw/exp7D_940nm_validation_*.png

## Review Response Status

| Issue | Status | Notes |
|-------|--------|-------|
| Simulation B Near-Zero | ✓ FIXED | Duration corrected to 1 hour |
| Simulation C Near-Zero | ✓ FIXED | Duration corrected to 1 hour |
| 7D Spectral Mismatch | ✓ FIXED | 940nm LED validated |
| Name Discrepancy | ✓ FIXED | Corrected to Chandan Sheikder |
| Loihi 2 Clarification | ✓ DONE | Documented as proposed |
