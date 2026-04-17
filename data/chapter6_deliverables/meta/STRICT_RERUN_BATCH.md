# Chapter 6 Strict Rerun Batch (Experiments 1-7)

This runbook is for collecting thesis-grade fresh data on robot hardware and then regenerating all Chapter 6 tables/figures.

## 0) Pre-check (once)

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
ros2 daemon stop || true
ros2 daemon start
```

Optional device checks:

```bash
ls -la /dev/serial/by-id
i2cdetect -y 1
```

## 1) Experiment 1 (Sensor Calibration)

Run:

```bash
python3 /home/jetson/exp1_logs/run_exp1_hardware_integration.py
```

Notes:
- Follow physical prompts exactly (wall distances, odom marks, LED on/off).
- For RGB-D reprojection, rerun with value once you have checkerboard result:

```bash
python3 /home/jetson/exp1_logs/run_exp1_hardware_integration.py --rgbd-reprojection 0.32
```

## 2) Experiment 2 (Power Profiling)

Quick test (2 min):

```bash
MISSION_DURATION_S=120 bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh
```

Thesis run (6 h):

```bash
bash /home/jetson/exp1_logs/run_exp2_full_pipeline.sh
```

## 3) Experiment 3 (SLAM Strict)

Run:

```bash
bash /home/jetson/exp1_logs/run_exp3_full_strict.sh
```

Monitor:

```bash
tail -f /home/jetson/exp1_logs/exp3_run_*.log
```

## 4) Experiment 4 (Maze Trials)

Run full trials:

```bash
EXP4_NUM_TRIALS=20 EXP4_TRIAL_TIMEOUT_S=120 bash /home/jetson/exp1_logs/run_exp4_stack.sh --with-trials
```

## 5) Experiment 5 (Dynamic Obstacle + Sensor Fault)

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
ros2 run formica_experiments exp5_fault
```

Important:
- This experiment is interactive and requires physical obstacle insertion + sensor kill timing.
- Do not skip manual actions; otherwise data is not thesis-valid.

## 6) Experiment 6 (CNN Recognition)

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
ros2 run formica_experiments jetson_camera --ros-args -p topic_name:=/rgb/image_raw
```

In second terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ros2_ws/install/setup.bash
ros2 run formica_experiments exp6_cnn
```

Important:
- Requires full 45 combinations (3 lighting x 3 classes x 5 distances) x 30 events each.
- Make sure target objects and distances are physically correct.

## 7) Experiment 7 (Pheromone)

Run:

```bash
bash /home/jetson/exp1_logs/run_exp7_full_pipeline.sh
```

## 8) Rebuild Chapter 6 Deliverables

```bash
python3 /home/jetson/chapter6_formica_deliverables/build_chapter6_from_logs.py
```

## 9) Final Verification Files

Check these first:

- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_1_Sensor_Calibration.csv`
- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_2_Power_Profile.csv`
- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_3a_SLAM_Summary.txt`
- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_3_Maze_Summary.txt`
- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_4_Dynamic_Obstacle_and_Fault_Tolerance.txt`
- `/home/jetson/chapter6_formica_deliverables/tables/Table_6_5_CNN_Detection_All_Logged.csv`
- `/home/jetson/chapter6_formica_deliverables/meta/PROVENANCE.md`

