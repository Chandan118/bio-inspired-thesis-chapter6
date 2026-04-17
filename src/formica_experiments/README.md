# formica_experiments

Chapter 6 Experimental Validation Code for FormicaBot - A Single-Robot Platform Based on NVIDIA Jetson Orin Nano 8GB and ROS 2 Humble.

## Project Overview

This repository contains the experimental validation code for Chapter 6 of the FormicaBot thesis. It implements seven distinct experiments (Exp 1-7) designed to validate the robot's capabilities across multiple domains including sensor calibration, power profiling, SLAM mapping, maze navigation, fault tolerance, CNN-based target recognition, and pheromone trail following.

## Robot Platform

- **Hardware**: NVIDIA Jetson Orin Nano 8GB
- **ROS Version**: ROS 2 Humble
- **Sensors**: RPLIDAR A1, Arduino base with line sensors, Azure Kinect camera, IMU (LPMS IG1)
- **Actuators**: Differential drive base with motor control via Arduino

## Quick Start

```bash
# Clone and setup
cd ~/formica_experiments

# Hardware check
./scripts/chapter6_experiment_runner.sh check

# Bringup the robot
./scripts/chapter6_experiment_runner.sh bringup

# Run any experiment
./scripts/chapter6_experiment_runner.sh exp1  # Sensor calibration
./scripts/chapter6_experiment_runner.sh exp2  # Power profiling
./scripts/chapter6_experiment_runner.sh exp3  # SLAM mapping
./scripts/chapter6_experiment_runner.sh exp4  # Maze navigation
./scripts/chapter6_experiment_runner.sh exp5  # Obstacle & fault tolerance
./scripts/chapter6_experiment_runner.sh exp6  # CNN target detection
./scripts/chapter6_experiment_runner.sh exp7  # Pheromone trail
```

## Package Structure

```
formica_experiments/
├── formica_experiments/     # Main Python package
│   ├── __init__.py
│   ├── data_logger.py       # Shared CSV logger
│   ├── exp1_sensor_calibration.py
│   ├── exp2_power_profiling.py
│   ├── exp3_slam_mapping.py
│   ├── exp4_maze_navigation.py
│   ├── exp5_obstacle_fault.py
│   ├── exp6_cnn_detection.py
│   ├── exp7_pheromone_trail.py
│   ├── arduino_base_node.py
│   ├── cmd_vel_relay.py
│   ├── mapping_motion_helper.py
│   ├── hardware_checker.py
│   └── ...
├── launch/                  # ROS 2 launch files
│   ├── bringup_launch.py
│   ├── nav_stack_launch.py
│   └── ...
├── config/                  # YAML configurations
│   ├── nav2_params.yaml
│   └── slam_params.yaml
├── scripts/                # Helper scripts
│   ├── chapter6_experiment_runner.sh
│   └── detect_formica_ports.py
├── data/                   # Experiment output (CSV files)
├── package.xml
├── setup.py
└── README.md
```

## Experiments Summary

| Exp | Name | Description | Output |
|-----|------|-------------|--------|
| 1 | Sensor Calibration | LiDAR, IMU, camera, line/gas sensors calibration | CSV |
| 2 | Power Profiling | INA219 power monitoring over mission duration | CSV |
| 3 | SLAM Mapping | Online SLAM with landmark detection | CSV + map |
| 4 | Maze Navigation | Nav2 autonomous navigation in known map | CSV |
| 5 | Fault Tolerance | Obstacle avoidance and sensor failure handling | CSV |
| 6 | CNN Detection | TensorRT-based target recognition | CSV |
| 7 | Pheromone Trail | LED trail following with ethanol detection | CSV |

## Building

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select formica_experiments
source install/setup.bash
```

## Data Output

All experiment data is saved to `~/formica_experiments/data/` as timestamped CSV files. Each experiment uses the `CsvLogger` utility from `data_logger.py` to ensure consistent formatting.

## Citation

If you use this code in your research, please cite:

```
@software{formica_experiments,
  title = {FormicaBot Chapter 6 Experimental Validation},
  author = {Chandan Sheik},
  year = {2026}
}
```

## License

MIT License