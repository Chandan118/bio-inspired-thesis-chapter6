# Bio-Inspired Cross-Species Navigator - Chapter 6 Experimental Validation

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions/workflows/ci.yml/badge.svg)](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions)
[![Validate](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions/workflows/validate.yml/badge.svg)](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions)

## Overview

This repository contains the complete experimental validation data, simulation code, and analysis tools for **Chapter 6** of the Bio-Inspired Cross-Species Navigator thesis.

**Robot Platform**: FormicaBot (hexapod robot) based on NVIDIA Jetson Orin Nano 8GB

## Dual ROS Version Support

This project supports both **ROS 1 Noetic** and **ROS 2 Humble**:

```
src/
├── formica_experiments/              # ROS 2 Humble package
│   ├── formica_experiments/         # exp1-exp7 scripts (rclpy)
│   ├── launch/                      # ROS 2 launch files
│   └── config/                      # ROS 2 config
└── formica_experiments_ros1/        # ROS 1 Noetic package
    ├── scripts/                      # exp1-exp7 scripts (rospy)
    ├── launch/                      # ROS 1 launch files
    └── config/                      # ROS 1 config
```

## Project Structure

```
bio-inspired-thesis-chapter6/
├── src/
│   ├── formica_experiments/           # ROS 2 Humble
│   │   ├── formica_experiments/        # Experiment scripts (Exp 1-7)
│   │   ├── launch/                     # ROS 2 launch files
│   │   └── config/                     # Configuration files
│   └── formica_experiments_ros1/       # ROS 1 Noetic
│       ├── scripts/                     # Experiment scripts (Exp 1-7)
│       ├── launch/                     # ROS 1 launch files
│       └── config/                     # Configuration files
├── data/
│   ├── chapter6_deliverables/          # Generated figures and tables
│   │   ├── tables/                     # Table 6.1 - 6.7
│   │   └── figures/                   # Figure 6.1 - 6.12
│   ├── raw_logs/                       # Raw experiment logs
│   ├── simulation_data/                # Simulation outputs
│   └── aliengo_model/                  # Aliengo quadruped comparison
├── docs/                               # Documentation
├── README.md
└── LICENSE (MIT)
```

## Experiments Summary (Chapter 6)

| Exp | Name | Description | ROS 1 | ROS 2 | Output |
|-----|------|-------------|-------|-------|--------|
| 1 | Sensor Calibration | LiDAR, IMU, camera, line/gas sensors | ✅ | ✅ | Table 6.1 |
| 2 | Power Profiling | INA219 power monitoring | ✅ | ✅ | Table 6.2, Fig 6.3 |
| 3 | SLAM Mapping | Online SLAM with landmark detection | ✅ | ✅ | Table 6.3a, Fig 6.2, 6.4, 6.5 |
| 4 | Maze Navigation | Nav2 autonomous navigation | ✅ | ✅ | Table 6.3, Fig 6.6 |
| 5 | Fault Tolerance | Obstacle avoidance, sensor failure | ✅ | ✅ | Table 6.4, Fig 6.7 |
| 6 | CNN Detection | TensorRT-based target recognition | ✅ | ✅ | Table 6.5, Fig 6.8 |
| 7 | Pheromone Trail | LED trail following with ethanol detection | ✅ | ✅ | Table 6.6-7, Fig 6.9-12 |

## Robot Platforms Compared

### FormicaBot (Primary - Hexapod)
- **Weight**: 3 kg
- **DOF**: 18 (3 per leg × 6 legs)
- **Cost**: ~$500
- **Power**: 15W
- **Terrain Adaptability**: Very High

### Aliengo (Baseline - Quadruped)
- **Weight**: 24 kg
- **DOF**: 12 (3 per leg × 4 legs)
- **Cost**: ~$20,000
- **Power**: 150W
- **Terrain Adaptability**: High

See `data/aliengo_model/` for full specifications and comparison data.

## Quick Start

### ROS 2 Humble (Recommended)
```bash
# Install ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select formica_experiments
source install/setup.bash

# Hardware check
ros2 run formica_experiments hardware_checker

# Run experiments
ros2 run formica_experiments exp1_sensor_calibration
ros2 run formica_experiments exp2_power_profiling
ros2 run formica_experiments exp3_slam_mapping
ros2 run formica_experiments exp4_maze_navigation
ros2 run formica_experiments exp5_obstacle_fault
ros2 run formica_experiments exp6_cnn_detection
ros2 run formica_experiments exp7_pheromone_trail
```

### ROS 1 Noetic
```bash
# Install ROS 1 Noetic
source /opt/ros/noetic/setup.bash

# Build with catkin
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Hardware check
rosrun formica_experiments hardware_checker.py

# Run experiments
rosrun formica_experiments exp1_sensor_calibration.py
rosrun formica_experiments exp2_power_profiling.py
rosrun formica_experiments exp3_slam_mapping.py
rosrun formica_experiments exp4_maze_navigation.py
rosrun formica_experiments exp5_obstacle_fault.py
rosrun formica_experiments exp6_cnn_detection.py
rosrun formica_experiments exp7_pheromone_trail.py
```

## ROS Topics

### ROS 2 (Humble)
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | RPLIDAR A1M8 |
| `/imu/data` | sensor_msgs/Imu | MPU6050 IMU |
| `/odom` | nav_msgs/Odometry | Wheel odometry |
| `/line_sensors` | std_msgs/Float32MultiArray | TCRT5000 line sensors |
| `/gas_sensor` | std_msgs/Float32 | MQ-3 ethanol sensor |
| `/rgb/image_raw` | sensor_msgs/Image | Azure Kinect camera |

### ROS 1 (Noetic)
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | RPLIDAR A1M8 |
| `/imu/data` | sensor_msgs/Imu | MPU6050 IMU |
| `/odom` | nav_msgs/Odometry | Wheel odometry |
| `/line_sensors` | std_msgs/Float32MultiArray | TCRT5000 line sensors |
| `/gas_sensor` | std_msgs/Float32 | MQ-3 ethanol sensor |
| `/rgb/image_raw` | sensor_msgs/Image | Azure Kinect camera |

## Data Provenance

All experimental data is traced in `data/chapter6_deliverables/meta/PROVENANCE.md`.

## CI/CD

This project uses GitHub Actions for continuous integration:
- **CI**: Lint and basic validation (ROS 1 + ROS 2)
- **Build**: Generate Chapter 6 deliverables
- **Validate**: Verify experiment files and data integrity

## Citation

```bibtex
@software{formica_chapter6,
  title = {FormicaBot Chapter 6 Experimental Validation},
  author = {Chandan Sheikder},
  year = {2026},
  url = {https://github.com/Chandan118/bio-inspired-thesis-chapter6}
}
```

## License

MIT License - See LICENSE file.

## Author

**Chandan Sheikder**  
Bio-Inspired Robotics Research  
2026

<!-- Status: Complete -->