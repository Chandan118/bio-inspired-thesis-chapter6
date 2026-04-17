# Bio-Inspired Cross-Species Navigator - Chapter 6 Experimental Validation

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions/workflows/ci.yml/badge.svg)](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions)
[![Validate](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions/workflows/validate.yml/badge.svg)](https://github.com/Chandan118/bio-inspired-thesis-chapter6/actions)

## Overview

This repository contains the complete experimental validation data, simulation code, and analysis tools for **Chapter 6** of the Bio-Inspired Cross-Species Navigator thesis. It validates the FormicaBot platform (hexapod robot) based on NVIDIA Jetson Orin Nano 8GB and ROS 2 Humble.

## Project Structure

```
bio-inspired-thesis-chapter6/
├── src/
│   ├── formica_experiments/           # ROS 2 Humble experiment package
│   │   ├── formica_experiments/        # Experiment scripts (Exp 1-7)
│   │   ├── launch/                     # ROS 2 launch files
│   │   ├── config/                     # Configuration files
│   │   └── hardware_checker.py         # Hardware validation
│   └── appeal_experiments/             # Appeal response simulation code
│       ├── simulation_A_fast.py       # Baseline foraging
│       ├── simulation_B_scalability.py # Scalability analysis
│       └── simulation_C_fault_tolerance.py
├── data/
│   ├── chapter6_deliverables/          # Generated figures and tables
│   │   ├── tables/                     # Table 6.1 - 6.7
│   │   └── figures/                    # Figure 6.1 - 6.12
│   ├── raw_logs/                       # Raw experiment logs
│   ├── simulation_data/                # Simulation outputs
│   └── aliengo_model/                   # Aliengo quadruped comparison
├── docs/                               # Documentation
├── workflows/                          # GitHub Actions CI/CD
├── README.md
└── LICENSE (MIT)
```

## Experiments Summary (Chapter 6)

| Exp | Name | Description | Output |
|-----|------|-------------|--------|
| 1 | Sensor Calibration | LiDAR, IMU, camera, line/gas sensors | Table 6.1 |
| 2 | Power Profiling | INA219 power monitoring | Table 6.2, Fig 6.3 |
| 3 | SLAM Mapping | Online SLAM with landmark detection | Table 6.3a, Fig 6.2, 6.4, 6.5 |
| 4 | Maze Navigation | Nav2 autonomous navigation | Table 6.3, Fig 6.6 |
| 5 | Fault Tolerance | Obstacle avoidance, sensor failure | Table 6.4, Fig 6.7 |
| 6 | CNN Detection | TensorRT-based target recognition | Table 6.5, Fig 6.8 |
| 7 | Pheromone Trail | LED trail following with ethanol detection | Table 6.6-7, Fig 6.9-12 |

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

### Hardware Requirements
- NVIDIA Jetson Orin Nano 8GB
- ROS 2 Humble
- FormicaBot hardware (LiDAR, IMU, camera, gas sensors)

### Run Experiments
```bash
# Install ROS 2 dependencies
source /opt/ros/humble/setup.bash

# Hardware check
python3 src/formica_experiments/hardware_checker.py --quick

# Experiment 1 - Sensor Calibration
python3 src/formica_experiments/formica_experiments/exp1_sensor_calibration.py

# Experiment 2 - Power Profiling
python3 src/formica_experiments/formica_experiments/exp2_power_profiling.py

# Experiment 3 - SLAM Mapping
python3 src/formica_experiments/formica_experiments/exp3_slam_mapping.py

# Experiment 4 - Maze Navigation
python3 src/formica_experiments/formica_experiments/exp4_maze_navigation.py

# Experiment 5 - Fault Tolerance
python3 src/formica_experiments/formica_experiments/exp5_obstacle_fault.py

# Experiment 6 - CNN Detection
python3 src/formica_experiments/formica_experiments/exp6_cnn_detection.py

# Experiment 7 - Pheromone Trail
python3 src/formica_experiments/formica_experiments/exp7_pheromone_trail.py
```

### Run Simulations
```bash
cd src/appeal_experiments

# Simulation A - Baseline foraging
python3 simulation_A_fast.py

# Simulation B - Scalability (1000-10000 robots)
python3 simulation_B_scalability_fixed.py

# Simulation C - Fault tolerance
python3 simulation_C_fast.py
```

## Data Provenance

All experimental data is traced in `data/chapter6_deliverables/meta/PROVENANCE.md`.

## CI/CD

This project uses GitHub Actions for continuous integration:
- **CI**: Lint and basic validation
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
