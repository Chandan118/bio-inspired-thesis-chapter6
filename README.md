# Bio-Inspired Cross-Species Navigator — Chapter 6 Experimental Validation

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

This repository contains the complete experimental validation data, simulation code, and analysis tools for **Chapter 6** of the Bio-Inspired Cross-Species Navigator thesis. It validates a single-robot platform (FormicaBot) based on NVIDIA Jetson Orin Nano 8GB and ROS 2 Humble.

## Project Structure

```
bio-inspired-thesis-chapter6/
├── src/
│   ├── formica_experiments/     # ROS 2 Humble experiment code (Exp 1-7)
│   └── appeal_experiments/      # Appeal response simulation code
├── data/
│   ├── chapter6_deliverables/   # Generated figures and tables
│   ├── raw_logs/                # Raw experiment logs
│   └── simulation_data/         # Simulation scripts and outputs
├── docs/                        # Documentation
├── workflows/                   # GitHub Actions CI/CD
├── README.md
└── LICENSE
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

## Simulation Experiments

- **Simulation A**: Baseline foraging efficiency
- **Simulation B**: Scalability analysis (1000-10000 robots)
- **Simulation C**: Fault tolerance validation
- **Simulation D**: Leader election protocols

## Quick Start

### Hardware Setup
```bash
# Clone repository
cd ~/bio-inspired-thesis-chapter6/src/formica_experiments

# Hardware check
python3 hardware_checker.py --quick
```

### Run Experiments
```bash
# Sensor calibration
python3 formica_experiments/exp1_sensor_calibration.py

# Power profiling
python3 formica_experiments/exp2_power_profiling.py

# SLAM mapping
python3 formica_experiments/exp3_slam_mapping.py

# Maze navigation
python3 formica_experiments/exp4_maze_navigation.py
```

### Run Simulations
```bash
cd data/simulation_data
python3 simulation_A_fast.py
python3 simulation_B_scalability_fixed.py
python3 simulation_C_fast.py
```

## Data Provenance

All experimental data is traced in `data/chapter6_deliverables/meta/PROVENANCE.md`.

## Citation

If you use this data in your research, please cite:

```bibtex
@software{formica_chapter6,
  title = {FormicaBot Chapter 6 Experimental Validation},
  author = {Chandan Sheikder},
  year = {2026},
  url = {https://github.com/chandan-bio-inspired/bio-inspired-thesis-chapter6}
}
```

## License

MIT License - See LICENSE file for details.

## Author

**Chandan Sheikder**  
Bio-Inspired Robotics Research  
2026
