# Aliengo Robot Model - Chapter 6 Reference Implementation

## Overview

This directory contains the Aliengo quadruped robot model specifications used in the bio-inspired navigation thesis Chapter 6 comparative analysis with FormicaBot.

## Robot Specifications (Aliengo Unitree)

| Parameter | Value |
|-----------|-------|
| Weight | 24 kg |
| DOF | 12 (3 per leg) |
| Leg Span | 0.6 m |
| Max Speed | 0.8 m/s |
| Battery Life | 2-3 hours |
| Sensors | IMU, encoders, force sensors |
| Compute | External (Jetson Xavier NX compatible) |

## Files

| File | Description |
|------|-------------|
| `aliengo_urdf.urdf` | Robot URDF model (12 DOF) |
| `gait_params.yaml` | Gait parameters for walk/trot/pace/bound |
| `experiment_data.csv` | Comparative experiment logs |
| `model_specs.md` | Full specifications and comparison |

## Chapter 6 Comparison with FormicaBot

| Feature | Aliengo | FormicaBot |
|---------|---------|------------|
| Type | Quadruped | Hexapod |
| Legs | 4 | 6 |
| DOF | 12 | 18 |
| Weight | 24 kg | 3 kg |
| Cost | ~$20,000 | ~$500 |
| Terrain Adaptability | High | Very High |
| Power Consumption | 150W | 15W |

## Usage

The Aliengo model serves as the **quadruped comparison baseline** for validating FormicaBot's superior terrain adaptability and multi-legged navigation advantages.

## Reference

Unitree Aliengo Robot: https://www.unitree.com/aliengo/
