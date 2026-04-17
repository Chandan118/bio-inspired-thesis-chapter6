# Aliengo Robot Model - Chapter 6 Reference Implementation

## Overview

This directory contains the Aliengo quadruped robot model specifications used in the bio-inspired navigation thesis Chapter 6 comparative analysis.

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

- `aliengo_urdf.urdf` - Robot URDF model
- `gait_params.yaml` - Gait parameters
- `experiment_data.csv` - Comparative experiment logs
- `model_specs.md` - Full specifications

## Usage

The Aliengo model is used as a **quadruped comparison baseline** for the FormicaBot (hexapod) platform validation in Chapter 6.

## Reference

Unitree Aliengo Robot: https://www.unitree.com/aliengo/
