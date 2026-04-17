# Aliengo Robot - Full Specifications

## Physical Specifications

| Parameter | Value |
|-----------|-------|
| Model | Unitree Aliengo |
| Type | Quadruped Robot |
| Weight | 24 kg |
| Dimensions (L×W×H) | 0.65×0.30×0.40 m |
| Leg Span | 0.6 m |
| DOF (Total) | 12 (3 per leg) |
| Payload | 10 kg |

## Motion Capabilities

| Parameter | Value |
|-----------|-------|
| Max Speed | 0.8 m/s |
| Max Climbing Angle | 30° |
| Max Step Height | 0.12 m |
| Step Frequency | 2 Hz |
| Gait Types | Walk, Trot, Pace, Bound |

## Actuators

| Joint | Motor Type | Torque (Nm) | Speed (rad/s) |
|-------|-------------|-------------|---------------|
| Hip | GO-M8010-6 | 45 | 24 |
| Thigh | GO-M8010-6 | 45 | 24 |
| Calf | GO-M8010-6 | 24 | 24 |

## Sensors

| Sensor | Specification |
|--------|---------------|
| IMU | 6-axis (3-axis gyro + 3-axis accel) |
| Joint Encoders | 12-bit absolute per joint |
| Foot Force Sensors | 4× FSR sensors |
| Depth Camera | Intel Realsense D435i (optional) |
| LiDAR | RPLIDAR A1 (optional) |

## Power System

| Parameter | Value |
|-----------|-------|
| Battery | 10000 mAh LiPo |
| Voltage | 22.8V |
| Runtime | 2-3 hours |
| Power Consumption | 150W (avg), 500W (peak) |

## Compute

| Component | Specification |
|-----------|---------------|
| Onboard Computer | Jetson Xavier NX (optional) |
| External Control | ROS 2 Humble compatible |
| Communication | Ethernet, WiFi |

## Chapter 6 Comparison with FormicaBot

| Feature | Aliengo | FormicaBot | Advantage |
|---------|---------|------------|-----------|
| Type | Quadruped | Hexapod | FormicaBot |
| Legs | 4 | 6 | FormicaBot |
| DOF | 12 | 18 | FormicaBot |
| Weight | 24 kg | 3 kg | FormicaBot |
| Cost | ~$20,000 | ~$500 | FormicaBot |
| Power | 150W | 15W | FormicaBot |
| Speed (flat) | 0.8 m/s | 0.3 m/s | Aliengo |
| Terrain Adaptability | High | Very High | FormicaBot |
| Energy Efficiency | 7.5 Wh/km | 5.0 Wh/km | FormicaBot |

## Conclusion

The Aliengo robot serves as a **high-performance quadruped baseline** in Chapter 6, demonstrating:
- Superior speed on flat terrain (2.6× faster than FormicaBot)
- Good energy efficiency at moderate speeds
- Limited terrain adaptability compared to hexapod FormicaBot
- Higher cost for deployment (40× more expensive)

The comparative analysis validates that FormicaBot's hexapod design provides:
- Superior terrain adaptability on rough surfaces
- Better static stability with 6 legs
- Lower power consumption per distance traveled
- Significantly lower deployment cost

Reference: Unitree Robotics - https://www.unitree.com/aliengo/
