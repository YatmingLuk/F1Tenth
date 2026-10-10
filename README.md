# F1Tenth Autonomous Racing Platform

This repository contains our ROS 2 F1Tenth vehicle stack for localization, mapping,
trajectory generation, vehicle control, and autonomous-racing experiments. The current
milestone integrates the VESC IMU and Hokuyo LiDAR into Cartographer and publishes a
real-time vehicle pose and occupancy grid for RViz 2.

## Current milestone: LiDAR-IMU localization

- VESC IMU published as `sensor_msgs/msg/Imu` on `/sensors/imu/raw` at approximately 50 Hz.
- Hokuyo LiDAR published as `sensor_msgs/msg/LaserScan` on `/scan` at approximately 10 Hz.
- Static transforms connect `base_link` to the `imu` and `laser` sensor frames.
- Cartographer consumes both sensor streams and publishes `map -> odom -> base_link` TF (the composed transform gives vehicle pose).
- The occupancy grid is published on `/map` at 1 Hz for real-time RViz 2 visualization.
- VESC acceleration and angular velocity are converted to ROS-standard units: m/s^2 and rad/s.

The implementation details and validation results are documented in
[LiDAR-IMU Cartographer integration](docs/IMU_LIDAR_CARTOGRAPHER.md).

## Expected system architecture

![Expected ROS 2 architecture for the F1Tenth vehicle](docs/assets/f1tenth-architecture.png)

The diagram shows the intended end-to-end architecture, from LiDAR-based mapping and
waypoint generation to pure pursuit, overtaking decisions, VESC control, and the physical car.
The current tested milestone covers the sensor and localization path; motion-based validation
and higher-level autonomous behavior remain active work.

### Localization data flow

```mermaid
flowchart LR
    VESC[VESC IMU] -->|/sensors/imu/raw| CARTO[Cartographer]
    LIDAR[Hokuyo LiDAR] -->|/scan| CARTO
    TF[Static TF: base_link to imu/laser] --> CARTO
    CARTO -->|/map| RVIZ[RViz 2]
    CARTO -->|map to base_link TF| RVIZ
```

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/vesc` | VESC driver, messages, and Ackermann bridge |
| `src/my_robot_bringup` | Hokuyo launch and static sensor transforms |
| `src/my_cartographer_config` | Cartographer launch and 2D SLAM configuration |
| `src/car_control` | Drive bridge, waypoint logging, and pure pursuit |
| `src/map_to_centerline` | Centerline extraction from a saved map |
| `docs/OPERATIONS.md` | Build, launch, validation, mapping, and driving commands |
| `docs/IMU_LIDAR_CARTOGRAPHER.md` | Sensor-fusion changes, rationale, results, and limits |
| `docs/research/2026-10-10-weekly-improvements.md` | Weekly architecture, change locations, code excerpts, and velocity plan |
| `docs/research/2026-10-10-code-diff.patch` | Exact new code/config changes against the reviewed base |

## Requirements

- Ubuntu with ROS 2 Humble
- Cartographer ROS
- Hokuyo `urg_node`
- `tf2_ros`, `sensor_msgs`, `nav2_map_server`, and RViz 2
- A configured VESC and Hokuyo LiDAR connected to the F1Tenth vehicle

## Build

```bash
cd ~/newf1_ws
colcon build
source install/setup.bash
```

## Quick start

Use separate terminals for the main processes:

```bash
# Terminal 1: VESC and IMU
ros2 launch vesc_driver vesc_driver_node.launch.py

# Terminal 2: drive-command bridge
ros2 run car_control drive_to_vesc_bridge --ros-args -r /drive:=/ackermann_cmd

# Terminal 3: LiDAR and static transforms
ros2 launch my_robot_bringup robot.launch.py

# Terminal 4: LiDAR-IMU Cartographer
ros2 launch my_cartographer_config cartographer.launch.py
```

Before each command, run `source ~/newf1_ws/install/setup.bash`. Do not start a second
`urg_node` manually while `robot.launch.py` is running; duplicate drivers caused the LiDAR
rate to fall from approximately 10 Hz to an intermittent 0.06-0.17 Hz during testing.

See [Vehicle operations](docs/OPERATIONS.md) for SSH setup, ROS domain configuration,
RViz checks, map saving, centerline generation, teleoperation, and pure pursuit.

## Validation snapshot

| Signal | Observed rate | Status |
| --- | ---: | --- |
| VESC IMU | 49.993-50.001 Hz | Stable |
| Hokuyo LiDAR | 10.002-10.005 Hz | Stable after duplicate driver removal |
| Occupancy grid `/map` | 1.000 Hz | Stable |

Cartographer was observed subscribing to both `/scan` and `/sensors/imu/raw`, while the
`map -> base_link` transform updated continuously. The documented sensor, TF, frequency,
deadband, and USB-port changes were validated on the vehicle under Linux. Quantitative
evaluation of the planned Cartographer-derived velocity estimator remains next-stage work.

## Research record

[Weekly improvements (October 5-10, 2026)](docs/research/2026-10-10-weekly-improvements.md)
consolidates existing sensor integration with configurable low-speed odometry, USB identification,
and the next Cartographer-only velocity experiment. Historical measurements are distinguished
from changes that still need Jetson validation.


The detailed October 1, 2026 experiment log is available as a
[Personal Research Journal PDF](docs/research/2026-10-01-personal-research-journal.pdf).

## Contributors

Developed by the F1Tenth project team. LiDAR-IMU integration, Cartographer configuration,
validation, and repository documentation were prepared by Yiming Lu.
