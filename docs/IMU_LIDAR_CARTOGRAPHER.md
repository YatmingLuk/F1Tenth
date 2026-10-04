# LiDAR-IMU Integration with Cartographer

This document records the changes used to connect the VESC IMU and Hokuyo LiDAR to
Cartographer on ROS 2 Humble. It also distinguishes verified behavior from work that still
requires motion-based testing.

## Sensor contract

| Sensor | ROS topic | Message type | Frame | Observed rate |
| --- | --- | --- | --- | ---: |
| Hokuyo LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | `laser` | about 10 Hz |
| VESC IMU | `/sensors/imu/raw` | `sensor_msgs/msg/Imu` | `imu` | about 50 Hz |

Cartographer tracks the `imu` frame and publishes the vehicle pose as `base_link`. Static
transforms connect both sensor frames to `base_link`.

## 1. Publish a ROS-standard IMU message

File: `src/vesc/vesc_driver/src/vesc_driver.cpp`

The driver publishes a `sensor_msgs/msg/Imu` message on `/sensors/imu/raw`. The custom and
standard messages share one timestamp and use `frame_id = "imu"` so the sample and TF tree
can be aligned consistently.

The VESC reports acceleration in g and angular velocity in degrees per second. ROS expects
metres per second squared and radians per second, so the standard message converts both:

```cpp
constexpr double kStandardGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

std_imu_msg.linear_acceleration.x = imuData->acc_x() * kStandardGravity;
std_imu_msg.linear_acceleration.y = imuData->acc_y() * kStandardGravity;
std_imu_msg.linear_acceleration.z = imuData->acc_z() * kStandardGravity;

std_imu_msg.angular_velocity.x = imuData->gyr_x() * kDegreesToRadians;
std_imu_msg.angular_velocity.y = imuData->gyr_y() * kDegreesToRadians;
std_imu_msg.angular_velocity.z = imuData->gyr_z() * kDegreesToRadians;
```

Stationary acceleration after conversion was approximately 10.00-10.02 m/s^2 on the gravity
axis. A manual rotation test produced a peak of approximately 2.54 rad/s after conversion.

## 2. Activate the sensor transforms

File: `src/my_robot_bringup/launch/robot.launch.py`

The `base_link -> imu` transform had been defined but omitted from the returned
`LaunchDescription`. The launch file now starts both static transforms:

```text
base_link -> laser  translation: [ 0.11, 0.00, 0.12 ], rotation: identity
base_link -> imu    translation: [-0.10, 0.00, 0.12 ], rotation: identity
```

The IMU rotation is currently an identity transform. It must be checked against the physical
mounting orientation during a controlled yaw test.

Verify the TF tree with:

```bash
ros2 run tf2_ros tf2_echo base_link imu
ros2 run tf2_ros tf2_echo base_link laser
```

## 3. Enable the IMU in Cartographer

File: `src/my_cartographer_config/config/hokuyo_2d.lua`

```lua
tracking_frame = "imu"
published_frame = "base_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
```

File: `src/my_cartographer_config/launch/cartographer.launch.py`

```python
remappings=[
    ('scan', '/scan'),
    ('imu', '/sensors/imu/raw'),
]
```

This configuration gives Cartographer a gravity-aligned tracking frame while publishing the
estimated vehicle pose as `base_link`.

## 4. Diagnose duplicate LiDAR drivers

The first LiDAR rate test showed approximately 0.06-0.17 Hz, including a gap of 24.432 s.
The cause was two `urg_node_driver` processes using the same Hokuyo sensor: one launched
manually and one launched by `robot.launch.py`.

```bash
ros2 topic info /scan --verbose
ros2 node info /urg_node
ps aux | grep -E "urg_node|urg_node_driver"
```

After the manually started process was stopped, `/scan` returned to approximately 10 Hz.
Only one process should own the LiDAR connection.

## 5. Verification

```bash
ros2 topic hz /sensors/imu/raw
ros2 topic hz /scan
ros2 node info /cartographer_node
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /map
```

Observed results:

- IMU: 49.993, 50.001, and 49.999 Hz.
- LiDAR after troubleshooting: 10.002, 10.005, and 10.003 Hz.
- Occupancy grid: 1.000 Hz.
- Cartographer subscribed to both sensor topics.
- The `map -> base_link` transform became available and updated continuously.

For RViz 2, use `map` as the fixed frame and display `/map`, `/scan`, and TF.

## Remaining validation

- Verify that LiDAR points remain aligned with the occupancy grid during motion.
- Confirm the IMU yaw direction and physical mounting transform.
- Measure stationary and moving pose drift with and without IMU input.
- Measure end-to-end latency and tune Cartographer for realistic driving speed.
- Validate a reliable VESC or odometry source for ego-vehicle speed.

The corresponding experiment log is stored in
[`docs/research/2026-10-01-personal-research-journal.pdf`](research/2026-10-01-personal-research-journal.pdf).
