# F1Tenth Vehicle Operations

This runbook consolidates the current team commands for connecting to the Jetson, launching
the ROS 2 stack, validating LiDAR-IMU localization, saving a map, and running pure pursuit.

> Keep the vehicle raised or in a clear test area while bringing up motor control. Be ready to
> stop the control node. Confirm that steering and speed limits match the vehicle configuration.

## 1. Connect to the Jetson

Connect the laptop and Jetson to the same network. On the Jetson, find its address:

```bash
hostname -I
```

From the laptop, connect with the correct Jetson account and IP address:

```bash
ssh <jetson-user>@<jetson-ip>
```

Use the same ROS domain on the Jetson and any ROS 2 laptop/WSL session:

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

## 2. Build and source the workspace

```bash
cd ~/newf1_ws
colcon build
source install/setup.bash
```

Run the `source` command in every new terminal.

## 3. Launch the vehicle stack

### Terminal 1: VESC driver and IMU

```bash
cd ~/newf1_ws
source install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

### Terminal 2: Ackermann-to-VESC bridge

```bash
cd ~/newf1_ws
source install/setup.bash
ros2 run car_control drive_to_vesc_bridge --ros-args -r /drive:=/ackermann_cmd
```

### Terminal 3: LiDAR and sensor TF

```bash
cd ~/newf1_ws
source install/setup.bash
ros2 launch my_robot_bringup robot.launch.py
```

`Streaming data` in the driver output indicates that the LiDAR is running. Do not start a
second `urg_node` manually; only one driver process should access the Hokuyo sensor.

### Terminal 4: Cartographer

```bash
cd ~/newf1_ws
source install/setup.bash
ros2 launch my_cartographer_config cartographer.launch.py
```

Confirm that localization is active:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

### Terminal 5: Teleoperation

```bash
cd ~/newf1_ws
source install/setup.bash
python3 src/joy_to_vesc.py
```

## 4. Validate the sensor and SLAM pipeline

```bash
ros2 run tf2_ros tf2_echo base_link imu
ros2 topic hz /scan
ros2 topic hz /sensors/imu/raw
ros2 topic info /scan --verbose
ros2 node info /urg_node
ps aux | grep -E "urg_node|urg_node_driver"
ros2 node info /cartographer_node
ros2 run tf2_ros tf2_echo map base_link
ros2 topic list | grep map
ros2 topic hz /map
```

Open RViz 2:

```bash
rviz2
```

Set:

- Fixed Frame: `map`
- Map topic: `/map`
- LaserScan topic: `/scan`
- TF display: enabled

## 5. Save the map

After driving several laps and obtaining a stable map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p save_map_timeout:=20.0
```

## 6. Generate and inspect the centerline

```bash
ros2 run map_to_centerline generate_centerline
```

Check that the waypoint file contains data:

```bash
head ~/newf1_ws/src/car_control/car_control/waypoints/waypoints.csv
tail ~/newf1_ws/src/car_control/car_control/waypoints/waypoints.csv
```

## 7. Run pure pursuit

Stop the teleoperation process before starting autonomous path following:

```bash
cd ~/newf1_ws
source install/setup.bash
ros2 run car_control pure_pursuit_tf
```

## Troubleshooting quick reference

| Symptom | Check | Likely action |
| --- | --- | --- |
| `/scan` is far below 10 Hz | Publisher count and `urg_node_driver` processes | Stop the duplicate LiDAR driver |
| `base_link` or `imu` frame is missing | `tf2_echo base_link imu` | Rebuild and launch `my_robot_bringup` |
| Cartographer has no IMU subscription | `ros2 node info /cartographer_node` | Confirm the IMU remap and `use_imu_data = true` |
| No `/map` output | Cartographer logs and sensor rates | Restore `/scan`, IMU, and required TF inputs |
| Vehicle does not respond | VESC driver, bridge, and topic remap | Confirm both control nodes are active |
