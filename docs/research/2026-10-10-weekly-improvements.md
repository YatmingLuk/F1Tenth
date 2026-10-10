# F1TENTH 周报：传感器集成、低速 odometry 与速度估计计划

Author: Yiming Lu

整理日期：2026-10-10；本周范围：2026-10-05 至 2026-10-10。

## 1. 仓库审查与证据边界

本次审查的默认分支为 `main`，基准提交为
[`6d3db57`](https://github.com/YatmingLuk/F1-Tenth/commit/6d3db57b54075602a029b3d4310e226cf8a727f6)。
审查时只有 `main` 分支，没有现有 PR。先通过 GitHub 连接读取固定提交的快照，再下载完整仓库并核对 HEAD 与基准一致。
在独立分支应用已检查的补丁，只更新列出的文件，保留其余文件、图片及 PDF。

| 内容 | 证据与状态 | 本次处理 |
| --- | --- | --- |
| VESC IMU SI 单位转换 | 10 月 4 日 `d6d9579` 已合入 | 引用既有代码和历史 diff，避免重复转换 |
| Cartographer IMU/TF 配置 | 同一提交已合入 | 保留配置，说明实际 TF 链及归属 |
| Hokuyo 重复节点与扫描频率 | 既有集成文档记录了排查和结果 | 保留为历史记录，补充复现与验收步骤 |
| VESC 低速 deadband | 基准代码硬编码 `0.05 m/s` | 新增可配置参数；默认仍为 `0.05` |
| VESC USB 端口 | YAML 固定 `/dev/ttyACM1` | 新增启动覆盖参数及识别流程 |
| Cartographer 速度估计 | 可读取的旧对话提出方案，未提供节点或实测结果 | 明确列为下一阶段计划 |

本周改进已由作者在实车 Linux 系统上完成验证，并据此写入 Research Journal。
本仓库保留可复现的配置、检查命令和代码 diff；当前提交环境未重复连接车辆，
因此不另行改写或推断 Journal 未记录的数值。Cartographer 速度估计仍是下一阶段计划。

## 2. 当前架构与 TF 所有权

```mermaid
flowchart LR
    V[VESC USB] --> D[vesc_driver]
    D -->|/sensors/imu/raw: SI units| C[Cartographer]
    D -->|/sensors/core: ERPM| O[vesc_to_odom: comparison only]
    H[Hokuyo] --> U[one urg_node_driver]
    U -->|/scan| C
    S[base_link to imu / laser: static TF] --> C
    C --> T[map to odom to base_link]
    C --> M[/map]
    T -.-> E[planned velocity estimator]
    O -.-> E
```

当前 `hokuyo_2d.lua` 的关键配置：

```lua
tracking_frame = "imu",
published_frame = "base_link",
odom_frame = "odom",
provide_odom_frame = true,
use_odometry = false,
-- 在 options 表之外：
TRAJECTORY_BUILDER_2D.use_imu_data = true
```

实际动态链为 `map -> odom -> base_link`，两个动态链接当前均由 Cartographer 提供。
`tf2_echo map base_link` 查看的是组合变换；不能据此认为存在直接发布的 TF 边。
静态链接由 `robot.launch.py` 提供：`base_link -> laser` 为 `[0.11, 0, 0.12] m`，
`base_link -> imu` 为 `[-0.10, 0, 0.12] m`，旋转目前均为 identity，安装姿态需实车核对。
`tracking_frame` 放在 IMU 原点，避免把非零 IMU 平移当作共点测量。

VESC odometry 目前没有被 Cartographer 融合。对照实验使用 `/vesc/odom`，并设置
`publish_tf = false`，避免与 Cartographer 争用 `odom -> base_link`。
注意：类中的默认值为 false，但仓库的 `vesc_to_odom_node.launch.xml` 显式设为 true，
且其 ERPM gain 为占位值 `1.0`；不要把该 XML 的默认启动当作已校准的实车配置。

## 3. IMU：改了哪里、原因及历史 diff

位置：[`src/vesc/vesc_driver/src/vesc_driver.cpp`](../../src/vesc/vesc_driver/src/vesc_driver.cpp)，
`VescDriver::vescPacketCallback()` 的 `ImuData` 分支。
VESC packet 解码器提供 g 和 deg/s；ROS 标准 IMU 消息需要 m/s² 和 rad/s。
转换只在标准消息边界做一次，自定义 `/sensors/imu` 保留 VESC 原始单位。

历史变更摘要（已经合入；不是本 PR 新增）：

```diff
- std_imu_msg.linear_acceleration.x = imuData->acc_x();
+ constexpr double kStandardGravity = 9.80665;
+ constexpr double kDegreesToRadians = 0.017453292519943295;
+ std_imu_msg.linear_acceleration.x = imuData->acc_x() * kStandardGravity;
- std_imu_msg.angular_velocity.x = imuData->gyr_x();
+ std_imu_msg.angular_velocity.x = imuData->gyr_x() * kDegreesToRadians;
```

y/z 分量使用同样转换；两个消息共用一次 `now()` 和 `frame_id = "imu"`。
时间戳是主机收到样本的时间，并非已验证的硬件采样时钟。
历史记录：静止重力轴约 `10.00–10.02 m/s²`，手动旋转峰值约 `2.54 rad/s`。
下一次检查应同时看三轴加速度模长、旋转符号与频率，不能只看单轴值。
不要在 `imu_relay.py` 再转换；它只改 frame 名称，不会旋转数值。
`vesc_full_bridge.py` 是另一个独立串口客户端，不属于当前 Cartographer 启动链，
不要与 C++ driver 同时访问同一个 VESC USB 端口。

## 4. Cartographer 配置与 Hokuyo 排查

历史修改位置及原因：

| 文件 / 位置 | 变更 | 原因 |
| --- | --- | --- |
| `src/my_cartographer_config/config/hokuyo_2d.lua` / `options` | `tracking_frame`: `base_link` → `imu` | IMU 跟踪原点一致 |
| 同上 / `TRAJECTORY_BUILDER_2D` | `use_imu_data`: false → true | 启用 IMU 输入 |
| `src/my_cartographer_config/launch/cartographer.launch.py` / remappings | `/imu/data_fixed` → `/sensors/imu/raw` | 直接订阅已转换的标准消息 |
| `src/my_robot_bringup/launch/robot.launch.py` / 返回列表 | 加入 `static_tf_base_to_imu` | 定义 TF 后必须实际启动 |

历史 diff 摘要：

```diff
- tracking_frame = "base_link",  -- imu / base_link
+ tracking_frame = "imu",
- TRAJECTORY_BUILDER_2D.use_imu_data = false -- IMU 開關
+ TRAJECTORY_BUILDER_2D.use_imu_data = true
- ('imu', '/imu/data_fixed')
+ ('imu', '/sensors/imu/raw'),
- #       static_tf_base_to_imu
+         static_tf_base_to_imu,
```

完整历史源代码 diff 见 [2026-10-04-integration-code-diff.patch](2026-10-04-integration-code-diff.patch)
及 [原始提交](https://github.com/YatmingLuk/F1-Tenth/commit/d6d957915434ac9c98d30c643afe6ebab978469e)。
本次没有重复改 Lua 或 IMU remap；只删掉 bringup 中从未启动的 VESC Node 定义，
避免它的硬编码端口与正式 driver 配置混淆。实际启动节点仍为一个 Hokuyo 和两个静态 TF。

Hokuyo 历史问题是两个 `urg_node_driver` 同时访问同一传感器，记录到 `0.06–0.17 Hz`
及 `24.432 s` 间隔。停止手动启动的重复进程后，记录恢复为 `10.002–10.005 Hz`。
这不是本次修改雷达硬件扫描频率的结果，也不能以 `ros2 topic hz` 代替设备规格。

```bash
ros2 topic info /scan --verbose
ros2 node info /urg_node
pgrep -af 'urg_node|urg_node_driver'
ros2 topic hz /scan
```

确认单个传感器对应一个 driver 进程和一个 `/scan` publisher；通过其启动终端 Ctrl+C
停止识别出的重复实例，再观察至少 60 s。不要用模糊的全局进程终止命令。
当前 `num_accumulated_range_data = 2`，10 Hz 扫描积累两帧后处理；
`pose_publish_period_sec = 0.005` 是发布周期配置，不表示 200 Hz 独立定位测量，
也不表示 Hokuyo 的扫描频率。

## 5. VESC 速度和 odometry 的低速 deadband

位置：[`vesc_to_odom.cpp`](../../src/vesc/vesc_ackermann/src/vesc_to_odom.cpp)
的构造函数和 `vescStateCallback()`，以及
[`vesc_to_odom.hpp`](../../src/vesc/vesc_ackermann/include/vesc_ackermann/vesc_to_odom.hpp)。
现有速度公式保留：

```cpp
double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
```

`state.speed` 是 ERPM，gain 的单位为 ERPM/(m/s)，offset 为 ERPM。
负号是仓库原有约定，须以实车正向行驶核对；不能把指令速度当作车体真值，
不能直接把 command bridge 的 gain 当成已校准 odometry gain。

原来的 `abs(v) < 0.05` 会同时让 twist 和积分位移为零，因此真实缓慢运动可能消失。
本次保持默认值并参数化，支持对照实验，不在缺少测量依据时降低阈值：

```diff
+ speed_deadband_ = declare_parameter<double>("speed_deadband", 0.05);
+ if (!std::isfinite(speed_deadband_) || speed_deadband_ < 0.0) {
+   throw std::invalid_argument("speed_deadband must be finite and non-negative (m/s)");
+ }
- if (std::fabs(current_speed) < 0.05) {
+ if (std::fabs(current_speed) < speed_deadband_) {
    current_speed = 0.0;
  }
```

参数在启动时读取；没有实现运行中动态更新。阈值为 0 可关闭抑制，负数或非有限数拒绝启动。
比较是严格小于：默认下 ±0.049 m/s 被抑制，±0.050 m/s 保留。
driver 的 `/sensors/core` 原始 ERPM 和电机指令路径都不经过这个 deadband。
它只能调整估计器输出，不能修复 VESC 固件自身的低速测量极限、轮滑或齿比标定。
默认 `use_servo_cmd_to_calc_angular_velocity = true` 时，没有舵机指令会提前返回，不发布 odometry。

独立速度对照可暂时关闭舵机依赖；此时角速度和 yaw 积分不能用作有效转向估计。
先把实测 gain/offset 写到车上 `vesc_odom_calibrated.yaml`：

```bash
ros2 run vesc_ackermann vesc_to_odom_node --ros-args \
  --params-file vesc_odom_calibrated.yaml \
  -p speed_deadband:=0.05 -p publish_tf:=false \
  -p use_servo_cmd_to_calc_angular_velocity:=false \
  -r odom:=/vesc/odom
```

停止该节点后，使用同一校准和测试路线，把 `speed_deadband:=0.05` 改为 `speed_deadband:=0.0`
做第二轮。记录原始 ERPM、转换前速度、输出 twist、积分位移及参考距离/时间，
覆盖静止、缓慢前进/倒车和正常速度；比较抑制静止噪声与丢失低速位移的取舍。

## 6. VESC USB 识别与端口覆盖

修改位置：driver 的 `VescDriver()`、`vesc_driver_node.launch.py` 和 `vesc_config.yaml` 注释。
`ttyACM0/1` 可能随重插和设备枚举顺序改变；本次未连接车辆，无法认定哪个端口是 VESC。
不要随机试端口或给所有串口下发电机命令。先只读检查：

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/serial/by-path/
readlink -f /dev/sensors/hokuyo
udevadm info --query=property --name=/dev/ttyACM1
udevadm info --attribute-walk --name=/dev/ttyACM1
```

把设备 vendor/product、serial 和物理插口与 VESC 对照；必要时停用 driver 后重插 VESC，
比较新增/消失的设备。优先选择经核实的 `/dev/serial/by-id/...`；无 serial 时，
`by-path` 路径依赖物理 USB 插口。对别名执行 `readlink -f` 确认指向目标设备。
仓库的 `99-vesc6.rules` 使用 vendor `0483`、product `5740` 和 `/bin/vesc_device_lookup`，
这些不是本车本次确认值；在 helper 未安装或身份未核对时不要直接套用规则。

新启动方式（将变量填为已经确认的实际路径）：

```bash
VESC_PORT='/dev/serial/by-id/<confirmed-vesc-device>'
ros2 launch vesc_driver vesc_driver_node.launch.py port:="$VESC_PORT"
```

`port` 启动参数默认为空；空值保留原 YAML 的 `port`，非空值通过 driver 的
`port_override` 优先使用。原有 `config:=...` 参数继续有效。YAML 默认仍为 `/dev/ttyACM1`，
本次不虚构 serial 或安装 udev 规则。单独直接运行 driver 时也可用既有 `-p port:=...`。
bringup 中未使用的另一份 VESC 配置被删掉，正式启动入口只有上述 driver launch。

## 7. 下一阶段：Cartographer-only 速度估计

状态：设计计划，尚未添加运行节点、测试结果或 EKF。旧对话提出由位置变化计算速度、
速度方向和 heading；本次结合实际配置细化如下。

1. 先确认 `odom -> base_link` 连续、TF 时间戳递增、没有 competing broadcaster。
   使用 Cartographer 的本地连续坐标求导；`map -> base_link` 留作全局位置/heading 对照。
2. 按 TF 样本时间戳计算 `dt`，重复样本不重新计算，`dt <= 0`、过期数据、长间隔和
   重启后样本对丢弃并重置。不能用查询定时器的次数代替新定位样本数。
3. 计算 `vx = (x2-x1)/dt`、`vy = (y2-y1)/dt`、`speed = hypot(vx,vy)`，
   移动方向 `atan2(vy,vx)`；heading 从四元数读取，角速度使用绕 ±π 展开的 yaw 差。
   接近静止时移动方向无定义，应输出有效标志。
4. 先输出原始差分并记录噪声，再比较窗口回归或低通滤波；窗口大小由误差和延迟评估决定。
   同时记录 `map -> odom` 修正，验证回环时速度没有被全局坐标跳变污染。
5. 规划独立 `/cartographer/velocity` 输出并声明 frame 为 `odom`；其 vx/vy 是 odom 坐标分量。
   与 VESC 的 base_link.x 比较时，先用 yaw 把速度转到车体系：
   `v_forward = cos(yaw)*vx + sin(yaw)*vy`。speed 是非负模长，不能直接与有符号的倒车速度相比。
6. 录制静止、定速直线、低速、倒车、转弯、停车和回环场景；同时保存 `/scan`、
   `/sensors/imu/raw`、`/sensors/core`、`/vesc/odom`、`/tf`、`/tf_static`。
   输出静止速度噪声、参考距离/时间的速度偏差、响应延迟和无效样本比例。
   先验证 Cartographer-only，再决定是否引入 VESC 融合及 EKF。

## 8. 实车验证记录与复现检查

作者已在实车 Linux 系统上验证本周记录的传感器、TF、扫描频率、低速 deadband 和 USB 端口改进，
Research Journal 是该次验证的项目记录。本次仓库整理还通过了完整仓库 Python 语法检查、
用替身 API 执行的启动描述结构检查，以及完整补丁与源代码 patch 的 `git apply --check`。
结构检查确认 bringup 为一个 Hokuyo 和两个静态 TF、USB 覆盖为字符串、空端口默认和原有 config 参数保留。
以下命令用于在 Jetson/Linux 复现和回归检查：

```bash
cd ~/newf1_ws
colcon build --packages-select vesc_driver vesc_ackermann my_robot_bringup my_cartographer_config
source install/setup.bash
colcon test --packages-select vesc_driver vesc_ackermann my_robot_bringup my_cartographer_config
colcon test-result --verbose
ros2 topic hz /sensors/imu/raw
ros2 topic hz /scan
ros2 node info /cartographer_node
ros2 run tf2_ros tf2_echo base_link imu
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /map
```

| 检查 | 验收 / 证据 |
| --- | --- |
| 端口兼容 | 无 port 参数继续用 YAML；显式 port 使用已识别设备；自定义 config 保持有效 |
| deadband | 默认抑制 abs(v)<0.05；0 保留低速；负数/NaN/Inf 被拒绝；正反向都测 |
| IMU | 消息单位/坐标正确；模长约重力；轴向、符号和 TF 一致 |
| Hokuyo | 单一 publisher/进程；连续观察频率及最大间隔 |
| TF | 本地/全局链接存在且只有一个动态发布者；VESC 不发布 TF |
| 低速 odometry | 实车验证已完成；后续回归保留 deadband 默认、关闭、边界及正反向检查 |
| 速度估计 | Cartographer-derived 速度估计属于下一阶段，需另行采集 rosbag 和参考距离/时间 |

实车基线记录：IMU `49.993–50.001 Hz`、scan `10.002–10.005 Hz`、map `1.000 Hz`。
后续修改应以这些记录和上述复现命令做回归对比。

## 9. 本次具体 diff 与参考资料

- [本次源代码/配置完整 diff](2026-10-10-code-diff.patch)：相对 `6d3db57`，包括参数声明、验证、回调、端口覆盖和 bringup 清理。
- [既有集成与测量记录](../IMU_LIDAR_CARTOGRAPHER.md)。
- [运行操作与本周排查入口](../OPERATIONS.md)。
- [ROS 2 Humble Imu 消息定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)：标准消息单位约定。
- [Cartographer ROS 配置参考](https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html)：frame 和 odometry 配置含义。
- [REP 105 原文](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)：连续 odom、可跳变 map 及 TF 归属。
