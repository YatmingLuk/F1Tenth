include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",  -- imu / base_link
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 累積 2 幀，增加點的密度，有助於平面的生成
-- 強制提高雷達在優化中的權重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.  -- (原本 1.) 強制算法一定要對齊地圖
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100. -- (原本 10) 降低對馬達偏移的信任
TRAJECTORY_BUILDER_2D.use_imu_data = false -- IMU 開關
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 利用laser scan matching

-- 先把 POSE_GRAPH.optimization_problem 全部註解掉，讓它用預設值
-- POSE_GRAPH.optimization_problem.imu_acceleration_weight = 1.0
-- POSE_GRAPH.optimization_problem.imu_rotation_weight = 1.0

return options
