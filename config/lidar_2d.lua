include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",    
  published_frame = "odom", -- 发布的坐标系      
  odom_frame = "odom",
  provide_odom_frame = false,      -- 是否提供里程计坐标系
  publish_frame_projected_to_2d = true,   -- 发布的位姿投影到2D平面
  use_pose_extrapolator = true,
  use_odometry = true,  -- 是否使用里程计          
  use_nav_sat = false,  -- 是否使用GPS
  use_landmarks = false,
  num_laser_scans = 1,-- 激光雷达数量
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0, 
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,-- 位姿发布频率
  trajectory_publish_period_sec = 30e-3,-- 优化轨迹发布频率
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4     

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0


--此处为地图更新的关键参数调整
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03  -- 平移 3 厘米就更新
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5) -- 旋转 0.5 度就更新
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3      -- 最多 0.3 秒就更新

-- 子图相关参数调整
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.75
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5 -- 每5帧雷达数据构建一个子图
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 20.0 -- 平移代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0 -- 旋转代价权重

TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.0 -- IMU 重力时间常数


POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- Huber 核函数尺度
POSE_GRAPH.optimize_every_n_nodes = 60 -- 每60个节点进行一次优化
POSE_GRAPH.constraint_builder.min_score = 0.65 -- 约束建立的最低分数


return options