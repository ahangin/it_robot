
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",      --imu_link
  published_frame = "base_footprint",  --base_footprint
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.05,--0.5
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
MAP_BUILDER.num_background_threads = 8.
---------------------------------------------------------------------------------
TRAJECTORY_BUILDER_2D.min_range = 0.4            --0.4
TRAJECTORY_BUILDER_2D.max_range = 30.  --20.0
TRAJECTORY_BUILDER_2D.min_z = 0.1           --0.1
TRAJECTORY_BUILDER_2D.max_z = 1.5
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.02

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 100. --1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1. --10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 150. ---40.

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(20) -- 1.                
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2--0.2  0.5->0.2 约束线变多
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5. -- 5. 
                          --  体素滤波 -- 
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025 -- 0.025
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2 -- 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200 -- 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50. -- 50.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9 -- 0.9.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100 -- 100.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50. -- 50.

TRAJECTORY_BUILDER_2D.use_imu_data = true

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)-- 20
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1 -- 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1 -- 1e-1

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90. --90.
--------------------------------------------------------------------------------
POSE_GRAPH.optimize_every_n_nodes = 90. --90  
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 -- 0.3
POSE_GRAPH.global_sampling_ratio = 0.003 -- 0.003

POSE_GRAPH.constraint_builder.max_constraint_distance = 15. -- 15.
POSE_GRAPH.constraint_builder.min_score = 0.55 -- 0.55    
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55 -- 0.6

-- POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0
-- POSE_GRAPH.global_sampling_ratio = 0

return options
