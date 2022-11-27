-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"
 
options = {
  map_builder = MAP_BUILDER,                            -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,              -- trajectory_builder.lua的配置信息
  map_frame = "map",                                    -- 地图坐标系的名字
  tracking_frame = "base_link",                         -- 将所有传感器数据转换到这个坐标系下默认imu_link，在有IMU的情况下将其他数据转到IMU下，因为IMU频率高，转到其他坐标系下需要转换次数更多
  published_frame = "odom",                          -- tf: map -> odom
  odom_frame = "odom",                                  -- 里程计的坐标系名字
  provide_odom_frame = false,                           -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
  publish_frame_projected_to_2d = true,                 -- 是否将坐标系投影到平面上
  use_odometry = true,                                  -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                                  -- 是否使用gps
  use_landmarks = false,                                -- 是否使用landmark
  num_laser_scans = 1,                                  -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,                       -- 是否使用multi_echo_laser_scans数据
                                                        -- 这两个还有下面的是否使用点云数据不能同时为0
  num_subdivisions_per_laser_scan = 1,                  -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                                 -- 是否使用点云数据
  lookup_transform_timeout_sec = 0.2,                   -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,                      -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,                       -- 发布pose的时间间隔，比如：5e-3频率是200Hz
  trajectory_publish_period_sec = 30e-3,                -- 发布轨迹标记的间隔，30e-3是科学记数法，表示的是30乘以10的-3次方。也就是0.030秒，就是30毫秒。
  rangefinder_sampling_ratio = 1.,                      -- 传感器数据的采样频率，多少次数据采样一次，默认都是1。
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
 
MAP_BUILDER.use_trajectory_builder_2d = true
 
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35       -- 其含义应该应该是多少帧插入一次子图，算法中还有一个乘二操作。
--num_range_data设置的值与CPU有这样一种关系，值小(10)，CPU使用率比较稳定，整体偏高，值大时，CPU短暂爆发使用(插入子图的时候)，平时使用率低，呈现极大的波动状态。
TRAJECTORY_BUILDER_2D.min_range = 0.3                   --激光的最近有效距离
TRAJECTORY_BUILDER_2D.max_range = 8.                    --激光最远的有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.      --无效激光数据设置距离为该数值，滤波的时候使用
TRAJECTORY_BUILDER_2D.use_imu_data = false              --是否使用imu数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true     -- 选择是否先求解online scan matching，然后用correlative scan matcher为Ceres求解器产生一个好的初始解，为true时效果更好
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 线距离搜索框，在这个框的大小内，搜索最佳scan匹配.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  --两个参数相当于最小化误差函数中的权重值，两者的比值，决定了更侧重与平移和旋转中的哪部分
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}
 
POSE_GRAPH.optimization_problem.huber_scale = 1e2       --鲁棒核函数，去噪
POSE_GRAPH.optimize_every_n_nodes = 40                  --后端优化节点
POSE_GRAPH.constraint_builder.min_score = 0.65          --当匹配分数低于此值时,忽略该分数。感觉可能是用来检测是否匹配到回环的


-- fast localization

-- MAP_BUILDER.num_background_threads = 8
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio
-- POSE_GRAPH.global_sampling_ratio = 0.1 * POSE_GRAPH.global_sampling_ratio
-- POSE_GRAPH.max_num_final_iterations = 1

 
return options
