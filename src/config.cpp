// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifdef SLAM_USE_ROS
#include <vikit/params_helper.h>
#endif
#include "config.h"

namespace ORB_SLAM {

Config::Config() :
#ifdef SLAM_USE_ROS
    trace_name(vk::getParam<string>("svo/trace_name", "svo")),
    trace_dir(vk::getParam<string>("svo/trace_dir", "/tmp")),
    n_pyr_levels(vk::getParam<int>("svo/n_pyr_levels", 3)),
    use_imu(vk::getParam<bool>("svo/use_imu", false)),
    core_n_kfs(vk::getParam<int>("svo/core_n_kfs", 3)),
    map_scale(vk::getParam<double>("svo/map_scale", 1.0)),
    grid_size(vk::getParam<int>("svo/grid_size", 30)),
    init_min_disparity(vk::getParam<double>("svo/init_min_disparity", 50.0)),
    init_min_tracked(vk::getParam<int>("svo/init_min_tracked", 50)),
    init_min_inliers(vk::getParam<int>("svo/init_min_inliers", 40)),
    klt_max_level(vk::getParam<int>("svo/klt_max_level", 4)),
    klt_min_level(vk::getParam<int>("svo/klt_min_level", 2)),
    reproj_thresh(vk::getParam<double>("svo/reproj_thresh", 2.0)),
    poseoptim_thresh(vk::getParam<double>("svo/poseoptim_thresh", 2.0)),
    poseoptim_num_iter(vk::getParam<int>("svo/poseoptim_num_iter", 10)),
    structureoptim_max_pts(vk::getParam<int>("svo/structureoptim_max_pts", 20)),
    structureoptim_num_iter(vk::getParam<int>("svo/structureoptim_num_iter", 5)),
    loba_thresh(vk::getParam<double>("svo/loba_thresh", 2.0)),
    loba_robust_huber_width(vk::getParam<double>("svo/loba_robust_huber_width", 1.0)),
    loba_num_iter(vk::getParam<int>("svo/loba_num_iter", 5)),
    kfselect_mindist(vk::getParam<double>("svo/kfselect_mindist", 0.12)),
    triang_min_corner_score(vk::getParam<double>("svo/triang_min_corner_score", 20.0)),
    triang_half_patch_size(vk::getParam<int>("svo/triang_half_patch_size", 4)),
    subpix_n_iter(vk::getParam<int>("svo/subpix_n_iter", 10)),
    max_n_kfs(vk::getParam<int>("svo/max_n_kfs", 10)),
    img_imu_delay(vk::getParam<double>("svo/img_imu_delay", 0.0)),
    max_fts(vk::getParam<int>("svo/max_fts", 120)),
    quality_min_fts(vk::getParam<int>("svo/quality_min_fts", 50)),
    quality_max_drop_fts(vk::getParam<int>("svo/quality_max_drop_fts", 40))
#else
    trace_name("svo"),
    trace_dir("/tmp"),
    n_pyr_levels(3),
    use_imu(false),
    core_n_kfs(3),
    map_scale(1.0),
    grid_size(20),
    score_type(0),
    fast_threshold(20),
    init_min_disparity(50.0),
    init_min_tracked(16),
    init_min_inliers(40),
    klt_max_level(3),
    klt_min_level(1),
    reproj_thresh2(5.991),//experiment show that 5.991 is an optimal setting
    poseoptim_thresh(5.0),
    poseoptim_num_iter(10),
    structureoptim_max_pts(20),
    structureoptim_num_iter(5),
    loba_thresh(2.0),
    loba_robust_huber_width(1.0),
    loba_num_iter(5),
    kfselect_mindist(0.12),
    triang_min_corner_score(20.0),
    triang_half_patch_size(4),
    subpix_n_iter(10),
    max_n_kfs(15),
    img_imu_delay(0.0),
    max_fts(500),
    quality_min_fts(50),
    quality_max_drop_fts(40),
    triang_min_disp(3.5f),
    triang_max_cos_rays(0.99995f),
    pdop_thresh(1e8),
    crop_roi_xl(292), crop_roi_xr(949), //for KITTI set to 292 and 949
    use_decay_velocity_model(true),
    temporal_window_size(3),
    spatial_window_size(7)
#endif
{}

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace ORB_SLAM

