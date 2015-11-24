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

#ifndef ORBSLAM_CONFIG_H_
#define ORBSLAM_CONFIG_H_

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace ORB_SLAM {

using std::string;

/// Global configuration file of ORB_SLAM contains these parameters remain relatively stable over tests
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.
class Config
{
public:
    static Config& getInstance();

    /// Base-name of the tracefiles.
    static string& traceName() { return getInstance().trace_name; }

    /// Directory where the tracefiles are saved.
    static string& traceDir() { return getInstance().trace_dir; }

    /// Number of pyramid levels used for features.
    static size_t& nPyrLevels() { return getInstance().n_pyr_levels; }

    /// Use the IMU to get relative rotations.
    static bool& useImu() { return getInstance().use_imu; }

    /// Number of keyframes in the core. The core-kfs are optimized through bundle adjustment.
    static size_t& coreNKfs() { return getInstance().core_n_kfs; }

    /// Initial scale of the map. Depends on the distance the camera is moved for the initialization.
    static double& mapScale() { return getInstance().map_scale; }

    /// Feature grid size of a cell in [px].
    static size_t& gridSize() { return getInstance().grid_size; }
    static int& scoreType() { return getInstance().score_type; }
    static int& fastThreshold() { return getInstance().fast_threshold; }
    /// Initialization: Minimum required disparity between two frames' observations.
    static double& initMinDisparity() { return getInstance().init_min_disparity; }

    /// Initialization: Minimum number of tracked features.
    static int& initMinTracked() { return getInstance().init_min_tracked; }

    /// Initialization: Minimum number of inliers after RANSAC.
    static size_t& initMinInliers() { return getInstance().init_min_inliers; }

    /// Maximum level of the Lucas Kanade tracker.
    static size_t& kltMaxLevel() { return getInstance().klt_max_level; }

    /// Minimum level of the Lucas Kanade tracker.
    static size_t& kltMinLevel() { return getInstance().klt_min_level; }

    /// Reprojection threshold [px].
    static double& reprojThresh2() { return getInstance().reproj_thresh2; }

    /// Reprojection threshold after pose optimization.
    static double& poseOptimThresh() { return getInstance().poseoptim_thresh; }

    /// Number of iterations in local bundle adjustment.
    static size_t& poseOptimNumIter() { return getInstance().poseoptim_num_iter; }

    /// Maximum number of points to optimize at every iteration.
    static size_t& structureOptimMaxPts() { return getInstance().structureoptim_max_pts; }

    /// Number of iterations in structure optimization.
    static size_t& structureOptimNumIter() { return getInstance().structureoptim_num_iter; }

    /// Reprojection threshold after bundle adjustment.
    static double& lobaThresh() { return getInstance().loba_thresh; }

    /// Threshold for the robust Huber kernel of the local bundle adjustment.
    static double& lobaRobustHuberWidth() { return getInstance().loba_robust_huber_width; }

    /// Number of iterations in the local bundle adjustment.
    static size_t& lobaNumIter() { return getInstance().loba_num_iter; }

    /// Minimum distance between two keyframes. Relative to the average height in the map.
    static double& kfSelectMinDist() { return getInstance().kfselect_mindist; }

    /// Select only features with a minimum Harris corner score for triangulation.
    static double& triangMinCornerScore() { return getInstance().triang_min_corner_score; }

    /// Subpixel refinement of reprojection and triangulation. Set to 0 if no subpix refinement required!
    static size_t& subpixNIter() { return getInstance().subpix_n_iter; }

    /// Limit the number of keyframes in the map. This makes nslam essentially.
    /// a Visual Odometry. Set to 0 if unlimited number of keyframes are allowed.
    /// Minimum number of keyframes is 3.
    static size_t& maxNKfs() { return getInstance().max_n_kfs; }

    /// How much (in milliseconds) is the camera delayed with respect to the imu.
    static double& imgImuDelay() { return getInstance().img_imu_delay; }

    /// Maximum number of features that should be tracked.
    static size_t& maxFts() { return getInstance().max_fts; }

    /// If the number of tracked features drops below this threshold. Tracking quality is bad.
    static size_t& qualityMinFts() { return getInstance().quality_min_fts; }

    /// If within one frame, this amount of features are dropped. Tracking quality is bad.
    static int& qualityMaxFtsDrop() { return getInstance().quality_max_drop_fts; }

    /// the minimum disparity (in pixels) between two observations of stereo frames to triangulate a point
    /// set triang_min_disp as $f sqrt(2-2c)$ where c is triang_max_cos_rays, so that the two are roughly equivalent
    static float & triangMinDisp() { return getInstance().triang_min_disp;}

    /// the maximum cos of rays to triangulate a point
    static float & triangMaxCosRays() { return getInstance().triang_max_cos_rays;}
    /// the threshold of PDOP (the sqrt of trace of (J*J)^(-1)/f_x) to reject a point candidate
    static double & PDOPThresh(){return getInstance().pdop_thresh;}
    /// features outside crop ROI is not considered for motion estimation,
    /// xl is the minimum x value of a feature, xr is the maximum, if do not want crop effect, then set
    /// crop_roi_xl 0, crop_roi_xr column number of image
    static float & cropROIXL(){ return getInstance().crop_roi_xl;}
    static float & cropROIXR(){ return getInstance().crop_roi_xr;}
    /// do we use a decay motion model
    static bool &useDecayVelocityModel(){return getInstance().use_decay_velocity_model;}
    /// temporal window size not counting the previous and current frame
    static size_t &temporalWindowSize(){return getInstance().temporal_window_size;}
    /// spatial window size of keyframes
    static int &spatialWindowSize(){return getInstance().spatial_window_size;}
private:
    Config();
    Config(Config const&);
    void operator=(Config const&);
    string trace_name;
    string trace_dir;
    size_t n_pyr_levels;
    bool use_imu;
    size_t core_n_kfs;
    double map_scale;
    size_t grid_size;
    int score_type; // ORB Extractor: Score to sort features. 0 -> Harris Score, 1 -> FAST Score
    int fast_threshold; //ORB Extractor: Fast threshold (lower less restrictive)
    double init_min_disparity;
    int init_min_tracked;
    size_t init_min_inliers;
    size_t klt_max_level;
    size_t klt_min_level;
    double reproj_thresh2;
    double poseoptim_thresh;
    size_t poseoptim_num_iter;
    size_t structureoptim_max_pts;
    size_t structureoptim_num_iter;
    double loba_thresh;
    double loba_robust_huber_width;
    size_t loba_num_iter;
    double kfselect_mindist;
    double triang_min_corner_score;
    size_t triang_half_patch_size;
    size_t subpix_n_iter;
    size_t max_n_kfs;
    double img_imu_delay;
    size_t max_fts;
    size_t quality_min_fts;
    int quality_max_drop_fts;

    float triang_min_disp;
    float triang_max_cos_rays;
    double pdop_thresh;
    float crop_roi_xl;
    float crop_roi_xr;
    bool use_decay_velocity_model;
    size_t temporal_window_size;
    int spatial_window_size;
};

} // namespace

#endif // ORBSLAM_CONFIG_H_
