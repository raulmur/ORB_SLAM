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

    /// Initialization: Minimum number of tracked features.
    static int& initMinTracked() { return getInstance().init_min_tracked; }

    /// Reprojection threshold [px].
    static double& reprojThresh2() { return getInstance().reproj_thresh2; }

    /// the minimum disparity (in pixels) between two observations of stereo frames to triangulate a point
    /// set triang_min_disp as $f sqrt(2-2c)$ where c is triang_max_cos_rays, so that the two are roughly equivalent
    static float & triangMinDisp() { return getInstance().triang_min_disp;}

    /// the maximum cos of rays to triangulate a point
    static float & triangMaxCosRays() { return getInstance().triang_max_cos_rays;}

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

    int init_min_tracked;  
    double reproj_thresh2;
    float triang_min_disp;
    float triang_max_cos_rays;  
    float crop_roi_xl;
    float crop_roi_xr;
    bool use_decay_velocity_model;
    size_t temporal_window_size;
    int spatial_window_size;
};

} // namespace

#endif // ORBSLAM_CONFIG_H_
