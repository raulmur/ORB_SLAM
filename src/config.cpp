#include "config.h"
#ifdef SLAM_USE_ROS
#include <ros/console.h>
#endif

namespace ORB_SLAM {

Config::Config():
    init_min_tracked(16),
    reproj_thresh2(5.991),//experiment show that 5.991 is an optimal setting
    triang_min_disp(3.5f),
    triang_max_cos_rays(0.99995f),
    crop_roi_xl(0),
    crop_roi_xr(1241), //for KITTI set to 292 and 949
    use_decay_velocity_model(true),
    temporal_window_size(3),
    spatial_window_size(7)
{
#if defined(SLAM_USE_ROS) && defined(SLAM_DEBUG_OUTPUT)
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
        ros::console::notifyLoggerLevelsChanged();
    }
#endif
}

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace ORB_SLAM

