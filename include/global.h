
#ifndef GLOBAL_H
#define GLOBAL_H

#include <Eigen/Core>
#ifdef SLAM_TRACE
#include <vikit/performance_monitor.h>
#endif

#ifdef SLAM_USE_ROS
  #include <ros/console.h>
  #define SLAM_DEBUG_STREAM(x) ROS_DEBUG_STREAM(x)
  #define SLAM_INFO_STREAM(x) ROS_INFO_STREAM(x)
  #define SLAM_WARN_STREAM(x) ROS_WARN_STREAM(x)
  #define SLAM_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
  #define SLAM_ERROR_STREAM(x) ROS_ERROR_STREAM(x)
#else
  #define SLAM_INFO_STREAM(x) std::cout<<"\033[0;0m[INFO] "<<x<<"\033[0;0m"<<std::endl;
  #ifdef SLAM_DEBUG_OUTPUT
    #define SLAM_DEBUG_STREAM(x) SLAM_INFO_STREAM(x)
  #else
    #define SLAM_DEBUG_STREAM(x)
  #endif
  #define SLAM_WARN_STREAM(x) std::cerr<<"\033[0;33m[WARN] "<<x<<"\033[0;0m"<<std::endl;
  #define SLAM_ERROR_STREAM(x) std::cerr<<"\033[1;31m[ERROR] "<<x<<"\033[0;0m"<<std::endl;
  #include <chrono> // Adapted from rosconsole. Copyright (c) 2008, Willow Garage, Inc.
  #define SLAM_WARN_STREAM_THROTTLE(rate, x) \
    do { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = \
      std::chrono::system_clock::now(); \
      if (__log_stream_throttle__last_hit__ + rate <= \
          std::chrono::duration_cast<std::chrono::seconds>( \
          __log_stream_throttle__now__.time_since_epoch()).count()) { \
        __log_stream_throttle__last_hit__ = \
        std::chrono::duration_cast<std::chrono::seconds>( \
        __log_stream_throttle__now__.time_since_epoch()).count(); \
        SLAM_WARN_STREAM(x); \
      } \
    } while(0)
#endif

namespace ORB_SLAM
{
#ifdef SLAM_TRACE
  extern vk::PerformanceMonitor* g_permon;
  #define SLAM_LOG(value) g_permon->log(std::string((#value)),(value))
  #define SLAM_LOG2(value1, value2) SLAM_LOG(value1); SLAM_LOG(value2)
  #define SLAM_LOG3(value1, value2, value3) SLAM_LOG2(value1, value2); SLAM_LOG(value3)
  #define SLAM_LOG4(value1, value2, value3, value4) SLAM_LOG2(value1, value2); SLAM_LOG2(value3, value4)
  #define SLAM_START_TIMER(name) g_permon->startTimer((name))
  #define SLAM_STOP_TIMER(name) g_permon->stopTimer((name))
#else
  #define SLAM_LOG(v)
  #define SLAM_LOG2(v1, v2)
  #define SLAM_LOG3(v1, v2, v3)
  #define SLAM_LOG4(v1, v2, v3, v4)
  #define SLAM_START_TIMER(name)
  #define SLAM_STOP_TIMER(name)
#endif
const double EPS = 0.0000000001;
const double PI = 3.14159265;
}
#endif
