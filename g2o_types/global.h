// This file is part of ScaViSLAM.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// ScaViSLAM is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// ScaViSLAM is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with ScaViSLAM.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GLOBAL_H
#define GLOBAL_H

#include <Eigen/Core>
#include <vikit/performance_monitor.h>
//#include <Eigen/StdVector>

/*#include <opencv2/core/core.hpp>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <list>
#include <map>
#include <vector>

#include <tr1/unordered_set>
#include <tr1/unordered_map>
#include <tr1/memory>
// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&);

namespace std
{
namespace tr1
{
template<class T>
class hash<shared_ptr<T> > {
public:
  size_t operator()(const shared_ptr<T>& key) const
  {
    return (size_t)key.get();
  }
};
}
}

namespace VisionTools
{
}
*/
namespace ScaViSLAM
{

//using namespace Eigen;
//using namespace std;
//using namespace VisionTools;

typedef Eigen::Matrix< double, 5, 5 > Matrix5d;
typedef Eigen::Matrix< float, 5, 5 > Matrix5f;

typedef Eigen::Matrix< double, 6, 6 > Matrix6d;
typedef Eigen::Matrix< float, 6, 6 > Matrix6f;

typedef Eigen::Matrix< double, 7, 7 > Matrix7d;
typedef Eigen::Matrix< float, 7, 7 > Matrix7f;

typedef Eigen::Matrix< double, 5, 1 > Vector5d;
typedef Eigen::Matrix< float, 5, 1 > Vector5f;

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix< float, 6, 1 > Vector6f;

typedef Eigen::Matrix< double, 7, 1 > Vector7d;
typedef Eigen::Matrix< float, 7, 1 > Vector7f;

typedef Eigen::Matrix< double, 1, 5 > 	RowVector5d;
typedef Eigen::Matrix< float, 1, 5 > 	RowVector5f;

typedef Eigen::Matrix< double, 1, 6 > 	RowVector6d;
typedef Eigen::Matrix< float, 1, 6 > 	RowVector6f;

typedef Eigen::Matrix< double, 1, 7 > 	RowVector7d;
typedef Eigen::Matrix< float, 1, 7 > 	RowVector7f;

/*
template <int T>
struct VECTOR
{
  typedef Eigen::Matrix<double,T,1> col;
  typedef Eigen::Matrix<double,1,T> row;
};

typedef cv::Rect_<double> Rectangle;

const double EPS = 0.0000000001;
const int NUM_PYR_LEVELS  = 3;

class IntPairHash
{
public:
  size_t operator ()(const std::pair<int,int> & p) const
  {
    return std::tr1::hash<int>()(p.first) + std::tr1::hash<int>()(100000*p.second);
  }
};

// Defining a templated-struced is necessary since templated typedefs are only
// allowed in the upcoming C++0x standard!
template <class T>
struct ALIGNED
{
  typedef std::vector<T, Eigen::aligned_allocator<T> > vector;
  typedef std::list<T, Eigen::aligned_allocator<T> > list;
  typedef std::map<int,T, std::less<int>,

  Eigen::aligned_allocator<std::pair<const int, T> > >
  int_map;
  typedef std::tr1::unordered_map<int,T, std::tr1::hash<int>,  std::equal_to<int>,
  Eigen::aligned_allocator<std::pair<const int, T> > >
  int_hash_map;
  typedef std::tr1::unordered_map<std::pair<int,int>, T, IntPairHash,
  std::equal_to<std::pair<int, int> >,
  Eigen::aligned_allocator<std::pair<const std::pair<int,int>, T> > >
  intpair_hash_map;
};

typedef std::tr1::unordered_map<int,int> IntTable;*/
}
#ifdef SLAM_USE_ROS
  #include <ros/console.h>
  #define SLAM_DEBUG_STREAM(x) ROS_DEBUG_STREAM(x)
  #define SLAM_INFO_STREAM(x) ROS_INFO_STREAM(x)
  #define SLAM_WARN_STREAM(x) ROS_WARN_STREAM(x)
  #define SLAM_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
  #define SLAM_ERROR_STREAM(x) ROS_ERROR_STREAM(x)
#else
  #define SLAM_INFO_STREAM(x) std::cerr<<"\033[0;0m[INFO] "<<x<<"\033[0;0m"<<std::endl;
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
