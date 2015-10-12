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

#ifndef SCAVISLAM_MATHSUTILS_H
#define SCAVISLAM_MATHSUTILS_H

#define GL_GLEXT_PROTOTYPES 1

#include <set>

#include "global.h"

//namespace Sophus
//{
//class SE3d;
//}

namespace ScaViSLAM
{

//using namespace Eigen;
//using namespace std;

/*
class AbstractCamera;

double
interpolateDisparity         (const cv::Mat & m,
                              const Eigen::Vector2i & uv_pyr,
                              int level);
float
interpolateMat_32f           (const cv::Mat & mat,
                              const Eigen::Vector2f & uv);

float
interpolateMat_8u            (const cv::Mat & mat,
                              const Eigen::Vector2f & uv);*/
Eigen::Vector2d
project2d                    (const Eigen::Vector3d&);

Eigen::Vector3d
project3d                    (const Eigen::Vector4d&);

Eigen::Vector3d
unproject2d                  (const Eigen::Vector2d&);

Eigen::Vector4d
unproject3d                  (const Eigen::Vector3d&);

double
norm_max                     (const Eigen::VectorXd & v);

inline Eigen::Vector3d invert_depth(const Eigen::Vector3d & x)
{
  return unproject2d(x.head<2>())/x[2];
}

template <typename T>
inline T Po2(const T& value)
{
  return value * value;
}

template <typename T>
inline T Po3(const T& value)
{
  return Po2(value) * value;
}

template <typename T>
T Po4(const T& value)
{
  return Po2(Po2(value));
}

template <typename T>
T Po5(const T& value)
{
  return Po4(value) * value;
}

template <typename T>
T Po6(const T& value)
{
  return Po4(value) * Po2(value);
}

template <typename T>
T Po7(const T& value)
{
  return Po6(value) * value;
}

template <typename T>
T Po8(const T& value)
{
  return Po2(Po4(value));
}

template <typename T>
T median(const std::multiset<T> & m_set)
{
  int size = m_set.size();
  assert(size>0);
  typename std::multiset<T>::const_iterator it = m_set.begin();
  if (m_set.size()%2==1)
  {
    for (int i=0; i<size/2; ++i)
    {
      it++;
    }
    return *it;
  }

  for (int i = 0; i<size/2-1; ++i)
  {
    it++;
  }
  T val = *it;
  it++;
  val += *it;
  return 0.5*val;
}

}
#endif
