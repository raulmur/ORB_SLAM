/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
//#include"Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
//#include"Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "viso2/matrix.h"

namespace ORB_SLAM
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static Sophus::Sim3d toSim3d(const Sophus::SE3d &se3);
    static Sophus::Sim3d toSim3d(const g2o::Sim3 &se3q);
    static Sophus::SE3d toSE3d(const Sophus::Sim3d &sim3);
    static Sophus::SE3d toSE3d(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const Sophus::SE3d &se3d);
    static Sophus::SE3d toSE3d(const g2o::SE3Quat &se3q);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);

    static Sophus::SE3d toSE3d(const libviso2::Matrix& Tr);
    static libviso2::Matrix toViso2Matrix(const Sophus::SE3d & Tr);

    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat toCvSE3(const Sophus::SE3d &T );
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,2,1> toVector2d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);

    static cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
