/*
 * pinhole_camera.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: cforster
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>

namespace vk {

PinholeCamera::
PinholeCamera(double width, double height,
              double fx, double fy,
              double cx, double cy,
              double d0, double d1, double d2, double d3, double d4) :
              AbstractCamera(width, height),
              fx_(fx), fy_(fy), cx_(cx), cy_(cy),
              distortion_(fabs(d0) > 0.0000001),
              undist_map1_(height_, width_, CV_16SC2),
              undist_map2_(height_, width_, CV_16SC2),
              use_optimization_(false)
{
  d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;
  cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
  cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
  cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_,
                              cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
  K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  K_inv_ = K_.inverse();
}

PinholeCamera::
~PinholeCamera()
{}

Vector3d PinholeCamera::
cam2world(const double& u, const double& v) const
{
  Vector3d xyz;
  if(!distortion_)
  {
    xyz[0] = (u - cx_)/fx_;
    xyz[1] = (v - cy_)/fy_;
    xyz[2] = 1.0;
  }
  else
  {
    cv::Point2f uv(u,v), px;
    const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
    cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
    xyz[0] = px.x;
    xyz[1] = px.y;
    xyz[2] = 1.0;
  }
  return xyz.normalized();
}

Vector3d PinholeCamera::
cam2world (const Vector2d& uv) const
{
  return cam2world(uv[0], uv[1]);
}

Vector2d PinholeCamera::
world2cam(const Vector3d& xyz) const
{
  return world2cam(project2d(xyz));
}

Vector2d PinholeCamera::
world2cam(const Vector2d& uv) const
{
  Vector2d px;
  if(!distortion_)
  {
    px[0] = fx_*uv[0] + cx_;
    px[1] = fy_*uv[1] + cy_;
  }
  else
  {
    double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
    x = uv[0];
    y = uv[1];
    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
    xd = x*cdist + d_[2]*a1 + d_[3]*a2;
    yd = y*cdist + d_[2]*a3 + d_[3]*a1;
    px[0] = xd*fx_ + cx_;
    px[1] = yd*fy_ + cy_;
  }
  return px;
}

void PinholeCamera::
undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
  if(distortion_)
    cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
  else
    rectified = raw.clone();
}

} // end namespace vk
