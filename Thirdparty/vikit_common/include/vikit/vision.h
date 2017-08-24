/*
 * vision.h
 *
 *  Created on: May 14, 2013
 *      Author: cforster
 */

#ifndef VIKIT_VISION_H_
#define VIKIT_VISION_H_

#include <vikit/aligned_mem.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vk
{

//! Return value between 0 and 1
//! WARNING This function does not check whether the x/y is within the border
inline float
interpolateMat_32f(const cv::Mat& mat, float u, float v)
{
  assert(mat.type()==CV_32F);
  float x = floor(u);
  float y = floor(v);
  float subpix_x = u-x;
  float subpix_y = v-y;
  float wx0 = 1.0-subpix_x;
  float wx1 =  subpix_x;
  float wy0 = 1.0-subpix_y;
  float wy1 =  subpix_y;

  float val00 = mat.at<float>(y,x);
  float val10 = mat.at<float>(y,x+1);
  float val01 = mat.at<float>(y+1,x);
  float val11 = mat.at<float>(y+1,x+1);
  return (wx0*wy0)*val00 + (wx1*wy0)*val10 + (wx0*wy1)*val01 + (wx1*wy1)*val11;
}

//! Return value between 0 and 255
//! WARNING This function does not check whether the x/y is within the border
inline float
interpolateMat_8u(const cv::Mat& mat, float u, float v)
{
  assert(mat.type()==CV_8U);
  int x = floor(u);
  int y = floor(v);
  float subpix_x = u-x;
  float subpix_y = v-y;

  float w00 = (1.0f-subpix_x)*(1.0f-subpix_y);
  float w01 = (1.0f-subpix_x)*subpix_y;
  float w10 = subpix_x*(1.0f-subpix_y);
  float w11 = 1.0f - w00 - w01 - w10;

  const int stride = mat.step.p[0];
  unsigned char* ptr = mat.data + y*stride + x;
  return w00*ptr[0] + w01*ptr[stride] + w10*ptr[1] + w11*ptr[stride+1];
}

void halfSample(const cv::Mat& in, cv::Mat& out);

float shiTomasiScore(const cv::Mat& img, int u, int v);

void calcSharrDeriv(const cv::Mat& src, cv::Mat& dst);

#ifdef __SSE2__

/// Used to convert a Kinect depthmap
/// Code by Christian Kerl DVO, GPL Licence
void convertRawDepthImageSse_16u_to_32f(cv::Mat& depth_16u, cv::Mat& depth_32f, float scale);

#endif

} // namespace vk

#endif // VIKIT_VISION_H_
