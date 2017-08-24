
#include "vio/maths_utils.h"

#include <cassert>
#include <limits>

//#include <visiontools/accessor_macros.h>
//#include <visiontools/abstract_camera.h>

namespace vio
{
using namespace Eigen;
using namespace std;

//TODO: implement proper interpolation!!
//Input: disparity map at level 0, uv_pyr, location at pyramid level level
// return: disparity for uv_pyr at pyramid level
/*double
interpolateDisparity(const cv::Mat & disp,
                     const Eigen::Vector2i & uv_pyr,
                     int level)
{
  double inv_factor = VisionTools::pyrFromZero_d(1.,level);
  return disp.at<float>(uv_pyr[1]<<level,uv_pyr[0]<<level)*inv_factor;  
}
double
interpolateDisparity(const cv::Mat & disp,
                     const Eigen::Vector2f & uv_pyr,
                     int level)
{
  double inv_factor = VisionTools::pyrFromZero_d(1.,level);
  return interpolateMat_32f(disp, uv_pyr*pow(2.,level))*inv_factor;
}

float
interpolateMat_32f(const cv::Mat & mat, const Vector2f & uv)
{
  assert(mat.type()==CV_32F);
  float x = floor(uv[0]);
  float y = floor(uv[1]);
  float subpix_x = uv[0]-x;
  float subpix_y = uv[1]-y;
  float wx0 = 1-subpix_x;
  float wx1 =  subpix_x;
  float wy0 = 1-subpix_y;
  float wy1 =  subpix_y;

  float val00 = mat.at<float>(y,x);
  float val01 = mat.at<float>(y+1,x);
  float val10 = mat.at<float>(y,x+1);
  float val11 = mat.at<float>(y+1,x+1);
  return (wx0*wy0)*val00 + (wx0*wy1)*val01
      + (wx1*wy0)*val10 + (wx1*wy1)*val11;
}

float
interpolateMat_8u(const cv::Mat & mat, const Vector2f & uv)
{
  assert(mat.type()==CV_8U);
  float x = floor(uv[0]);
  float y = floor(uv[1]);
  float subpix_x = uv[0]-x;
  float subpix_y = uv[1]-y;
  float wx0 = 1.f-subpix_x;
  float wx1 =  subpix_x;
  float wy0 = 1.f-subpix_y;
  float wy1 =  subpix_y;

  uint8_t val00 = mat.at<uint8_t>(y,x);
  uint8_t val01 = mat.at<uint8_t>(y+1,x);
  uint8_t val10 = mat.at<uint8_t>(y,x+1);
  uint8_t val11 = mat.at<uint8_t>(y+1,x+1);
  return (wx0*wy0)*val00 + (wx0*wy1)*val01
      + (wx1*wy0)*val10 + (wx1*wy1)*val11;
}
*/

Vector2d
project2d(const Vector3d& v)
{
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector2f
project2f(const Vector3f& v)
{
  Vector2f res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d
project3d(const Vector4d& v)
{
  Vector3d res;
  res(0) = v(0)/v(3);
  res(1) = v(1)/v(3);
  res(2) = v(2)/v(3);
  return res;
}

Vector3d
unproject2d(const Vector2d& v)
{
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}


Vector3f
unproject2f(const Vector2f& v)
{
  Vector3f res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1.f;
  return res;
}

Vector4d
unproject3d(const Vector3d& v)
{
  Vector4d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = v(2);
  res(3) = 1;
  return res;
}

double
norm_max(const VectorXd & v)
{
  double max = -1;
  for (int i=0; i<v.size(); i++)
  {
    double abs = fabs(v[i]);
    if(abs>max){
      max = abs;
    }
  }
  return max;
}

}
