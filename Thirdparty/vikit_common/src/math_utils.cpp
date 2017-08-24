/*
 * math_utils.cpp
 *
 *  Created on: Jul 20, 2012
 *      Author: cforster
 */

#include <vikit/math_utils.h>

namespace vk {

using namespace Eigen;

//Vector3d
//triangulateFeatureNonLin(const Matrix3d& R,  const Vector3d& t,
//                         const Vector3d& feature1, const Vector3d& feature2 )
//{
//  Vector3d f2 = R * feature2;
//  Vector2d b;
//  b[0] = t.dot(feature1);
//  b[1] = t.dot(f2);
//  Matrix2d A;
//  A(0,0) = feature1.dot(feature1);
//  A(1,0) = feature1.dot(f2);
//  A(0,1) = -A(1,0);
//  A(1,1) = -f2.dot(f2);
//  Vector2d lambda = A.inverse() * b;
//  Vector3d xm = lambda[0] * feature1;
//  Vector3d xn = t + lambda[1] * f2;
//  return ( xm + xn )/2;
//}

bool
depthFromTriangulationExact(
    const Matrix3d& R_r_c,
    const Vector3d& t_r_c,
    const Vector3d& f_r,
    const Vector3d& f_c,
    double& depth_in_r,
    double& depth_in_c)
{
  // bearing vectors (f_r, f_c) do not need to be unit length
  const Vector3d f_c_in_r(R_r_c*f_c);
  const double a = f_c_in_r.dot(f_r) / t_r_c.dot(f_r);
  const double b = f_c_in_r.dot(t_r_c);
  const double denom = (a*b - f_c_in_r.dot(f_c_in_r));

  if(abs(denom) < 0.000001)
    return false;

  depth_in_c = (b-a*t_r_c.dot(t_r_c)) / denom;
  depth_in_r = (t_r_c + f_c_in_r*depth_in_c).norm();
  return true;
}

double
reprojError(const Vector3d& f1,
            const Vector3d& f2,
            double error_multiplier2)
{
  Vector2d e = project2d(f1) - project2d(f2);
  return error_multiplier2 * e.norm();
}

//double
//computeInliers(const vector<Vector3d>& features1, // c1
//               const vector<Vector3d>& features2, // c2
//               const Matrix3d& R,                 // R_c1_c2
//               const Vector3d& t,                 // c1_t
//               const double reproj_thresh,
//               double error_multiplier2,
//               vector<Vector3d>& xyz_vec,         // in frame c1
//               vector<int>& inliers,
//               vector<int>& outliers)
//{
//  inliers.clear(); inliers.reserve(features1.size());
//  outliers.clear(); outliers.reserve(features1.size());
//  xyz_vec.clear(); xyz_vec.reserve(features1.size());
//  double tot_error = 0;
//  //triangulate all features and compute reprojection errors and inliers
//  for(size_t j=0; j<features1.size(); ++j)
//  {
//    xyz_vec.push_back(triangulateFeatureNonLin(R, t, features1[j], features2[j] ));
//    double e1 = reprojError(features1[j], xyz_vec.back(), error_multiplier2);
//    double e2 = reprojError(features2[j], R.transpose()*(xyz_vec.back()-t), error_multiplier2);
//    if(e1 > reproj_thresh || e2 > reproj_thresh)
//      outliers.push_back(j);
//    else
//    {
//      inliers.push_back(j);
//      tot_error += e1+e2;
//    }
//  }
//  return tot_error;
//}

void
computeInliersOneView(const vector<Vector3d> & feature_sphere_vec,
                      const vector<Vector3d> & xyz_vec,
                      const Matrix3d &R,
                      const Vector3d &t,
                      const double reproj_thresh,
                      const double error_multiplier2,
                      vector<int>& inliers,
                      vector<int>& outliers)
{
  inliers.clear(); inliers.reserve(xyz_vec.size());
  outliers.clear(); outliers.reserve(xyz_vec.size());
  for(size_t j = 0; j < xyz_vec.size(); j++ )
  {
    double e = reprojError(feature_sphere_vec[j],
                           R.transpose() * ( xyz_vec[j] - t ),
                           error_multiplier2);
    if(e < reproj_thresh)
      inliers.push_back(j);
    else
      outliers.push_back(j);
  }
}

Vector3d
dcm2rpy(const Matrix3d &R)
{
  Vector3d rpy;
  rpy[1] = atan2( -R(2,0), sqrt( pow( R(0,0), 2 ) + pow( R(1,0), 2 ) ) );
  if( fabs( rpy[1] - M_PI/2 ) < 0.00001 )
  {
    rpy[2] = 0;
    rpy[0] = -atan2( R(0,1), R(1,1) );
  }
  else
  {
    if( fabs( rpy[1] + M_PI/2 ) < 0.00001 )
    {
      rpy[2] = 0;
      rpy[0] = -atan2( R(0,1), R(1,1) );
    }
    else
    {
      rpy[2] = atan2( R(1,0)/cos(rpy[1]), R(0,0)/cos(rpy[1]) );
      rpy[0] = atan2( R(2,1)/cos(rpy[1]), R(2,2)/cos(rpy[1]) );
    }
  }
  return rpy;
}

Matrix3d
rpy2dcm(const Vector3d &rpy)
{
  Matrix3d R1;
  R1(0,0) = 1.0; R1(0,1) = 0.0; R1(0,2) = 0.0;
  R1(1,0) = 0.0; R1(1,1) = cos(rpy[0]); R1(1,2) = -sin(rpy[0]);
  R1(2,0) = 0.0; R1(2,1) = -R1(1,2); R1(2,2) = R1(1,1);

  Matrix3d R2;
  R2(0,0) = cos(rpy[1]); R2(0,1) = 0.0; R2(0,2) = sin(rpy[1]);
  R2(1,0) = 0.0; R2(1,1) = 1.0; R2(1,2) = 0.0;
  R2(2,0) = -R2(0,2); R2(2,1) = 0.0; R2(2,2) = R2(0,0);

  Matrix3d R3;
  R3(0,0) = cos(rpy[2]); R3(0,1) = -sin(rpy[2]); R3(0,2) = 0.0;
  R3(1,0) = -R3(0,1); R3(1,1) = R3(0,0); R3(1,2) = 0.0;
  R3(2,0) = 0.0; R3(2,1) = 0.0; R3(2,2) = 1.0;

  return R3 * R2 * R1;
}

Quaterniond
angax2quat(const Vector3d& n, const double& angle)
{
  // n must be normalized!
  double s(sin(angle/2));
  return Quaterniond( cos(angle/2), n[0]*s, n[1]*s, n[2]*s );
}


Matrix3d
angax2dcm(const Vector3d& n, const double& angle)
{
  // n must be normalized
  Matrix3d sqewn(sqew(n));
  return Matrix3d(Matrix3d::Identity() + sqewn*sin(angle) + sqewn*sqewn*(1-cos(angle)));
}

double
sampsonusError(const Vector2d &v2Dash, const Matrix3d& Essential, const Vector2d& v2)
{
  Vector3d v3Dash = unproject2d(v2Dash);
  Vector3d v3 = unproject2d(v2);

  double dError = v3Dash.transpose() * Essential * v3;

  Vector3d fv3 = Essential * v3;
  Vector3d fTv3Dash = Essential.transpose() * v3Dash;

  Vector2d fv3Slice = fv3.head<2>();
  Vector2d fTv3DashSlice = fTv3Dash.head<2>();

  return (dError * dError / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice)));
}

} // end namespace vk
