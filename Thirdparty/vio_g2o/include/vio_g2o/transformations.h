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

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <list>

#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif

//#include <visiontools/linear_camera.h>

#include "vio/maths_utils.h"
//#include "stereo_camera.h"

namespace vio
{

//TODO: clean, hide implementation and remove stuff not needed here

struct AnchoredPoint3d
{
  AnchoredPoint3d(const Eigen::Vector3d & p_a, int frame_id)
    : p_a(p_a), frame_id(frame_id)
  {
  }
  Eigen::Vector3d p_a;
  int frame_id;
};

//$\frac{\partial Kproj(X)}{\partial X}$
inline Eigen::Matrix<double,2,3>
d_proj_d_y(const double & f, const Eigen::Vector3d & xyz)
{
  double z_sq = xyz[2]*xyz[2];
  Eigen::Matrix<double,2,3> J;
  J << f/xyz[2], 0,           -(f*xyz[0])/z_sq,
      0,           f/xyz[2], -(f*xyz[1])/z_sq;
  return J;
}

//$\frac{\partial Kproj(X)}{\partial X}$
inline Eigen::Matrix<double,2,3>
d_proj_d_y(const Eigen::Vector2d & f, const Eigen::Vector3d & xyz)
{
    double z_sq = xyz[2]*xyz[2];
    Eigen::Matrix<double,2,3> J;
    J << f[0]/xyz[2], 0,           -(f[0]*xyz[0])/z_sq,
            0,           f[1]/xyz[2], -(f[1]*xyz[1])/z_sq;
    return J;
}

inline Eigen::Matrix3d
d_stereoproj_d_y(const double & f, double b, const Eigen::Vector3d & xyz)
{
  double z_sq = xyz[2]*xyz[2];
  Eigen::Matrix3d J;
  J << f/xyz[2], 0,           -(f*xyz[0])/z_sq,
      0,           f/xyz[2], -(f*xyz[1])/z_sq,
      f/xyz[2], 0,           -(f*(xyz[0]-b))/z_sq;
  return J;
}
// this d_expy_d_y definition follows Strasdat's derivation in his dissertation
// It is also confirmed in github g2o: g2o/types/sba/types_six_dof_expmap.cpp
// though N.B. g2o used SE3Quat::exp([\omega, \upsilon]), Strasdat used SE3d::exp([\upsilon, \omega])
inline Eigen::Matrix<double,3,6>
d_expy_d_y(const Eigen::Vector3d & y)
{
  Eigen::Matrix<double,3,6> J;
  J.topLeftCorner<3,3>().setIdentity();
  J.bottomRightCorner<3,3>() = -Sophus::SO3::hat(y);
  return J;
}


// $\frac{\partial proj(T \psi^{-1})}{\partial \psi}$
inline Eigen::Matrix3d
d_Tinvpsi_d_psi(const Sophus::SE3d & T, const Eigen::Vector3d & psi)
{
  Eigen::Matrix3d R = T.rotationMatrix();
  Eigen::Vector3d x = invert_depth(psi);
  Eigen::Vector3d r1 = R.col(0);
  Eigen::Vector3d r2 = R.col(1);
  Eigen::Matrix3d J;
  J.col(0) = r1;
  J.col(1) = r2;
  J.col(2) = -R*x;
  J*=1./psi.z();
  return J;
}

//$\frac{\partial \tilde{u}- Kproj(RX+t)}{\partial X}$
inline void
point_jac_xyz2uv(const Eigen::Vector3d & xyz,
                 const Eigen::Matrix3d & R,
                 const double & focal_length,
                 Eigen::Matrix<double,2,3> & point_jac)
{
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  Eigen::Matrix<double,2,3> tmp;
  tmp(0,0) = focal_length;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*focal_length;
  tmp(1,0) = 0;
  tmp(1,1) = focal_length;
  tmp(1,2) = -y/z*focal_length;
  point_jac =  -1./z * tmp * R;
}

//$\frac{\partial \tilde{u}- Kproj(exp(\hat{\epsilon}))X)}{\partial \epsilon}$
inline void
frame_jac_xyz2uv(const Eigen::Vector3d & xyz,
                 const double & focal_length,
                 Eigen::Matrix<double,2,6> & frame_jac)
{
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  double z_2 = z*z;

  frame_jac(0,0) = -1./z *focal_length;
  frame_jac(0,1) = 0;
  frame_jac(0,2) = x/z_2 *focal_length;
  frame_jac(0,3) =  x*y/z_2 * focal_length;
  frame_jac(0,4) = -(1+(x*x/z_2)) *focal_length;
  frame_jac(0,5) = y/z *focal_length;

  frame_jac(1,0) = 0;
  frame_jac(1,1) = -1./z *focal_length;
  frame_jac(1,2) = y/z_2 *focal_length;
  frame_jac(1,3) = (1+y*y/z_2) *focal_length;
  frame_jac(1,4) = -x*y/z_2 *focal_length;
  frame_jac(1,5) = -x/z *focal_length;
}

inline void
frame_jac_xyz2uvu(const Eigen::Vector3d & xyz,
                  const Eigen::Vector2d & focal_length,
                  Eigen::Matrix<double,2,6> & frame_jac)
{
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  double z_2 = z*z;

  frame_jac(0,0) = -1./z *focal_length(0);
  frame_jac(0,1) = 0;
  frame_jac(0,2) = x/z_2 *focal_length(0);
  frame_jac(0,3) =  x*y/z_2 * focal_length(0);
  frame_jac(0,4) = -(1+(x*x/z_2)) *focal_length(0);
  frame_jac(0,5) = y/z *focal_length(0);

  frame_jac(1,0) = 0;
  frame_jac(1,1) = -1./z *focal_length(1);
  frame_jac(1,2) = y/z_2 *focal_length(1);
  frame_jac(1,3) = (1+y*y/z_2) *focal_length(1);
  frame_jac(1,4) = -x*y/z_2 *focal_length(1);
  frame_jac(1,5) = -x/z *focal_length(1);
}

//  /**
//   * Abstract prediction class
//   * Frame: How is the frame/pose represented? (e.g. SE3d)
//   * FrameDoF: How many DoF has the pose/frame? (e.g. 6 DoF, that is
//   *           3 DoF translation, 3 DoF rotation)
//   * PointParNum: number of parameters to represent a point
//   *              (4 for a 3D homogenious point)
//   * PointDoF: DoF of a point (3 DoF for a 3D homogenious point)
//   * ObsDim: dimensions of observation (2 dim for (u,v) image
//   *         measurement)
//   */
template <typename Frame,
          int FrameDoF,
          typename Point,
          int PointDoF,
          int ObsDim>
class AbstractPrediction
{
public:

  /** Map a world point x into the camera/sensor coordinate frame T
       * and create an observation*/
  virtual Eigen::Matrix<double,ObsDim,1>
  map                        (const Frame & T,
                              const Point & x) const = 0;

  virtual Eigen::Matrix<double,ObsDim,1>
  map_n_bothJac              (const Frame & T,
                              const Point & x,
                              Eigen::Matrix<double,ObsDim,FrameDoF> & frame_jac,
                              Eigen::Matrix<double,ObsDim,PointDoF> & point_jac) const
  {
    frame_jac = frameJac(T,x);
    point_jac = pointJac(T,x);
    return map(T,x);
  }

  virtual Eigen::Matrix<double,ObsDim,1>
  map_n_frameJac             (const Frame & T,
                              const Point & x,
                              Eigen::Matrix<double,ObsDim,FrameDoF> & frame_jac) const
  {
    frame_jac = frameJac(T,x);
    return map(T,x);
  }

  virtual Eigen::Matrix<double,ObsDim,1>
  map_n_pointJac             (const Frame & T,
                              const Point & x,
                              Eigen::Matrix<double,ObsDim,PointDoF> & point_jac) const
  {
    point_jac = pointJac(T,x);
    return map(T,x);
  }


  /** Jacobian wrt. frame: use numerical Jacobian as default */
  virtual Eigen::Matrix<double,ObsDim,FrameDoF>
  frameJac                   (const Frame & T,
                              const Point & x) const
  {
    double h = 0.000000000001;
    Eigen::Matrix<double,ObsDim,FrameDoF> J_pose
        = Eigen::Matrix<double,ObsDim,FrameDoF>::Zero();

    Eigen::Matrix<double,ObsDim,1>  fun = -map(T,x);
    for (unsigned int i=0; i<FrameDoF; ++i)
    {
      Eigen::Matrix<double,FrameDoF,1> eps
          = Eigen::Matrix<double,FrameDoF,1>::Zero();
      eps[i] = h;

      J_pose.col(i) = (-map(add(T,eps),x) -fun)/h ;
    }
    return J_pose;
  }

  /** Jacobian wrt. point: use numerical Jacobian as default */
  virtual Eigen::Matrix<double,ObsDim,PointDoF>
  pointJac                   (const Frame & T,
                              const Point & x) const
  {
    double h = 0.000000000001;
    Eigen::Matrix<double,ObsDim,PointDoF> J_x
        = Eigen::Matrix<double,ObsDim,PointDoF>::Zero();
    Eigen::Matrix<double,ObsDim,1> fun = -map(T,x);
    for (unsigned int i=0; i<PointDoF; ++i)
    {
      Eigen::Matrix<double,PointDoF,1> eps
          = Eigen::Matrix<double,PointDoF,1>::Zero();
      eps[i] = h;

      J_x.col(i) = (-map(T,add(x,eps)) -fun)/h ;

    }
    return J_x;
  }

  /** Add an incermental update delta to pose/frame T*/
  virtual Frame
  add                        (const Frame & T,
                              const Eigen::Matrix<double,FrameDoF,1> & delta) const = 0;

  /** Add an incremental update delta to point x*/
  virtual Point
  add                        (const Point & x,
                              const Eigen::Matrix<double,PointDoF,1> & delta) const = 0;
};


template <typename Frame,
          int FrameDoF,
          typename Point,
          int PointDoF,
          int ObsDim>
class AbstractAnchoredPrediction
{
public:

  /** Map a world point x into the camera/sensor coordinate frame T
       * and create an observation*/
  virtual Eigen::Matrix<double,ObsDim,1>
  map                        (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a) const = 0;

  virtual Eigen::Matrix<double,ObsDim,1>
  map_n_bothJac              (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a,
                              Eigen::Matrix<double,ObsDim,FrameDoF> & frame_jac,
                              Eigen::Matrix<double,ObsDim,PointDoF> & point_jac) const
  {
    frame_jac = frameJac(T_cw,A_wa,x_a);
    point_jac = pointJac(T_cw,A_wa,x_a);
    return map(T_cw,A_wa,x_a);
  }

  virtual Eigen::Matrix<double,ObsDim,1>
  map_n_allJac               (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a,
                              Eigen::Matrix<double,ObsDim,FrameDoF> & frame_jac,
                              Eigen::Matrix<double,ObsDim,FrameDoF> & anchor_jac,
                              Eigen::Matrix<double,ObsDim,PointDoF> & point_jac) const
  {
    frame_jac = frameJac(T_cw,A_wa,x_a);
    anchor_jac = anchorJac(T_cw,A_wa,x_a);
    point_jac = pointJac(T_cw,A_wa,x_a);
    return map(T_cw,A_wa,x_a);
  }


  /** Jacobian wrt. frame: use numerical Jacobian as default */
  virtual Eigen::Matrix<double,ObsDim,FrameDoF>
  frameJac                   (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a) const
  {
    double h = 0.000000000001;
    Eigen::Matrix<double,ObsDim,FrameDoF> J_pose
        = Eigen::Matrix<double,ObsDim,FrameDoF>::Zero();

    Eigen::Matrix<double,ObsDim,1>  fun = -map(T_cw,A_wa,x_a);
    for (unsigned int i=0; i<FrameDoF; ++i)
    {
      Eigen::Matrix<double,FrameDoF,1> eps
          = Eigen::Matrix<double,FrameDoF,1>::Zero();
      eps[i] = h;

      J_pose.col(i) = (-map(add(T_cw,eps),A_wa,x_a) -fun)/h ;
    }
    return J_pose;
  }

  /** Jacobian wrt. anchor: use numerical Jacobian as default */
  virtual Eigen::Matrix<double,ObsDim,FrameDoF>
  anchorJac                  (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a) const
  {
    double h = 0.000000000001;
    Eigen::Matrix<double,ObsDim,FrameDoF> J_pose
        = Eigen::Matrix<double,ObsDim,FrameDoF>::Zero();

    Eigen::Matrix<double,ObsDim,1>  fun = -map(T_cw,A_wa,x_a);
    for (unsigned int i=0; i<FrameDoF; ++i)
    {
      Eigen::Matrix<double,FrameDoF,1> eps
          = Eigen::Matrix<double,FrameDoF,1>::Zero();
      eps[i] = h;

      J_pose.col(i) = (-map(T_cw,add(A_wa,eps),x_a) -fun)/h ;
    }
    return J_pose;
  }

  /** Jacobian wrt. point: use numerical Jacobian as default */
  virtual Eigen::Matrix<double,ObsDim,PointDoF>
  pointJac                   (const Frame & T_cw,
                              const Frame & A_wa,
                              const Point & x_a) const
  {
    double h = 0.000000000001;
    Eigen::Matrix<double,ObsDim,PointDoF> J_x
        = Eigen::Matrix<double,ObsDim,PointDoF>::Zero();
    Eigen::Matrix<double,ObsDim,1> fun = -map(T_cw,A_wa,x_a);
    for (unsigned int i=0; i<PointDoF; ++i)
    {
      Eigen::Matrix<double,PointDoF,1> eps
          = Eigen::Matrix<double,PointDoF,1>::Zero();
      eps[i] = h;

      J_x.col(i) = (-map(T_cw,A_wa,add(x_a,eps)) -fun)/h ;

    }
    return J_x;
  }

  /** Add an incermental update delta to pose/frame T*/
  virtual Frame
  add                        (const Frame & T,
                              const Eigen::Matrix<double,FrameDoF,1> & delta
                              ) const = 0;

  /** Add an incremental update delta to point x*/
  virtual Point
  add                        (const Point & x,
                              const Eigen::Matrix<double,PointDoF,1> & delta
                              ) const = 0;
};



/** abstract prediction class dependig on
     * 3D rigid body transformations SE3d */
template <int PointParNum, int PointDoF, int ObsDim>
class SE3_AbstractPoint
    : public AbstractPrediction
    <Sophus::SE3d,6,Eigen::Matrix<double, PointParNum,1>,PointDoF,ObsDim>
{
public:
  Sophus::SE3d add(const Sophus::SE3d &T, const Eigen::Matrix<double,6,1> & delta) const
  {
    return Sophus::SE3d::exp(delta)*T;
  }
};

/*
class SE3XYZ_STEREO: public SE3_AbstractPoint<3, 3, 3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3XYZ_STEREO              (const StereoCamera & cam)
    : _cam(cam)
  {
  }

  Eigen::Matrix<double,3,6>
  frameJac(const Sophus::SE3d & se3,
           const Eigen::Vector3d & xyz)const
  {
    const Eigen::Vector3d & xyz_trans = se3*xyz;
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double f = _cam.focal_length();

    double one_b_z = 1./z;
    double one_b_z_sq = 1./(z*z);
    double A = -f*one_b_z;
    double B = -f*one_b_z;
    double C = f*x*one_b_z_sq;
    double D = f*y*one_b_z_sq;
    double E = f*(x-_cam.baseline())*one_b_z_sq;

    Eigen::Matrix<double, 3, 6> jac;
    jac <<  A, 0, C, y*C,     z*A-x*C, -y*A,
        0, B, D,-z*B+y*D, -x*D,     x*B,
        A, 0, E, y*E,     z*A-x*E, -y*A;
    return jac;
  }

  Eigen::Vector3d map(const Sophus::SE3d & T,
               const Eigen::Vector3d& xyz) const
  {
    return _cam.map_uvu(T*xyz);
  }

  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }
  void setCam(const StereoCamera cam)
  {
      _cam=cam;
  }
private:
  StereoCamera _cam;
};*/

#ifdef MONO
/*
class Sim3XYZ : public AbstractPrediction<Sophus::Sim3d,6,Vector3d,3,2>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sim3XYZ(const LinearCamera & cam)
  {
    this->cam = cam;
  }

  inline Vector2d map(const Sophus::Sim3d & T,
                      const Vector3d& x) const
  {
    return cam.map(project2d(T*x));
  }


  Vector3d add(const Vector3d & x,
               const Vector3d & delta) const
  {
    return x+delta;
  }

  Sophus::Sim3d add(const Sophus::Sim3d &T, const Eigen::Matrix<double,6,1> & delta) const
  {
    Eigen::Matrix<double,7,1> delta7;
    delta7.head<6>() = delta;
    delta7[6] = 0;
    return Sophus::Sim3d::exp(delta7)*T;
  }

private:
  LinearCamera  cam;

};

class Sim3XYZ_STEREO : public AbstractPrediction<Sophus::Sim3d,7,Vector3d,3,3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sim3XYZ_STEREO(const StereoCamera & cam)
  {
    this->cam = cam;
  }

  inline Vector3d map(const Sophus::Sim3d & T,
                      const Vector3d& x) const
  {
    return cam.map_uvu(T*x);
  }

  Vector3d add(const Vector3d & x,
               const Vector3d & delta) const
  {
    return x+delta;
  }

  Sophus::Sim3d add(const Sophus::Sim3d &T, const Eigen::Matrix<double,7,1> & delta) const
  {
    return Sophus::Sim3d::exp(delta)*T;
  }

private:
  StereoCamera  cam;

};*/

class AbsoluteOrient : public AbstractPrediction<Sophus::Sim3d,7,Eigen::Vector3d,3,3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AbsoluteOrient()
  {
  }

  inline Eigen::Vector3d map(const Sophus::Sim3d & T,
                      const Eigen::Vector3d& x) const
  {
    return T*x;
  }

  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }

  Sophus::Sim3d add(const Sophus::Sim3d &T, const Eigen::Matrix<double,7,1> & delta) const
  {
    return Sophus::Sim3d::exp(delta)*T;
  }
};
#endif



/** 3D Euclidean point class */
/*
class SE3XYZ: public SE3_AbstractPoint<3, 3, 2>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3XYZ(const VisionTools::LinearCamera & cam)
  {
    this->cam = cam;
  }

  inline Eigen::Vector2d map(const Sophus::SE3d & T,
                      const Eigen::Vector3d& x) const
  {
    return cam.map(project2d(T*x));
  }

  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }

private:
  VisionTools::LinearCamera  cam;

};
*/

/** 3D inverse depth point class*/
/*
class SE3UVQ : public SE3_AbstractPoint<3, 3, 2>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3UVQ ()
  {
  }

  SE3UVQ (const VisionTools::LinearCamera & cam_pars)
  {
    this->cam = cam_pars;
  }

  inline Eigen::Vector2d map(const Sophus::SE3d & T,
                      const Eigen::Vector3d& uvq_w) const
  {
    Eigen::Vector3d xyz_w = invert_depth(uvq_w);
    return cam.map(project2d(T*xyz_w));
  }

  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }

private:
  VisionTools::LinearCamera  cam;
};
*/

/** 3D inverse depth point class*/
/*
class SE3AnchordUVQ : public AbstractAnchoredPrediction<Sophus::SE3d,6,Eigen::Vector3d,3,2>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3AnchordUVQ ()
  {
  }

  SE3AnchordUVQ (const VisionTools::LinearCamera & cam_pars)
  {
    this->cam = cam_pars;
  }

  inline Eigen::Vector2d map(const Sophus::SE3d & T_cw,
                      const Sophus::SE3d & A_aw,
                      const Eigen::Vector3d& uvq_a) const
  {
    Eigen::Vector3d xyz_w = A_aw.inverse()*invert_depth(uvq_a);
    return cam.map(project2d(T_cw*xyz_w));
  }

  Eigen::Vector3d add(const Eigen::Vector3d & point,
               const Eigen::Vector3d & delta) const
  {
    return point+delta;
  }

  Eigen::Matrix<double,2,3>
  pointJac(const Sophus::SE3d & T_cw,
           const Sophus::SE3d & A_aw,
           const Eigen::Vector3d & psi_a) const
  {
    Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d y = T_ca*invert_depth(psi_a);
    Eigen::Matrix<double,2,3> J1
        = d_proj_d_y(cam.focal_length(),y);

    Eigen::Matrix3d J2 = d_Tinvpsi_d_psi(T_ca,  psi_a);
    return -J1*J2;

  }

  Eigen::Matrix<double,2,6>
  frameJac(const Sophus::SE3d & T_cw,
           const Sophus::SE3d & A_aw,
           const Eigen::Vector3d & psi_a) const
  {
      Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d y = T_ca*invert_depth(psi_a);
    Eigen::Matrix<double,2,3> J1 = d_proj_d_y(cam.focal_length(),y);
    Eigen::Matrix<double,3,6> J2 = d_expy_d_y(y);
    return -J1*J2;
  }

  Eigen::Matrix<double,2,6>
  anchorJac(const Sophus::SE3d & T_cw,
            const Sophus::SE3d & A_aw,
            const Eigen::Vector3d & psi_a) const
  {
    Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d x = invert_depth(psi_a);
    Eigen::Vector3d y = T_ca*x;
     Eigen::Matrix<double,2,3> J1
        = d_proj_d_y(cam.focal_length(),y);
    Eigen::Matrix<double,3,6> d_invexpx_dx
        = -d_expy_d_y(x);
    return -J1*T_ca.rotationMatrix()*d_invexpx_dx;
  }

  Sophus::SE3d add(const Sophus::SE3d &T, const Eigen::Matrix<double,6,1> & delta) const
  {
    return Sophus::SE3d::exp(delta)*T;
  }

private:
  VisionTools::LinearCamera  cam;
};
*/

/** 3D inverse depth point class*/
/*
class SE3NormUVQ : public AbstractPrediction<Sophus::SE3d,5,Eigen::Vector3d,3,2>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3NormUVQ ()
  {
  }

  SE3NormUVQ (const VisionTools::LinearCamera & cam_pars)
  {
    this->cam = cam_pars;
  }

  inline Eigen::Vector2d map(const Sophus::SE3d & T_cw,
                      const Eigen::Vector3d& uvq_w) const
  {
    Eigen::Vector3d xyz_w = invert_depth(uvq_w);
    return cam.map(project2d(T_cw*xyz_w));
  }

  Eigen::Vector3d add(const Eigen::Vector3d & point,
               const Eigen::Vector3d & delta) const
  {
    return point+delta;
  }

  Sophus::SE3d add(const Sophus::SE3d &T, const Eigen::Matrix<double,5,1> & delta) const
  {
    Vector6d delta6;
    delta6[0] = delta[0];
    delta6[1] = delta[1];
    delta6[2] = 0;
    delta6.tail<3>() = delta.tail<3>();

    Sophus::SE3d new_T = Sophus::SE3d::exp(delta6)*T;
    double length = new_T.translation().norm();
    assert(fabs(length)>0.00001);

    new_T.translation() *= 1./length;
    assert(fabs(new_T.translation().norm()-1) < 0.00001);

    return new_T;
  }


private:
  VisionTools::LinearCamera  cam;
};
*/

/** 3D inverse depth point class*/
/*
class SE3AnchordUVQ_STEREO
    : public AbstractAnchoredPrediction<Sophus::SE3d,6,Eigen::Vector3d,3,3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3AnchordUVQ_STEREO ()
  {
  }

  SE3AnchordUVQ_STEREO (const StereoCamera & cam_pars)
  {
    this->cam = cam_pars;
  }

  inline Eigen::Vector3d map(const Sophus::SE3d & T_cw,
                      const Sophus::SE3d & A_aw,
                      const Eigen::Vector3d& uvq_a) const
  {
    Eigen::Vector3d xyz_w = A_aw.inverse()*invert_depth(uvq_a);
    return cam.map_uvu(T_cw*xyz_w);
  }

  Eigen::Matrix3d
  pointJac(const Sophus::SE3d & T_cw,
           const Sophus::SE3d & A_aw,
           const Eigen::Vector3d & psi_a) const
  {
    Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d y = T_ca*invert_depth(psi_a);
    Eigen::Matrix3d J1
        = d_stereoproj_d_y(cam.focal_length(),
                           cam.baseline(),
                           y);
    Eigen::Matrix3d J2
        = d_Tinvpsi_d_psi(T_ca,
                          psi_a);
    return -J1*J2;
  }

  Eigen::Matrix<double,3,6>
  frameJac(const Sophus::SE3d & T_cw,
           const Sophus::SE3d & A_aw,
           const Eigen::Vector3d & psi_a) const
  {
    Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d y = T_ca*invert_depth(psi_a);
    Eigen::Matrix3d J1
        = d_stereoproj_d_y(cam.focal_length(),
                           cam.baseline(),
                           y);
    Eigen::Matrix<double,3,6> J2
        = d_expy_d_y(y);
    return -J1*J2;
  }


  Eigen::Matrix<double,3,6>
  anchorJac(const Sophus::SE3d & T_cw,
            const Sophus::SE3d & A_aw,
            const Eigen::Vector3d & psi_a) const
  {
    Sophus::SE3d T_ca = T_cw*A_aw.inverse();
    Eigen::Vector3d x = invert_depth(psi_a);
    Eigen::Vector3d y = T_ca*x;
    Eigen::Matrix3d J1
        = d_stereoproj_d_y(cam.focal_length(),
                           cam.baseline(),
                           y);
    Eigen::Matrix<double,3,6> d_invexpx_dx
        = -d_expy_d_y(x);
    return -J1*T_ca.rotationMatrix()*d_invexpx_dx;
  }

  Eigen::Vector3d add(const Eigen::Vector3d & point,
               const Eigen::Vector3d & delta) const
  {
    return point+delta;
  }

  Sophus::SE3d add(const Sophus::SE3d &T, const Eigen::Matrix<double,6,1> & delta) const
  {
    return Sophus::SE3d::exp(delta)*T;
  }

private:
  StereoCamera  cam;
};
*/
/** 3D inverse depth point class*/
/*
class SE3UVU_STEREO : public SE3_AbstractPoint<3, 3, 3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3UVU_STEREO ()
  {
  }

  SE3UVU_STEREO (const StereoCamera & cam)
  {
    this->cam = cam;
  }

  inline Eigen::Vector3d map(const Sophus::SE3d & T,
                      const Eigen::Vector3d& uvu) const
  {
    Eigen::Vector3d x = cam.unmap_uvu(uvu);
    return cam.map_uvu(T*x);
  }


  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }

private:
  StereoCamera  cam;
};
*/

/** 3D inverse depth point class*/
/*
class SE3UVQ_STEREO : public SE3_AbstractPoint<3, 3, 3>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE3UVQ_STEREO ()
  {
  }

  SE3UVQ_STEREO (const StereoCamera & cam)
  {
    this->cam = cam;
  }

  inline Eigen::Vector3d map(const Sophus::SE3d & T,
                      const Eigen::Vector3d& uvq) const
  {
    Eigen::Vector3d x = invert_depth(uvq);
    return cam.map_uvu(T*x);
  }

  Eigen::Vector3d add(const Eigen::Vector3d & x,
               const Eigen::Vector3d & delta) const
  {
    return x+delta;
  }

private:
  StereoCamera  cam;
};
*/

/** observation class */
template <int ObsDim>
class IdObs
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IdObs(){}
  IdObs(int point_id, int frame_id, const Eigen::Matrix<double,ObsDim,1> & obs)
    : frame_id(frame_id), point_id(point_id), obs(obs)
  {
  }

  int frame_id;
  int point_id;// point id w.r.t obs_list 0 based of track_data
  Eigen::Matrix<double,ObsDim,1> obs;// observation at level 0
};


/** observation class with inverse uncertainty*/
template <int ObsDim>
class IdObsLambda : public IdObs<ObsDim>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IdObsLambda(){}
  IdObsLambda(int point_id,
              int frame_id,
              const Eigen::Matrix<double,ObsDim,1> & obs,
              const Eigen::Matrix<double,ObsDim,ObsDim> & lambda)
    : IdObs<ObsDim>(point_id, frame_id,  obs) , lambda(lambda)
  {
  }
  Eigen::Matrix<double,ObsDim,ObsDim> lambda;
};

} //namespace


#endif
