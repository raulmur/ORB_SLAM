/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef EIGEN_UTILS_H_
#define EIGEN_UTILS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
/// returns the 3D cross product skew symmetric matrix of a given 3D vector
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> & vec)
  {
      typedef typename Derived::Scalar Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << Scalar(0.0), -vec[2], vec[1], vec[2],
            Scalar(0.0), -vec[0], -vec[1], vec[0], Scalar(0.0)).finished();
  }

/// returns a matrix with angular velocities used for quaternion derivatives/integration with the JPL notation
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param <vec> {3D vector with angular velocities}
 \return {4x4 matrix for multiplication with the quaternion}
 */
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatJPL(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (
        Eigen::Matrix<typename Derived::Scalar, 4, 4>() <<
        0, vec[2], -vec[1], vec[0],
        -vec[2], 0, vec[0], vec[1],
        vec[1], -vec[0], 0, vec[2],
        -vec[0], -vec[1], -vec[2], 0
        ).finished();
  }

/// returns a matrix with angular velocities used for quaternion derivatives/integration with the Hamilton notation
/**
 The quaternion to be multiplied with this matrix has to be in the order x y z w !!!
 \param <vec> {3D vector with angular velocities}
 \return {4x4 matrix for multiplication with the quaternion}
 */
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 4, 4> omegaMatHamilton(const Eigen::MatrixBase<Derived> & vec)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (
        Eigen::Matrix<typename Derived::Scalar, 4, 4>() <<
        0, -vec[2], vec[1], vec[0],
        vec[2], 0, -vec[0], vec[1],
        -vec[1], vec[0], 0, vec[2],
        -vec[0], -vec[1], -vec[2], 0
        ).finished();
  }

/// computes a quaternion from the 3-element small angle approximation theta
// this is a good approximation of AxisAngleType to QuaternionType in Eigen 3
template<class Derived>
  Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta)
  {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / Scalar(4.0);

    if ( q_squared < Scalar(1))
    {
      return Eigen::Quaternion<Scalar>(sqrt(Scalar(1) - q_squared),
                                       theta[0] * Scalar(0.5), theta[1] * Scalar(0.5), theta[2] * Scalar(0.5));
    }
    else
    {
      const Scalar w = 1.0 / sqrt(Scalar(1) + q_squared);
      const Scalar f = w*0.5;
      return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
    }
  }

/// debug output to check misbehavior of Eigen
template<class T>
  bool checkForNumeric(const T & vec, int size, const std::string & info)
  {
    for (int i = 0; i < size; i++)
    {
      if (isnan(vec[i]))
      {
        std::cerr << "=== ERROR ===  " << info << ": NAN at index " << i << std::endl;
        return false;
      }
      if (isinf(vec[i]))
      {
        std::cerr << "=== ERROR ===  " << info << ": INF at index " << i << std::endl;
        return false;
      }
    }
    return true;
  }


  template<class Derived>
  Eigen::Quaternion<typename Derived::Scalar> rvec2quat(const Eigen::MatrixBase<Derived> & rvec)
  {
      typedef typename Derived::Scalar Scalar;
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
      //normalize
      Scalar rot_ang=rvec.norm();    //assume always positive
      if (rot_ang<Scalar(1e-18))
          return Eigen::Quaternion<Scalar>(Scalar(1), Scalar(0),Scalar(0),Scalar(0));
      else{
          Scalar f= sin(rot_ang*Scalar(0.5))/rot_ang;
          return Eigen::Quaternion<Scalar>(cos(rot_ang*Scalar(0.5)), rvec[0] * f, rvec[1] * f, rvec[2] * f);
      }
  }
  // qc2b=qa2b*qc2a, qa2b->quaternion, qc2a->matrix
  //dir==0, quaternion*matrix, dir==1, quaternion'*matrix
  //dir==2, quaternion*matrix'
  template<class Derived>
  Eigen::MatrixBase<Derived> quatmult(const Eigen::MatrixBase<Derived> & quaternion,
                                      const Eigen::MatrixBase<Derived> & matrix, int dir)
  {
      typedef typename Derived::Scalar Scalar;
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4);

      Scalar a1 = quaternion[0];
      Scalar a2 = quaternion[1];
      Scalar a3 = quaternion[2];
      Scalar a4 = quaternion[3];
      Scalar b1 = matrix[0];
      Scalar b2 = matrix[1];
      Scalar b3 = matrix[2];
      Scalar b4 = matrix[3];
      switch (dir)
      {
      case 0:
      default: break;
      case 1:
          a2 = -a2;
          a3 = -a3;
          a4 = -a4;
          break;
      case 2:
          b2=-b2;
          b3=-b3;
          b4=-b4;
          break;
      }
      Eigen::Matrix<Scalar,4,1> result;
      result[0] = a1 * b1 - a2 * b2 - a3 * b3 - a4 * b4;
      result[1] = a1 * b2 + a2 * b1 + a3 * b4 - a4 * b3;
      result[2] = a1 * b3 - a2 * b4 + a3 * b1 + a4 * b2;
      result[3] = a1 * b4 + a2 * b3 - a3 * b2 + a4 * b1;

      return result;
  }
  template<class Derived>
  Eigen::MatrixBase<Derived>  quatrot(const Eigen::Quaternion<typename Derived::Scalar> & qab,
                                      const Eigen::MatrixBase<Derived> & vain, int dir)
  {
      typedef typename Derived::Scalar Scalar;
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);

      //dir=0 -> q=qab (vb=Cab*va)
      //dir=1 -> q=qba (vb=Cba'*va)
      Eigen::Matrix<Scalar, 4,1 > va;
      va[0]=0;
      va.tail(3)=vain;
      Eigen::Matrix<Scalar, 4,1 > q;
      if (dir == Scalar(0))
      {
          q[0] = qab.w();
          q[1] = qab.x();
          q[2] = qab.y();
          q[3] = qab.z();
      }
      else
      {
          q[0] = qab.w();
          q[1] = -qab.x();
          q[2] = -qab.y();
          q[3] = -qab.z();
      }

      Eigen::Matrix<Scalar, 4,1 > vr_a = quatmult(q, va);
      q[1] = -q[1];
      q[2] = -q[2];
      q[3] = -q[3];
      Eigen::Matrix<Scalar, 4,1 > vb = quatmult(vr_a, q);
      return vb.tail(3);
  }


Eigen::Vector3d rotro2eu(Eigen::Matrix3d R);
  //eul defined in "n": rotate "n" to obtain "b"
  //result: Cbn (from b to n)
Eigen::Matrix3d roteu2ro(Eigen::Vector3d eul);
  //input: lat, long in radians, height is immaterial
  //output: Ce2n
Eigen::Matrix3d llh2dcm( Eigen::Vector3d &llh);
  

#endif /* EIGEN_UTILS_H_ */
