/*

Copyright (c) 2016, Jianzhu Huai
You can contact the author at <huai dot 3 at osu dot edu>

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

namespace vio {

/// returns the 3D cross product skew symmetric matrix of a given 3D vector
template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew3d(const Eigen::MatrixBase<Derived> & vec)
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

/// computes a quaternion from the 3-element small angle approximation theta
/// this is the exact version of quaternionFromSmallAngle
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
      va[0]=Scalar(0);
      va.block<3, 1>(1,0)=vain;
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
      return vb.block<3,1>(1,0);
  }

  // assume the multiplied quaternion has entries in the [w,x,y,z] order
  template<class Scalar>
  Eigen::Matrix<Scalar, 4, 4> leftQuaternionProductMatrix(
          const Eigen::Quaternion<Scalar>& quat)
  {
      Eigen::Matrix<Scalar,4,4> Ql;
      Eigen::Matrix<Scalar, 3, 1> qv =quat.vec();
      Ql.template bottomRightCorner<3,3>() = Eigen::Matrix<Scalar,3,3>::Identity()*quat.w() + skew3d(qv);

      Ql.template topRightCorner<1,3>() = - qv.transpose();
      Ql.template bottomLeftCorner<3,1>() = qv;
      Ql(0,0) = quat.w();
      return Ql;
  }

  // assume the multiplied quaternion has entries in the [w,x,y,z] order
  template<class Scalar>
  Eigen::Matrix<Scalar, 4, 4> rightQuaternionProductMatrix(
          const Eigen::Quaternion<Scalar>& quat)
  {
      Eigen::Matrix<Scalar,4,4> Qr;
      Eigen::Matrix<Scalar, 3, 1> qv =quat.vec();
      Qr.template bottomRightCorner<3,3>() = Eigen::Matrix<Scalar,3,3>::Identity()*quat.w() - skew3d(qv);

      Qr.template topRightCorner<1,3>() = - qv.transpose();
      Qr.template bottomLeftCorner<3,1>() = qv;
      Qr(0,0) = quat.w();
      return Qr;
  }

Eigen::Vector3d rotro2eu(Eigen::Matrix3d R);
//eul defined in "n": rotate "n" to obtain "b"
//result: Cbn (from b to n) = R_3(-y)R_2(-p)R_1(-r), where R_x(theta) is the rotation matrix that 
//transforms a point from a frame to b frame representation. Rotating a frame theta around x axis gets b frame.
Eigen::Matrix3d roteu2ro(Eigen::Vector3d eul);
  //input: lat, long in radians, height is immaterial
  //output: Ce2n
Eigen::Matrix3d llh2dcm(const Eigen::Vector3d llh);
  
// get the LEFT nullspace basis of matrix A_{mxn}, where m>n A=[Q_1, Q_2][R, 0]', return Q_2
Eigen::MatrixXd nullspace(const Eigen::MatrixXd& A);

/**
 * @brief get the left nullspace basis(Q_2) and column space basis (Q_1) of matrix A_{mxn},
 *        where m>n, A=[Q_1, Q_2][R, 0]', and A is of full column rank
 */

void leftNullspaceAndColumnSpace(const Eigen::MatrixXd &A,
                                        Eigen::MatrixXd *Q2,
                                        Eigen::MatrixXd *Q1);

// template version has much trouble with Eigen::Matrix<double, Eigen::Dynamic, const> inputs
template<class Derived>
inline void leftNullspaceAndColumnSpace(const Eigen::MatrixBase<Derived> & A,
                                        Eigen::MatrixBase<Derived> * Q2,
                                        Eigen::MatrixBase<Derived> * Q1)
{
    int rows= A.rows(), cols= A.cols();
    assert( rows> cols); // "Rows should be greater than columns in computing left nullspace"
    Eigen::HouseholderQR<Eigen::MatrixBase<Derived> > qr(A);
    //don't use column pivoting because in that case Q*R-A!=0
    Eigen::MatrixBase<Derived> Q = qr.householderQ();

    Q2->template resize(rows, rows-cols);
    *Q2 = Q.template block(0,cols,rows,rows-cols);

    Q1->template resize(rows, cols);
    *Q1 = Q.template block(0,0,rows,cols);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> superdiagonal(
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & M);

Eigen::Matrix<double, Eigen::Dynamic, 1> subdiagonal(
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & M);

/**
 * @brief reparameterize_AIDP, reparameterize a anchored inverse depth point stemed from Li et al ICRA14
 *  optimization based estimator design supplemental material eq 37
 * in computing, equation $\rho_j \mathbf{T}_G^{C_j}\mathbf{T}_{C_i}^G\left[\begin{matrix}\alpha_i \\ \beta_i \\ 1\\ \rho_i
 * \end{matrix}\right] = \rho_i \left[\begin{matrix}\alpha_j \\ \beta_j \\ 1\\ \rho_j\end{matrix}\right]$
 * @param Ri Rotation from the old anchor frame i to the global frame
 * @param Rj Rotation from the new anchor frame j to the global frame
 * @param abrhoi old inverse depth representation, \alpha, \beta, \rho =1/z
 * @param pi position of the old anchor frame's origin in the global frame
 * @param pj position of the new anchor frame's origin in the global frame
 * @param abrhoj new inverse depth representation, \alpha, \beta, \rho =1/z
 * @param optional jacobian, $\frac{\partial(\alpha_j, \beta_j, \rho_j)}{\partial(\alpha_i, \beta_i, \rho_i, \mathbf{p}_i, \mathbf{p}_j)}$
 */
void reparameterize_AIDP(const Eigen::Matrix3d &Ri, const Eigen::Matrix3d &Rj, const Eigen::Vector3d &abrhoi,
                         const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, Eigen::Vector3d& abrhoj,
                         Eigen::Matrix<double, 3, 9>* jacobian = (Eigen::Matrix<double, 3, 9> *) (NULL));
/**
 * @brief compute the Jacobians for reparameterization, for debugging
 * @param Ri
 * @param Rj
 * @param abrhoi
 * @param pi
 * @param pj
 * @param abrhoj
 * @param jacobian
 */
void reparameterizeNumericalJacobian(const Eigen::Matrix3d &Ri, const Eigen::Matrix3d &Rj, const Eigen::Vector3d &abrhoi,
                                     const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, Eigen::Vector3d& abrhoj,
                                     Eigen::Matrix<double, 3, 9>& jacobian);
void testReparameterize();

/**
 * @brief extract blocks from matrix A
 * @param A input matrix
 * @param vRowStartInterval pairs of (startIndex 0 based, and interval length) for rows to keep, must be in ascending order
 * @param vColStartInterval pairs of (startIndex 0 based, and interval length) for cols to keep, must be in ascending order
 * @return the matrix with only the remaining entries
 */
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
extractBlocks(const Eigen::MatrixBase<Derived> & A,
                                         const std::vector<std::pair<size_t, size_t> >& vRowStartInterval,
                                         const std::vector<std::pair<size_t, size_t> >& vColStartInterval)
{
    size_t outputRows =0, outputCols =0;
    for(auto itRow = vRowStartInterval.begin(); itRow!= vRowStartInterval.end(); ++itRow){
        outputRows+=itRow->second;
    }
    for(auto itCol = vColStartInterval.begin(); itCol!= vColStartInterval.end(); ++itCol){
        outputCols+=itCol->second;
    }

    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> res(outputRows, outputCols);
    size_t startRow =0, startCol =0;
    for(auto itRow = vRowStartInterval.begin(); itRow!= vRowStartInterval.end(); ++itRow){
        startCol =0;
        for(auto itCol = vColStartInterval.begin(); itCol!= vColStartInterval.end(); ++itCol){
            res.block(startRow, startCol, itRow->second, itCol->second) =
            A.block(itRow->first, itCol->first, itRow->second, itCol->second);
            startCol += itCol->second;
        }
        startRow += itRow->second;
    }
    return res;
}

void testExtractBlocks();

//this is adapted from vee of so3 in sophus
Eigen::Vector3d unskew3d(const Eigen::Matrix3d & Omega);

}

#endif /* EIGEN_UTILS_H_ */
