#include "IMUErrorModel.h"

template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ag): b_a(b_ag.template head<3>()),
    b_g(b_ag.template tail<3>()),
    S_a(Eigen::Matrix<Scalar, 3, 3>::Zero()),S_g(Eigen::Matrix<Scalar, 3, 3>::Zero()), T_s(Eigen::Matrix<Scalar, 3, 3>::Zero()),
    invT_a(Eigen::Matrix<Scalar, 3, 3>::Identity()),invT_g(Eigen::Matrix<Scalar, 3, 3>::Identity())
{
}
/*template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 33, 1> bundle): b_a(bundle.block<3,1>(0,0)), b_g(bundle.block<3,1>(3, 0))
{
    for (int i=0; i<3; ++i){
        for (int j=0; j<3; ++j){
            S_a(i,j)=bundle(i*3+j+6);
            S_g(i,j)= bundle(i*3+j+15);
            T_s(i,j)=bundle(i*3+j+24);
        }
    }
    invT_a=(S_a+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
    invT_g=(S_g+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
}*/
template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ag, const Eigen::Matrix<Scalar, 27, 1> shapeMatrices):
 b_a(b_ag.template head<3>())/*block<3,1>(0,0))*/, b_g(b_ag.template tail<3>())//block<3,1>(3, 0))
{
    for (int i=0; i<3; ++i){
        for (int j=0; j<3; ++j){
            S_a(i,j)=shapeMatrices(i*3+j);
            S_g(i,j)= shapeMatrices(i*3+j+9);
            T_s(i,j)=shapeMatrices(i*3+j+18);
        }
    }
    invT_a=(S_a+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
    invT_g=(S_g+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
}
template <class Scalar>
void IMUErrorModel<Scalar>::estimate(const Eigen::Matrix<Scalar, 3, 1> a_m, const Eigen::Matrix<Scalar, 3, 1> w_m)
{
    a_est= invT_a*(a_m- b_a);
    w_est= invT_g*(w_m- b_g - T_s* a_est);
}
template <class Scalar>
void IMUErrorModel<Scalar>::predict(const Eigen::Matrix<Scalar, 3, 1> a_s, const Eigen::Matrix<Scalar, 3, 1> w_s)
{
    a_obs = S_a*a_s +a_s+ b_a;
    w_obs = S_g*w_s + w_s + T_s*a_s + b_g;
}
