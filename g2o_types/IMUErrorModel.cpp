#include "IMUErrorModel.h"

IMUErrorModel::IMUErrorModel(const Eigen::Matrix<double, 6, 1> b_ag): b_a(b_ag.head<3>()), b_g(b_ag.tail<3>()),
    S_a(Eigen::Matrix3d::Zero()),S_g(Eigen::Matrix3d::Zero()), T_s(Eigen::Matrix3d::Zero()),
    invT_a(Eigen::Matrix3d::Identity()),invT_g(Eigen::Matrix3d::Identity())
{
}
IMUErrorModel::IMUErrorModel(const Eigen::Matrix<double, 33, 1> bundle): b_a(bundle.block<3,1>(0,0)), b_g(bundle.block<3,1>(3, 0))
{
    for (int i=0; i<3; ++i){
        for (int j=0; j<3; ++j){
            S_a(i,j)=bundle(i*3+j+6);
            S_g(i,j)= bundle(i*3+j+15);
            T_s(i,j)=bundle(i*3+j+24);
        }
    }
    invT_a=(S_a+ Eigen::Matrix3d::Identity()).inverse();
    invT_g=(S_g+ Eigen::Matrix3d::Identity()).inverse();
}

void IMUErrorModel::estimate(const Eigen::Vector3d a_m, const Eigen::Vector3d w_m)
{
    a_est= invT_a*(a_m- b_a);
    w_est= invT_g*(w_m- b_g - T_s* a_est);
}
void IMUErrorModel::predict(const Eigen::Vector3d a_s, const Eigen::Vector3d w_s)
{
    a_obs = S_a*a_s +a_s+ b_a;
    w_obs = S_g*w_s + w_s + T_s*a_s + b_g;
}
