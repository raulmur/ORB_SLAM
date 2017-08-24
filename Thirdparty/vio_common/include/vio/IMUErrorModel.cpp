#include "vio/IMUErrorModel.h"


template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ga): b_g(b_ga.template head<3>()),
    b_a(b_ga.template tail<3>()),
    S_a(Eigen::Matrix<Scalar, 3, 3>::Zero()),S_g(Eigen::Matrix<Scalar, 3, 3>::Zero()), T_s(Eigen::Matrix<Scalar, 3, 3>::Zero()),
    invT_a(Eigen::Matrix<Scalar, 3, 3>::Identity()),invT_g(Eigen::Matrix<Scalar, 3, 3>::Identity())
{
}

template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ga, const Eigen::Matrix<Scalar, 27, 1> shapeMatrices, bool bTgTsTa):
    b_g(b_ga.template head<3>())/*block<3,1>(0,0))*/, b_a(b_ga.template tail<3>())//block<3,1>(3, 0))
{
    if(bTgTsTa)
    {
        for (int i=0; i<3; ++i){
            for (int j=0; j<3; ++j){
                S_g(i,j)=shapeMatrices(i*3+j);
                T_s(i,j)=shapeMatrices(i*3+j+9);
                S_a(i,j)=shapeMatrices(i*3+j+18);
            }
            S_g(i,i)-=1;
            S_a(i,i)-=1;
        }
    }
    else
    {
        for (int i=0; i<3; ++i){
            for (int j=0; j<3; ++j){
                S_g(i,j)=shapeMatrices(i*3+j);
                T_s(i,j)=shapeMatrices(i*3+j+9);
                S_a(i,j)=shapeMatrices(i*3+j+18);
            }
        }
    }
    invT_a=(S_a+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
    invT_g=(S_g+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
}


template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const Eigen::Matrix<Scalar, 27, 1> vSaSgTs, const Eigen::Matrix<Scalar, 6,1> b_ag)
:b_g(b_ag.template tail<3>()), b_a(b_ag.template head<3>())
{
    for (int i=0; i<3; ++i){
        for (int j=0; j<3; ++j){
            S_a(i,j)=vSaSgTs(i*3+j);
            S_g(i,j)=vSaSgTs(i*3+j+9);
            T_s(i,j)=vSaSgTs(i*3+j+18);
        }
    }

    invT_a=(S_a+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
    invT_g=(S_g+ Eigen::Matrix<Scalar, 3, 3>::Identity()).inverse();
}

template <class Scalar>
IMUErrorModel<Scalar>::IMUErrorModel(const IMUErrorModel<Scalar> & iem):
    b_g(iem.b_g), b_a(iem.b_a), S_g(iem.S_g), T_s(iem.T_s), S_a(iem.S_a), invT_a(iem.invT_a), invT_g(iem.invT_g)
{
}

template <class Scalar>
void IMUErrorModel<Scalar>::resetBgBa(const Eigen::Matrix<Scalar, 6, 1> b_ga)
{
    b_g = b_ga.template head<3>();
    b_a = b_ga.template tail<3>();
}

template <class Scalar>
void IMUErrorModel<Scalar>::estimate(const Eigen::Matrix<Scalar, 3, 1> w_m, const Eigen::Matrix<Scalar, 3, 1> a_m)
{
    a_est= invT_a*(a_m- b_a);
    w_est= invT_g*(w_m- b_g - T_s* a_est);
}
template <class Scalar>
void IMUErrorModel<Scalar>::predict(const Eigen::Matrix<Scalar, 3, 1> w_s, const Eigen::Matrix<Scalar, 3, 1> a_s)
{
    a_obs = S_a*a_s +a_s+ b_a;
    w_obs = S_g*w_s + w_s + T_s*a_s + b_g;
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 9> IMUErrorModel<Scalar>::dmatrix3_dvector9_multiply(const Eigen::Matrix<Scalar, 3, 1> rhs)
{
    Eigen::Matrix<Scalar, 3, 9> m = Eigen::Matrix<Scalar, 3, 9>::Zero();
    m.template topLeftCorner<1,3>()= rhs.transpose();
    m.template block<1,3>(1,3)= rhs.transpose();
    m.template block<1,3>(2,6)= rhs.transpose();
    return m;
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 6> IMUErrorModel<Scalar>::domega_B_dbgba()
{
    Eigen::Matrix<Scalar, 3, 6> dwB_dbgba = Eigen::Matrix<Scalar, 3, 6>::Zero();
    dwB_dbgba.template block<3,3>(0,0) = - invT_g;
    dwB_dbgba.template block<3,3>(0,3) = invT_g*T_s*invT_a;
    return dwB_dbgba;
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 27> IMUErrorModel<Scalar>::domega_B_dSgTsSa()
{
    Eigen::Matrix<Scalar, 3, 27> dwB_dSgTsSa = Eigen::Matrix<Scalar, 3, 27>::Zero();
    dwB_dSgTsSa.template block<3,9>(0,0) = - invT_g*dmatrix3_dvector9_multiply(w_est);
    dwB_dSgTsSa.template block<3,9>(0,9) = - invT_g*dmatrix3_dvector9_multiply(a_est);
    dwB_dSgTsSa.template block<3,9>(0,18) = invT_g*T_s*invT_a*dmatrix3_dvector9_multiply(a_est);
    return dwB_dSgTsSa;
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 6> IMUErrorModel<Scalar>::dacc_B_dbgba()
{
    Eigen::Matrix<Scalar, 3, 6> daB_dbgba = Eigen::Matrix<Scalar, 3, 6>::Zero();
    daB_dbgba.template block<3,3>(0,3) = -invT_a;
    return daB_dbgba;
}

template <class Scalar>
Eigen::Matrix<Scalar, 3, 27> IMUErrorModel<Scalar>::dacc_B_dSgTsSa()
{
    Eigen::Matrix<Scalar, 3, 27> daB_dSgTsSa = Eigen::Matrix<Scalar, 3, 27>::Zero();
    daB_dSgTsSa.template block<3,9>(0,18) = -invT_a*dmatrix3_dvector9_multiply(a_est);
    return daB_dSgTsSa;
}

template <class Scalar>
void IMUErrorModel<Scalar>::dwa_B_dbgbaSTS(Eigen::Matrix<Scalar, 6, 6+27> & output)
{
    output.template block<3, 6>(0,0) = domega_B_dbgba();
    output.template block<3, 6>(3,0) = dacc_B_dbgba();
    output.template block<3, 27>(0,6) = domega_B_dSgTsSa();
    output.template block<3, 27>(3,6) = dacc_B_dSgTsSa();

}
