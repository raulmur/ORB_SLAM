#ifndef IMU_ERROR_MODEL_H_
#define IMU_ERROR_MODEL_H_

#include <Eigen/Dense>
//accelerometer and gyro error model by drawing inspirations from Mingyang Li
// ICRA 2014, Titterton and Weston 12.5.2, and Tedaldi ICRA 2014 A robust and 
// easy to implement method. Here we follow exactly the model used in 
// Mingyang Li ICRA 2014 and Shelley 2014 master thesis
template <class Scalar>
class IMUErrorModel
{
public:
    //a volatile class to estimate linear acceleration and angular rate given measurements, or predict measurements given estimated values
    Eigen::Matrix<Scalar, 3,1> b_g;
    Eigen::Matrix<Scalar, 3, 1> b_a;
    Eigen::Matrix<Scalar, 3, 3> S_a; // S_a + I_3 = T_a
    Eigen::Matrix<Scalar, 3, 3> S_g; // S_g + I_3 = T_g
    Eigen::Matrix<Scalar, 3, 3> T_s; // T_s
    Eigen::Matrix<Scalar, 3, 3> invT_a; //inverse of T_a
    Eigen::Matrix<Scalar, 3, 3> invT_g; //inverse of T_g

    //intermediate variables
    Eigen::Matrix<Scalar, 3, 1> a_est;
    Eigen::Matrix<Scalar, 3, 1> w_est;
    Eigen::Matrix<Scalar, 3, 1> a_obs;
    Eigen::Matrix<Scalar, 3, 1> w_obs;

    IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ga = Eigen::Matrix<Scalar, 6, 1>::Zero());

    IMUErrorModel(const Eigen::Matrix<Scalar, 6,1> b_ga, const Eigen::Matrix<Scalar, 27, 1> shapeMatrices, bool bTgTsTa= true);

    IMUErrorModel(const Eigen::Matrix<Scalar, 27, 1> vSaSgTs, const Eigen::Matrix<Scalar, 6,1> b_ag);

    //copy constructor
    IMUErrorModel(const IMUErrorModel<Scalar> & iem);

    void resetBgBa(const Eigen::Matrix<Scalar, 6, 1> b_ga);
    void estimate(const Eigen::Matrix<Scalar, 3, 1> w_m, const Eigen::Matrix<Scalar, 3, 1> a_m);
    void predict(const Eigen::Matrix<Scalar, 3, 1> w_s, const Eigen::Matrix<Scalar, 3, 1> a_s);

    // the following functions refer to Michael Andrew Shelley master thesis 2014 with some corrections
    // calculate $\frac{\partial{T_{3\times3}}}{\partial \vec{T}_9}\vec{a}_{3}$
    Eigen::Matrix<Scalar, 3, 9> dmatrix3_dvector9_multiply(const Eigen::Matrix<Scalar, 3, 1> rhs);
    // calculate $\frac{\partial\ \omega_{WB}^B}{\partial {(b_g, b_a)}}$
    Eigen::Matrix<Scalar, 3, 6> domega_B_dbgba();
    // calculate $\frac{\partial\ \omega_{WB}^B}{\partial {(\vec{T}_g, \vec{T}_s, \vec{T}_a)}}$ which is also
    //$\frac{\partial\ \omega_{WB}^B}{\partial {(\vec{S}_g, \vec{T}_s, \vec{S}_a)}}$
    // Note call this function after estimate because it requires the latest a_est, and w_est
    Eigen::Matrix<Scalar, 3, 27> domega_B_dSgTsSa();

    // calculate $\frac{\partial\ a^B}{\partial {(b_g, b_a)}}$
    Eigen::Matrix<Scalar, 3, 6> dacc_B_dbgba();
    // calculate $\frac{\partial\ a^B}{\partial {(\vec{T}_g, \vec{T}_s, \vec{T}_a)}}$ which is also
    //$\frac{\partial\ a^B}{\partial {(\vec{S}_g, \vec{T}_s, \vec{S}_a)}}$
    // Note call this function after estimate because it requires the latest a_est, and w_est
    Eigen::Matrix<Scalar, 3, 27> dacc_B_dSgTsSa();

    // calculate $\frac{\partial\ [\omega_{WB}^B, a^B]}{\partial {(b_g, b_a, \vec{T}_g, \vec{T}_s, \vec{T}_a)}}$
    // Note call this function after estimate because it requires the latest a_est, and w_est
    void dwa_B_dbgbaSTS(Eigen::Matrix<Scalar, 6, 27+6> & output);
};

#endif
