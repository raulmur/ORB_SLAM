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
    Eigen::Matrix<Scalar, 3,1> b_a;
    Eigen::Matrix<Scalar, 3, 1> b_g;
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

    //IMUErrorModel(): bInitialized(false){}
    IMUErrorModel(const Eigen::Matrix<Scalar, 6, 1> b_ag);
    //IMUErrorModel(const Eigen::Matrix<Scalar, 33, 1> bundle);
    IMUErrorModel(const Eigen::Matrix<Scalar, 6,1> b_ag, const Eigen::Matrix<Scalar, 27, 1> shapeMatrices);
    void estimate(const Eigen::Matrix<Scalar, 3, 1> a_m, const Eigen::Matrix<Scalar, 3, 1> w_m);
    void predict(const Eigen::Matrix<Scalar, 3, 1> a_s, const Eigen::Matrix<Scalar, 3, 1> w_s);
};

#endif
