#ifndef IMU_ERROR_MODEL_H_
#define IMU_ERROR_MODEL_H_
#include <Eigen/Dense>
//accelerometer and gyro error model by drawing inspirations from Mingyang Li
// ICRA 2014, Titterton and Weston 12.5.2, and Tedaldi ICRA 2014 A robust and 
// easy to implement method. Here we follow exactly the model used in 
// Mingyang Li ICRA 2014 and Shelley 2014 master thesis

class IMUErrorModel
{
public:
    //a volatile class to estimate linear acceleration and angular rate given measurements, or predict measurements given estimated values
    Eigen::Vector3d b_a;
    Eigen::Vector3d b_g;
    Eigen::Matrix3d S_a; // S_a + I_3 = T_a
    Eigen::Matrix3d S_g; // S_g + I_3 = T_g
    Eigen::Matrix3d T_s; // T_s
    Eigen::Matrix3d invT_a; //inverse of T_a
    Eigen::Matrix3d invT_g; //inverse of T_g
    bool bInitialized; // is the model parameters initialized?

    //intermediate variables
    Eigen::Vector3d a_est;
    Eigen::Vector3d w_est;
    Eigen::Vector3d a_obs;
    Eigen::Vector3d w_obs;

    //IMUErrorModel(): bInitialized(false){}
    IMUErrorModel(const Eigen::Matrix<double, 6, 1> b_ag);
    IMUErrorModel(const Eigen::Matrix<double, 33, 1> bundle);

    void estimate(const Eigen::Vector3d a_m, const Eigen::Vector3d w_m);
    void predict(const Eigen::Vector3d a_s, const Eigen::Vector3d w_s);
};





#endif
