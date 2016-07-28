#ifndef SLIDINGWINDOW_H
#define SLIDINGWINDOW_H
#include "IMU_constraint.h"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <vector>
#include <deque>
#include <string>
#include <fstream>


/*Frames: the world frame can be any well-defined Earth fixed frame, e.g., a navigation (NED) frame affixed to the GPS antenna
 at the start of the field test, called n0 frame in this program. This frame has a known perfect transformation to the ECEF frame.
the reference sensor frame is simply the IMU frame,
but we still have to set the transform from the reference sensor to the IMU
Discrete states: states are associated with each epoch of GPS, [transform from the reference sensor(called 'c') to the world frame,
velocity of IMU in the world frame, accelerometer biases, gyro biases, position of antenna in the reference sensor frame], i.e., $[T_c^w\vert v_s^w, b_a, b_g]$;
For convenience of implementation, states at an epoch are split into two groups, $T_c^w$ and $[v_s^w, b_a, b_g]$,
represented by G2oVertexSE3 and G2oVertexSpeedBias
For comparison, Furgale formulated continuous IMU states representation in "unified spatial-temporal IMU calibration..."
Cost function: it includes those incurred from GPS observations, IMU constraints, and state prior*/
class SlidingWindow
{//sliding window filter to integrate GPS/IMU data
public:

    SlidingWindow(const std::string settingFile);
    ~SlidingWindow();
    //input: T_{s(k+1)}^w,  t_{k+1}, IMUMeas [t_{p(k)-1}, t_{p(k+1)-1}] where t_{p(k)-1}<= t_k and t_{p(k)}> t_k
    void AddVertex(const Sophus::SE3d predTskp12w, const double timestamp,
                   const Eigen::Matrix<double,9,1> predSBkp1,
                   const Eigen::Vector3d vGPSMeas, const Eigen::Matrix3d covGPSMeas,
                   const std::vector<Eigen::Matrix<double, 7,1> > vIMUMeas = std::vector<Eigen::Matrix<double, 7,1> >());
    // if more than nWindowSize vertices, remove the first one, set FEJ for its related variables
    void RemoveVertex(std::ofstream & out_stream_optimized);
    // construct optimizer, and restore optimized values
    void Optimize(int, char bOptimizeSagsAndArm= 0x00); //0x01 optimize lever arm, 0x10 optimize Sags, 0x11 optimize both
    // initialize S_a, S_g with its diagonal elements
    void SetInitialShapeMatrices(const Eigen::Matrix<double, 6,1>);
    // optimize all the parameters and states in the whole session
    void GlobalOptimize(int, const Eigen::Matrix<double,6,1> = Eigen::Matrix<double, 6,1>::Zero());
    // print out the left state variables
    void DumpVertices(std::ofstream & out_stream_optimized);
    ScaViSLAM::G2oIMUParameters GetIMUParam(){return imu_;}
    void GetInitStates(Sophus::SE3d &T_s1_to_w,Eigen::Matrix<double, 9,1>& speed_bias_1)
    {
        T_s1_to_w= priorTc2w*imu_.T_imu_from_cam.inverse();
        speed_bias_1 = priorSB;
    }
    // call this after optimize()
    void GetOptimizedStates(Sophus::SE3d &T_s1_to_w,Eigen::Matrix<double, 9,1>& speed_bias_1)
    {
        T_s1_to_w= (imu_.T_imu_from_cam*vTw2cj.back()).inverse();
        speed_bias_1 = vSpeedBias.back();
    }
private:
    std::deque<double> vTimes; // N+1 timestamps for vertices
    ///states
    std::deque<Sophus::SE3d> vTw2cj; // N+1 vertices from k to k+N
    std::deque<Eigen::Matrix<double,9,1> > vSpeedBias; //n+1 vertices for speed and biases
    Eigen::Matrix<double, 27,1> Sags_; // T_a= S_a +I; T_g= S_g+ I; T_s
    std::deque<Eigen::Matrix<double, 3,1> > vla_; //lever arm of the GPS antenna in the reference sensor frame(so-called camera frame)
    /// intermediate state pointers in optimization
    std::vector<ScaViSLAM::G2oVertexSE3* > vpSE3;
    std::vector<ScaViSLAM::G2oVertexSpeedBias* > vpSB;
    ScaViSLAM::G2oVertexShapeMatrices* pSags;
    std::vector<ScaViSLAM::G2oVertexLeverArm*> vpla;

    ///states covariances

    ///actual measurements
    std::deque<Eigen::Vector3d> vGPSMeas; // for each vertex there is a GPS measurement, position of antenna in the ECEF frame
    std::deque<std::vector<Eigen::Matrix<double, 7,1> > > vIMUMeas; //N vectors of measurements between (k, k+1),..., (k+N-1, k+N)
    // for the a measurement vector corresponds to [t_j, t_{j+1}], timestamps in measurements satisfy that
    // vMeas[j][0][0]<= t_j, vMeas[j][1][0]> t_j, vMeas[j][end][0]< t_{j+1}
    //given GPS observations indexed k and its timestamp t(k), assume we have IMU readings indexed p(k) s.t. t(p(k)-1)<=t(k)<t(p(k))
    /// measurement covariances
    std::deque<Eigen::Matrix3d> vInvCovGPSMeas; // inverse covariance for each gps measurement in ECEF frame

    /// prior estimates
    Sophus::SE3d priorTc2w;
    Eigen::Matrix<double, 9, 1> priorSB;
    Eigen::Matrix<double, 27, 1> priorSags;
    Eigen::Matrix<double, 3, 1> priorla; //prior estimated lever arm
    /// prior covariances
    Eigen::Matrix<double, 6, 6> invCovPriorPose;// inverse of covariance of prior pose, in order of [\upsilon, \omega] = \hat{priorT}_c^w *(priorT_c^w)^{-1}
    Eigen::Matrix<double, 9, 9> invCovPriorSB; //inverse of covariance of prior speed bias
    Eigen::Matrix<double, 27, 27> invCovSags;
    Eigen::Matrix<double, 3, 3> invCovla;

    /// parameters
    ScaViSLAM::G2oIMUParameters imu_;
    Sophus::SE3d constTw2e; // the transformation from the local NED frame to the ECEF frame
    Sophus::SE3d constTe2w;
    size_t nWindowSize; //maximum number of SE3 vertices in the sliding window
    static const double qla_; // assume lever arm in each axis follow random walk, \dot{l}= n_l ~ N(0, q), if in 0.2 s, the std dev of l becomes 0.5 mm, then q= (5e-4)^2/0.2
    static const int MAGIC2 =3; //we use MAGIC2*i to identify pose vertices and MAGIC2*i+1 for speeb bias vertices in g2o optimizer

};

#endif // SLIDINGWINDOW_H
