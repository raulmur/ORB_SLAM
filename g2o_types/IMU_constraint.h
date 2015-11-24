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

#ifndef SCAVISLAM_G2O_IMU_CONSTRAINT_H
#define SCAVISLAM_G2O_IMU_CONSTRAINT_H

#include <g2o/core/base_multi_edge.h>
#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif

#include "anchored_points.h"
#include "eigen_utils.h" //for rvec2quat
#include "timegrabber.h" //imugrabber
#include <iostream>
using namespace std;
namespace ScaViSLAM{

class G2oIMUParameters : public g2o::Parameter
{
public:
    G2oIMUParameters()
    {
        q_n_aw_babw.setZero();
        gwomegaw.setZero();
        gwomegaw[3]=9.81;
    }

    G2oIMUParameters        (const Eigen::Matrix<double, 3,1> & q_noise_acc,const Eigen::Matrix<double, 3,1> &q_noise_gyr,
                             const Eigen::Matrix<double, 3,1> & q_noise_accbias,const Eigen::Matrix<double, 3,1> &q_noise_gyrbias,
                             const Sophus::SE3Group<double> & T_c_to_s, const Eigen::Matrix<double, 6,1> &gomegaw)
        : T_imu_from_cam(T_c_to_s), gwomegaw(gomegaw)
    {
        q_n_aw_babw.head(3)=q_noise_acc;
        q_n_aw_babw.segment(3,3)=q_noise_gyr;
        q_n_aw_babw.segment(6,3)=q_noise_accbias;
        q_n_aw_babw.tail(3)=q_noise_gyrbias;
    }
    G2oIMUParameters        (const G2oIMUParameters& other)
        : q_n_aw_babw(other.q_n_aw_babw), T_imu_from_cam(other.T_imu_from_cam),
          gwomegaw(other.gwomegaw)
    {
    }
    G2oIMUParameters & operator= (const G2oIMUParameters& other)
    {
        if(this==&other)
            return *this;
        q_n_aw_babw=other.q_n_aw_babw;
        T_imu_from_cam=other.T_imu_from_cam;
        gwomegaw=other.gwomegaw;

        return *this;
    }

    virtual bool
    read                       (std::istream& is)
    {
        for(int i=0; i<12;++i)
            is >> q_n_aw_babw[i];
        return true;
    }

    virtual bool
    write                      (std::ostream& os) const
    {
        for(int i=0; i<12;++i)
            os << q_n_aw_babw[i]<<" ";
        return true;
    }
    Eigen::Matrix<double,12,1> q_n_aw_babw; //squared noise density of VRW, ARW, noise of random walk of accelerometer bias and gyro bias
    Sophus::SE3Group<double> T_imu_from_cam; // the transformation from camera to IMU sensor frame
    Eigen::Matrix<double, 6,1> gwomegaw;// gravity in local world frame in m/s^2, and earth rotation rate in world frame in rad/sec

};
// time varying lever arm vertex can represent the antenna position in the reference sensor frame
class G2oVertexLeverArm : public g2o::BaseVertex<3, Eigen::Vector3d >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexLeverArm               (){}

    virtual bool
    read                       (std::istream& is)
    {
        return false;
    }

    virtual bool
    write                      (std::ostream& os) const
    {return false;}

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Vector3d > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
};


class G2oVertexSpeedBias : public g2o::BaseVertex<9, Eigen::Matrix<double,9,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexSpeedBias               ():first_estimate(NULL){}
    ~G2oVertexSpeedBias               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 9,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,9,1>& fe){
        first_estimate=new Eigen::Matrix<double,9,1>(fe);
    }
    Eigen::Matrix<double,9,1>* first_estimate;
};
// extended speed bias vertex, including speed of the reference sensor in world frame, accelerometer bias, gyro bias,
// elements of S_a, S_g, T_s in row major order following IMUErrorModel's definition
class G2oVertexSpeedBiasEx : public g2o::BaseVertex<36, Eigen::Matrix<double,36,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexSpeedBiasEx               ():first_estimate(NULL){}
    ~G2oVertexSpeedBiasEx               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 36,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,36,1>& fe){
        first_estimate=new Eigen::Matrix<double,36,1>(fe);
    }
    Eigen::Matrix<double,36,1>* first_estimate;
};
//s0 can be any local world frame
// acc, m/s^2, estimated acc from imu in s frame with bias removed, gyro, rad/s, estimated angular rate by imu with bias removed
template<typename Scalar>
void strapdown_local_quat_bias(const Eigen::Matrix<Scalar,3,1>& rs0, const Eigen::Matrix<Scalar,3,1> & vs0, const Eigen::Quaternion<Scalar>& qs0_2_s,
                               const Eigen::Matrix<Scalar,3,1> & a, const Eigen::Matrix<Scalar,3,1>& w, Scalar dt,
                               const Eigen::Matrix<Scalar, 6,1>& gomegas0,
                               Eigen::Matrix<Scalar,3,1>* rs0_new, Eigen::Matrix<Scalar,3,1>* vs0_new, Eigen::Quaternion<Scalar>* qs0_2_s_new)
{
    Eigen::Matrix<Scalar,3,1> wie2s0=gomegas0.tail(3);
    //Update attitude
    // method (1) second order integration
    Eigen::Quaternion<Scalar> qe=rvec2quat(wie2s0*dt);
    (*qs0_2_s_new)=qs0_2_s*qe;
    Eigen::Quaternion<Scalar> qb=rvec2quat(-w*dt);
    (*qs0_2_s_new)=qb*(*qs0_2_s_new);

    //// method (2) Runge-Kutta 4th order integration, empirically, this sometimes
    // gives worse result than second order integration
    // wie2s=quatrot_v000(rvqs0(7:10),wie2s0,0);
    // omega=zeros(4,2);
    // omega(1,2)=dt;
    // omega(2:4,1)=w-wie2s;
    // omega(2:4,2)=lastw-wie2s;
    // qs2s0=rotationRK4( omega, [rvqs0(7); -rvqs0(8:10)]);
    // rvqs0_new(7:10)=[qs2s0(1); -qs2s0(2:4)];

    //// better velocity and position integration than first order rectanglar rule
    //Update Velocity
    //    Vector3d vel_inc1=(quatrot(qs0_2_s,a*dt,1)+quatrot(*qs0_2_s_new,a*dt,1))/2;
    Eigen::Matrix<Scalar,3,1> vel_inc1=(qs0_2_s.conjugate()._transformVector(a*dt)+(*qs0_2_s_new).conjugate()._transformVector(a*dt))*Scalar(0.5);

    Eigen::Matrix<Scalar,3,1> vel_inc2=(gomegas0.head(3)-Scalar(2)*wie2s0.cross(vs0))*dt;

    (*vs0_new)=vs0+vel_inc1+vel_inc2;
    //Update_pos
    (*rs0_new)=rs0+((*vs0_new)+vs0)*dt*Scalar(0.5);
}
// system process model in local world frame denoted by s0 ( which has no other meanings) with dcm formulation
// rs in s0, vs in s0, qs0 2s, covariance P corresponds to error states,
// \delta rs in w, \delta v s in w, \psi w, ba, bg. where \tilde{R}_s^w=(I-[\psi^w]_\times)R_s^w and \delta v_s^w= \tilde{v}_s^w-v_s^w
// acc bias is assumed random walk, gyro bias also random walk
// acc, m/s^2, estimated acc from imu in s frame with bias removed, gyro, rad/s, estimated angular rate by imu with bias removed
// w.r.t i frame coordinated in s frame, dt, time interval for covariance update
// q_n_a: accelerometer VRW noise density squared, q_n_ba, accelerometer bias noise density squared
// input P stores previous covariance, in output P sotres predicted covariance
template<typename Scalar>
void sys_local_dcm_bias(const Eigen::Matrix<Scalar, 3,1> & rs0, const Eigen::Matrix<Scalar, 3,1> & vs0,const Eigen::Quaternion<Scalar>& qs0_2_s,
                        const Eigen::Matrix<Scalar, 3,1> & acc, const Eigen::Matrix<Scalar, 3,1> & gyro, Scalar dt,
                        const Eigen::Matrix<Scalar, 3,1> & q_n_a,const Eigen::Matrix<Scalar, 3,1> & q_n_w, const Eigen::Matrix<Scalar, 3,1> & q_n_ba,
                        const Eigen::Matrix<Scalar, 3,1>& q_n_bw, Eigen::Matrix<Scalar, 15, 15>* P)
{
    //system disturbance coefs
    const int navStates=9;
    Eigen::Matrix<Scalar, navStates, 6> Nnav=Eigen::Matrix<Scalar, navStates, 6>::Zero();
    Eigen::Matrix<Scalar, 3,3> Cs02s=qs0_2_s.toRotationMatrix();

    Nnav.block(3,0,3,3)=Cs02s.transpose(); //velocity
    Nnav.block(6,3,3,3)=-Cs02s.transpose(); //attitude
    //system matrix
    Eigen::Matrix<Scalar, navStates, navStates> Anav=Eigen::Matrix<Scalar, navStates, navStates>::Zero();
    Anav.block(0,3,3,3).setIdentity(); // rs in s0
    //Velocity  // for low grade IMU, w_ie^e is buried
    Anav.block(3,6,3,3)=skew(Cs02s.transpose()*acc);
    //Imu error model parameters
    Eigen::Matrix<Scalar, 6,6> Rimu=Eigen::Matrix<Scalar, 6,6>::Zero();

    Rimu.topLeftCorner(3,3)=q_n_a.asDiagonal();
    Rimu.bottomRightCorner(3,3)=q_n_w.asDiagonal();
    Eigen::Matrix<Scalar,6,6> Qimu_d=Eigen::Matrix<Scalar, 6,6>::Zero();
    Qimu_d.topLeftCorner(3,3)=q_n_ba.asDiagonal()*dt;
    Qimu_d.bottomRightCorner(3,3)=q_n_bw.asDiagonal()*dt;

    //Combine and discretize nav and imu models
    // this discretization can also be accomplished by Loan's matrix exponential
    // method, see sys_metric_phipsi_v000.m

    Eigen::Matrix<Scalar, navStates,navStates> Anav_d=Eigen::Matrix<Scalar, navStates,navStates>::Identity()+dt*Anav;  //Use 1st order taylor series to discretize Anav
    Eigen::Matrix<Scalar, navStates, navStates> Qnav=Nnav*Rimu*Nnav.transpose();
    Eigen::Matrix<Scalar, navStates, navStates> Qnav_d=dt*Scalar(0.5)*(Anav_d*Qnav+Qnav*Anav_d.transpose());      //Use trapezoidal rule to discretize Rimu

    Eigen::Matrix<Scalar, 15,15> STM=Eigen::Matrix<Scalar, 15,15>::Zero();

    STM.topLeftCorner(navStates,navStates)=Anav_d;
    STM.block(0,navStates,navStates,6)=Nnav*dt;
    STM.block(navStates, navStates,6,6).setIdentity();

    Eigen::Matrix<Scalar, 15,15> Qd=Eigen::Matrix<Scalar, 15,15>::Zero();

    Qd.topLeftCorner(navStates,navStates)=Qnav_d;
    Qd.block(navStates, navStates,6,6)=Qimu_d;
    Qd.block(0,navStates,navStates,6)=Nnav*Qimu_d*dt*Scalar(0.5);
    Qd.block(navStates,0,6,navStates)=Qd.block(0,navStates,navStates,6).transpose();

    (*P)=STM*(*P)*STM.transpose()+Qd;// covariance of the navigation states and imu error terms
}

//given pose/velocity of IMU sensor, i.e., T_sensor_to_world, v_sensor_in_world,
// and IMU biases at epoch t(k), i.e., time_pair[0], measurements from t(p^k-1) to t(p^{k+1}-1),
// where t(p^k-1) is the closest epoch to t(k) less or equal to t(k), and gravity in world frame in m/s^2
// which can be roughly computed using some EGM model or assume constant, e.g., 9.81 m/s^2
// and earth rotation rate in world frame in rad/sec which can often be set to 0
// predict states in terms of IMU sensor frame at epoch t(k+1), i.e., time_pair[1]
// optionally, propagate covariance in the local world frame, the covariance corresponds to states,
// \delta rs in w, \delta v s in w, \psi w, ba, bg. where \tilde{R}_s^w=(I-[\psi^w]_\times)R_s^w
// covariance of states can be treated more rigorously as in ethz asl sensor_fusion on github by Stephan Weiss
// P stores covariance at t(k), update it to t(k+1)
template<typename Scalar>
void predictStates(const Sophus::SE3Group<Scalar> &T_sk_to_w, const Eigen::Matrix<Scalar, 9,1>& speed_bias_k, const Scalar * time_pair,
                   const std::vector<Eigen::Matrix<Scalar, 7,1> >& measurements, const Eigen::Matrix<Scalar, 6,1> & gwomegaw,
                   const Eigen::Matrix<Scalar, 12, 1>& q_n_aw_babw,
                   Sophus::SE3Group<Scalar>* pred_T_skp1_to_w, Eigen::Matrix<Scalar, 3,1>* pred_speed_kp1, Eigen::Matrix<Scalar, 15,15> *P)
{
    bool predict_cov=(P!=NULL);
    int every_n_reading=2;// update covariance every n IMU readings,
    // the eventual covariance has little to do with this param as long as it remains small
    Eigen::Matrix<Scalar, 3,1> r_new, r_old(T_sk_to_w.translation()), v_new, v_old(speed_bias_k.head(3));
    Eigen::Quaternion<Scalar> q_new, q_old(T_sk_to_w.unit_quaternion().conjugate());
    Scalar dt=measurements[1][0]-time_pair[0];
    Scalar covupt_time(time_pair[0]);//the time to which the covariance is updated. N.B. the initial covariance is updated to $t_k$
    assert(dt>Scalar(0)&&dt<=Scalar(0.01+1e-8));
    Eigen::Matrix<Scalar, 6, 1> est_measurement=measurements[0].tail(6)-speed_bias_k.tail(6);
    Eigen::Matrix<Scalar, 3,1> est_accel=est_measurement.head(3);
    Eigen::Matrix<Scalar, 3,1> est_gyro=est_measurement.tail(3);
    const Eigen::Matrix<Scalar, 3,1> qna=q_n_aw_babw.head(3), qnw=q_n_aw_babw.segment(3,3),
            qnba=q_n_aw_babw.segment(6,3),qnbw=q_n_aw_babw.tail(3);
    strapdown_local_quat_bias( r_old, v_old, q_old, est_accel, est_gyro,
                               dt, gwomegaw, &r_new, &v_new, &q_new);

    if(predict_cov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old, est_accel,est_gyro,
                           measurements[1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
        //for more precise covariance update, we can use average estimated accel and angular rate over n(every_n_reading) IMU readings
        covupt_time=measurements[1][0];
    }
    r_old=r_new;
    v_old=v_new;
    q_old=q_new;
    int unsigned i=1;
    for (; i<measurements.size()-1;++i){
        dt=measurements[i+1][0]-measurements[i][0];
        est_measurement=measurements[i].tail(6)-speed_bias_k.tail(6);
        est_accel=est_measurement.head(3);
        est_gyro=est_measurement.tail(3);
        strapdown_local_quat_bias( r_old, v_old, q_old, est_accel,
                                   est_gyro, dt, gwomegaw, &r_new, &v_new, &q_new);
        if(predict_cov&&(i%every_n_reading==0))
        {
            sys_local_dcm_bias(r_old, v_old, q_old,est_accel,
                               est_gyro, measurements[i+1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
            covupt_time=measurements[i+1][0];
        }
        r_old=r_new;
        v_old=v_new;
        q_old=q_new;
    }
    //assert(i==measurements.size()-1);
    dt=time_pair[1]-measurements[i][0];//the last measurement
    assert(dt>=Scalar(0)&& dt<Scalar(0.01));
    est_measurement=measurements[i].tail(6)-speed_bias_k.tail(6);
    est_accel=est_measurement.head(3);
    est_gyro=est_measurement.tail(3);
    strapdown_local_quat_bias( r_old, v_old, q_old, est_accel, est_gyro,
                               dt, gwomegaw, &r_new, &v_new, &q_new);
    if(predict_cov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old, est_accel, est_gyro,
                           time_pair[1]-covupt_time,qna, qnw, qnba, qnbw,P);
        covupt_time=time_pair[1];
    }
    pred_T_skp1_to_w->setQuaternion(q_new.conjugate());
    pred_T_skp1_to_w->translation()=r_new;
    (*pred_speed_kp1)=v_new;
}


class G2oEdgeIMUConstraint : public  g2o::BaseMultiEdge<15, std::vector<Eigen::Matrix<double, 7,1> > >
{
    //IMU measurements are stored in a std::vector<Matrix<double, 7,1> > structure, each entry: timestamp in seconds,
    //acceleration measurements in m/s^2, gyro measurements in radian/sec
    //the measurements goes from $t(p^k-1)$ which is the closest epoch less than or equal to $t(k)$, up to
    // $t(p^(k+1)-1)$ which is the closest epoch less than or equal to $t(k+1)$.
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeIMUConstraint()
    {
        g2o_IMU=0;
        resizeParameters(1);
        installParameter(g2o_IMU, 0);
    }

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;

    void
    computeError               ();
    template<typename T>
    bool operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1, const T* pXsbkp1, T* error) const;

    void linearizeOplus        ();
    void calcAndSetInformation(const G2oIMUParameters &);
    void SetFrameEpoch(const double timestamp, const int index){
        time_frames[index]=timestamp;
    }
    G2oIMUParameters * g2o_IMU;
    double time_frames[2]; //timestamps for the two frames connected by this multi-edge, $t(k)$ and $t(k+1)$
    //first element corresponds to first vertex which has a smaller timestamp, $t(k)$
    //_measurement[0][0],i.e., $t(p^k-1)$,is less than or equal to $t(k)$, but _measurement[1][0] must be greater than $t(k)$

};
//gravity in world frame=$-R_s^w(\tilde{a}^s-b_a)+n_g$, $\tilde{a}^s$ is pseudo measurement, $b_a$ comes from G2oVertexSpeedBias,
// $R_s^w$ inertial sensor to world rotation comes from G2oVertexSE3, i.e., transform from world to camera frame
class IMURollPitchEdge : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, G2oVertexSE3, G2oVertexSpeedBias>
{
public:
    IMURollPitchEdge()
    {
        g2o_IMU=0;
        resizeParameters(1);
        installParameter(g2o_IMU, 0);
    }
    virtual void computeError();
    virtual void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
    G2oIMUParameters * g2o_IMU;
};

// The idealised GPS measurement of camera position in the world frame; this is 3D and linear, adapted from g2o/examples/targetTypes3D.hpp
// here I used G2oVertexSE3 to represent transform from world frame to the camera frame
class GPSObservationPosition3DEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, G2oVertexSE3>
{
public:
    GPSObservationPosition3DEdge()
    {
    }

    void computeError()
    {
        const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
        _error = v->estimate().inverse().translation() - _measurement;
    }
    void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
};
// The GPS measurement of antenna position in the world frame; this is 3D and linear, adapted from g2o/examples/targetTypes3D.hpp
// here I used G2oVertexSE3 to represent transform from world frame to the reference sensor frame,
// G2oVertexLeverArm to represent antenna position in the reference sensor frame
// often the world frame is defined as a frame anchored to the earth which is convenient for navigation,
// e.g., the N frame anchored at the nominal ECEF point close to the position of the sensor assembly at the start of experiment
class G2oEdgeGPSObservation : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, G2oVertexSE3, G2oVertexLeverArm>
{
public:
    G2oEdgeGPSObservation()
    {
    }

    void computeError()
    {
        const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
        const G2oVertexLeverArm* vl = static_cast<const G2oVertexLeverArm*>(_vertices[1]);
        _error = v->estimate()*_measurement-vl->estimate();
    }
    void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
};

// The lever arm should not change much between two vertices separated by a short interval
// this act as a constraint
// G2oVertexLeverArm to represent antenna position in the reference sensor frame
class G2oEdgeLeverArm: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, G2oVertexLeverArm, G2oVertexLeverArm>
{
public:
    G2oEdgeLeverArm()
    {
    }

    void computeError()
    {
        const G2oVertexLeverArm* v = static_cast<const G2oVertexLeverArm*>(_vertices[0]);
        const G2oVertexLeverArm* vl = static_cast<const G2oVertexLeverArm*>(_vertices[1]);
        _error = v->estimate()-vl->estimate();
    }
    void linearizeOplus()
    {
        _jacobianOplusXi= Eigen::Matrix3d::Identity();
        _jacobianOplusXj= -Eigen::Matrix3d::Identity();
    }

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
};


//prior of pose, measurement is transform from camera to world frame, vertex states are transform from world to camera frame
class G2oSE3Observation6DEdge : public g2o::BaseUnaryEdge<6, Sophus::SE3d, G2oVertexSE3>
{
public:
    G2oSE3Observation6DEdge()
    {
    }

    void computeError()
    {
        const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
        _error= Sophus::SE3d::log(_measurement*v->estimate());
    }
    void linearizeOplus()
    {
        _jacobianOplusXi.topLeftCorner(6,6)= third(_measurement,  _error);
    }

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
};
// prior of speed in world frame, accelerometer biases and gyro biases
class G2oSpeedBiasObs9DEdge : public g2o::BaseUnaryEdge<9, Eigen::Matrix<double ,9,1>, G2oVertexSpeedBias>
{
public:
    G2oSpeedBiasObs9DEdge()
    {
    }

    void computeError()
    {
        const G2oVertexSpeedBias* v = static_cast<const G2oVertexSpeedBias*>(_vertices[0]);
        _error =_measurement- v->estimate();
    }
    void linearizeOplus()
    {
        _jacobianOplusXi= -Eigen::Matrix<double, 9,9>::Identity();
    }

    virtual bool read(std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        return false;
    }
};
class IMUProcessor
{
public:   
    IMUProcessor(const string imu_file, const double sample_interval,const G2oIMUParameters &imu):
        speed_bias_1(Eigen::Matrix<double, 9,1>::Zero()), pred_speed_bias_2(Eigen::Matrix<double, 9,1>::Zero()),
        ft(PlainText), ig(imu_file, ft, sample_interval), imu_(imu), bStatesInitialized(false)
    {
        time_pair[0]=-1;
        time_pair[1]=-1;
    }
    // output the transformation from previous to current camera frame
    Sophus::SE3d propagate(const double time_frame)
    {
        bool is_meas_good=ig.getObservation(time_frame);
        assert(is_meas_good);
        time_pair[0]=time_pair[1];
        time_pair[1]=time_frame;

        Eigen::Vector3d tempVs0inw;
        Eigen::Matrix<double, 15,15>* holder=NULL;
        ScaViSLAM::predictStates(T_s1_to_w, speed_bias_1, time_pair,
                                 ig.measurement, imu_.gwomegaw, imu_.q_n_aw_babw,
                                 &pred_T_s2_to_w, &tempVs0inw, holder);
        pred_speed_bias_2.head(3)=tempVs0inw;
        pred_speed_bias_2.tail(6)=speed_bias_1.tail(6);     //biases do not change in propagation
        Sophus::SE3d pred_Tr_delta=pred_T_s2_to_w*imu_.T_imu_from_cam;
#ifdef ONLY_USE_IMU     //for IMU debug
        out_stream<<i<<" "<< pred_Tr_delta.matrix().row(0)<<" "<<pred_Tr_delta.matrix().row(1)<<" ";
        out_stream<<pred_Tr_delta.matrix().row(2)<<" "<<pred_speed_bias_2.transpose()<<endl;
#endif
        pred_Tr_delta=pred_Tr_delta.inverse()*(T_s1_to_w*imu_.T_imu_from_cam);
        //                        pred_Tr_delta.translation().setZero();  // remove the translation part
        T_s1_to_w=pred_T_s2_to_w;
        speed_bias_1=pred_speed_bias_2;
        return pred_Tr_delta;
    }
    void initStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1, const double timestamp)
    {
        assert(!bStatesInitialized);
        bStatesInitialized=true;
        bool is_meas_good=ig.getObservation(timestamp);
        assert(!is_meas_good);
        time_pair[0]=time_pair[1];
        time_pair[1]=timestamp;
        resetStates(Ts1tow, sb1);
    }
    void resetStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1)
    {
        T_s1_to_w=Ts1tow;
        speed_bias_1=sb1;
    }
    //IMU readings from t(p(k)-1) to t(p(k+1)-1)
    const std::vector<Eigen::Matrix<double, 7,1> > & getMeasurements() const
    {
        return ig.measurement;
    }
    Sophus::SE3d T_s1_to_w;               //IMU pose at t(k) which timestamps image indexed k
    Sophus::SE3d pred_T_s2_to_w;          //predicted IMU pose at t(k+1) of image indexed k+1
    Eigen::Matrix<double, 9,1> speed_bias_1; //IMU speed and acc bias and gyro biases at t(k)
    Eigen::Matrix<double, 9,1> pred_speed_bias_2; //predicted IMU speed in world frame at t(k+1) and biases
    double time_pair[2];              // timestamps of the last and current images, k and k+1

    IMUFileType ft;
    IMUGrabber ig;
    G2oIMUParameters imu_;
    bool bStatesInitialized;
};

//extended IMU constraints with vertices of G2oVertexSpeedBiasEx
//observations are the difference between predicted states at k+1 and the states at k+1
// states at k or k+1 include two components: one is a G2oVertexSE3 that represents
// the SE3 transform from the world frame to the reference sensor frame, the other is an G2oVertexSpeedBiasEx
class G2oEdgeIMUConstraintEx : public  g2o::BaseMultiEdge<42, std::vector<Eigen::Matrix<double, 7,1> > >
{
    //IMU measurements are stored in a std::vector<Matrix<double, 7,1> > structure, each entry: timestamp in seconds,
    //acceleration measurements in m/s^2, gyro measurements in radian/sec
    //the measurements goes from $t(p^k-1)$ which is the closest epoch less than or equal to $t(k)$, up to
    // $t(p^(k+1)-1)$ which is the closest epoch less than or equal to $t(k+1)$.
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeIMUConstraintEx()
    {
        g2o_IMU=0;
        resizeParameters(1);
        installParameter(g2o_IMU, 0);
    }

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;

    void
    computeError               ();
    template<typename T>
    bool operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1, const T* pXsbkp1, T* error) const;

    void linearizeOplus        ();
    void calcAndSetInformation(const G2oIMUParameters &);
    void SetFrameEpoch(const double timestamp, const int index){
        time_frames[index]=timestamp;
    }
    G2oIMUParameters * g2o_IMU;
    double time_frames[2]; //timestamps for the two frames connected by this multi-edge, $t(k)$ and $t(k+1)$
    //first element corresponds to first vertex which has a smaller timestamp, $t(k)$
    //_measurement[0][0],i.e., $t(p^k-1)$,is less than or equal to $t(k)$, but _measurement[1][0] must be greater than $t(k)$

};
// camera related parameters in visual inertial navigation, following Li et al ICRA 2014 eq (10)
// 14 dimensions are position of the reference sensor frame in the camera frame, c_x, c_y, f_x, f_y,
// k_1, k_2, k_3, t_1, t_2, t_r, t_d
class G2oVertexCamParams : public g2o::BaseVertex< 14, Eigen::Matrix<double,14,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexCamParams               ():first_estimate(NULL){}
    ~G2oVertexCamParams               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 14,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,14,1>& fe){
        first_estimate=new Eigen::Matrix<double,14,1>(fe);
    }
    Eigen::Matrix<double,14,1>* first_estimate;
};
// 10 dimensional rolling shutter camera vertex, f_x, f_y, c_x, c_y, k_1, k_2, k_3, t_1, t_2, t_r
// focal length in pixel units, principal points in pixel units, radial distortion,
// tangential distortion, read out time for each frame
class G2oVertexRSCamera : public g2o::BaseVertex<10, Eigen::Matrix<double,10,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexRSCamera               ():first_estimate(NULL){}
    ~G2oVertexRSCamera               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 10,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,10,1>& fe){
        first_estimate=new Eigen::Matrix<double,10,1>(fe);
    }
    Eigen::Matrix<double,10,1>* first_estimate;
};
// extended rolling shutter camera vertex, besides the rolling shutter 10 dimensional parameters,
// also appends $p_s^c, t_d$ to its end which makes a 14 dimensional vector
// $p_s^c$ is the reference sensor in the camera frame, t_d is the offset
// between the reference sensor time and the camera timestamp, i.e., time of image in reference sensor time = time of image + t_d
// according to Li ICRA 2013 and 2014: if  the IMU  has  a  longer  latency  than  the  camera,  then t_d will
// be  positive,  while  in  the  opposite  case t_d will  be  negative.
class G2oVertexRSCameraEx : public g2o::BaseVertex<36, Eigen::Matrix<double,14,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexRSCameraEx               ():first_estimate(NULL){}
    ~G2oVertexRSCameraEx               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 14,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,14,1>& fe){
        first_estimate=new Eigen::Matrix<double,14,1>(fe);
    }
    Eigen::Matrix<double,14,1>* first_estimate;
};
// global shutter camera vertex following Li ICRA 2013, 3-D Motion Estimation and Online Temporal Calibration
// $T_s^c, t_d$, transform from reference sensor frame to the camera frame, t_d is the time delay
// between the reference sensor time and the camera timestamp, i.e., time of image in reference sensor time = time of image + t_d
// according to Li ICRA 2013
// the arrangement of data is qxyzw, txyz, t_d, and the 7 dimensional minimal representation is [\upsilon, \omega, \delta t_d]
class G2oVertexGSCamera : public g2o::BaseVertex<7, Eigen::Matrix<double,8,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexGSCamera               ():first_estimate(NULL){}
    ~G2oVertexGSCamera               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        double updatetd(update_p[6]);
        _estimate[7]=updatetd+_estimate[7];

        Eigen::Map<const Vector6d> update(update_p);
        double temp_estimate[7]= {0};
        for(int jack=0; jack<7 ;++jack)
            temp_estimate[jack]= _estimate[jack];
        const Eigen::Map<const Sophus::SE3d > se3Ts2c(temp_estimate); //qxyzw txyz
        Sophus::SE3d se3Ts2c_updated= Sophus::SE3d::exp(update)*se3Ts2c;
        double* dataPtr= se3Ts2c_updated.data();
        for(int jack=0; jack<7 ;++jack)
            _estimate[jack]= dataPtr[jack];
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
        _estimate[3]=1.0;
    }
    void setFirstEstimate(const Eigen::Matrix<double,8,1>& fe){
        first_estimate=new Eigen::Matrix<double,8,1>(fe);
    }
    Eigen::Matrix<double,8,1>* first_estimate;
};
void testG2OVertexGSCamera();
}
#endif
