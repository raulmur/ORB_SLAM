
#ifndef G2O_IMU_CONSTRAINT_H
#define G2O_IMU_CONSTRAINT_H

#include "vio/IMUErrorModel.h" //template class
#include "vio_g2o/anchored_points.h" //G2oVertexSE3

#include "vio/eigen_utils.h" //for rvec2quat, skew3d
#include "vio/ImuGrabber.h" //imugrabber

#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif

#include <g2o/core/base_multi_edge.h>

#include <iostream>

namespace vio{

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
        q_n_aw_babw.head<3>()=q_noise_acc;
        q_n_aw_babw.segment<3>(3)=q_noise_gyr;
        q_n_aw_babw.segment<3>(6)=q_noise_accbias;
        q_n_aw_babw.tail<3>()=q_noise_gyrbias;
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
    double sampling_interval; // 1/frequency, unit sec
};
// time varying lever arm vertex can represent the antenna position in the IMU frame
class G2oVertexLeverArm : public g2o::BaseVertex<3, Eigen::Vector3d >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexLeverArm               (){}

    virtual bool
    read                       (std::istream& is)
    {
        is>> _estimate[0]>>  _estimate[1]>>  _estimate[2];
        return is.good();
    }

    virtual bool
    write                      (std::ostream& os) const
    {
        os<< "measurement:"<< _estimate[0]<< " "<< _estimate[1]<<" "<< _estimate[2]<<std::endl;
        return os.good();}

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
// Concatenated shape matrix elements, including
// elements of S_a, S_g, T_s in row major order cf. IMUErrorModel's definition
class G2oVertexShapeMatrices : public g2o::BaseVertex<27, Eigen::Matrix<double,27,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexShapeMatrices               ():first_estimate(NULL){}
    ~G2oVertexShapeMatrices               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is)
    {
        for (int jack=0; jack<27; ++jack){
            is>> _estimate[jack];
        }
        return is.good();
    }

    virtual bool
    write                      (std::ostream& os) const
    {
        for (int jack=0; jack<27; ++jack){
            os<< _estimate[jack]<<" ";
        }
        os<<std::endl;
        return os.good();
    }

    // assume $\delta x= x \boxplus \hat{x}^{-1}$
    virtual void
    oplusImpl                  (const double * update_p)
    {
        Eigen::Map<const Eigen::Matrix<double, 27,1> > update(update_p);
        setEstimate(update+estimate());
    }

    virtual void
    setToOriginImpl            () {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Matrix<double,27,1>& fe){
        first_estimate=new Eigen::Matrix<double,27,1>(fe);
    }
    Eigen::Matrix<double,27,1>* first_estimate;
};
// extended speed bias vertex, including speed of the IMU sensor in world frame, accelerometer bias, gyro bias,
// elements of S_a, S_g, T_s in row major order cf. IMUErrorModel's definition
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
// for better accuracy within this function, (1) use a gravity model as simple as Heiskanen and Moritz 1967,
// (2) higher order integration method
template<typename Scalar>
void strapdown_local_quat_bias(const Eigen::Matrix<Scalar,3,1>& rs0, const Eigen::Matrix<Scalar,3,1> & vs0, const Eigen::Quaternion<Scalar>& qs0_2_s,
                               const Eigen::Matrix<Scalar,3,1> & a, const Eigen::Matrix<Scalar,3,1>& w, Scalar dt,
                               const Eigen::Matrix<Scalar, 6,1>& gomegas0,
                               Eigen::Matrix<Scalar,3,1>* rs0_new, Eigen::Matrix<Scalar,3,1>* vs0_new, Eigen::Quaternion<Scalar>* qs0_2_s_new)
{
    Eigen::Matrix<Scalar,3,1> wie2s0=gomegas0.template tail<3>();
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

    Eigen::Matrix<Scalar,3,1> vel_inc2=(gomegas0.template head<3>()-Scalar(2)*wie2s0.cross(vs0))*dt;

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
// for better accuracy and possibly slower computation, use the covariance propagation model in ethz asl msf on github
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

    Nnav.template block<3,3>(3,0)=Cs02s.transpose(); //velocity
    Nnav.template block<3,3>(6,3)=-Cs02s.transpose(); //attitude
    //system matrix
    Eigen::Matrix<Scalar, navStates, navStates> Anav=Eigen::Matrix<Scalar, navStates, navStates>::Zero();
    Anav.template block<3,3>(0,3).template setIdentity(); // rs in s0
    //Velocity  // for low grade IMU, w_ie^e is buried
    Anav.template block<3,3>(3,6)=skew3d(Cs02s.transpose()*acc);
    //Imu error model parameters
    Eigen::Matrix<Scalar, 6,6> Rimu=Eigen::Matrix<Scalar, 6,6>::Zero();

    Rimu.template topLeftCorner<3,3>()=q_n_a.asDiagonal();
    Rimu.template bottomRightCorner<3,3>()=q_n_w.asDiagonal();
    Eigen::Matrix<Scalar,6,6> Qimu_d=Eigen::Matrix<Scalar, 6,6>::Zero();
    Qimu_d.template topLeftCorner<3,3>()=q_n_ba.asDiagonal()*dt;
    Qimu_d.template bottomRightCorner<3,3>()=q_n_bw.asDiagonal()*dt;

    //Combine and discretize nav and imu models
    // this discretization can also be accomplished by Loan's matrix exponential
    // method, see sys_metric_phipsi_v000.m

    Eigen::Matrix<Scalar, navStates,navStates> Anav_d=Eigen::Matrix<Scalar, navStates,navStates>::Identity()+dt*Anav;  //Use 1st order taylor series to discretize Anav
    Eigen::Matrix<Scalar, navStates, navStates> Qnav= (Nnav*Rimu). template eval()*Nnav.transpose();
    Eigen::Matrix<Scalar, navStates, navStates> Qnav_d=dt*Scalar(0.5)*(Anav_d*Qnav+Qnav*Anav_d.transpose());      //Use trapezoidal rule to discretize Rimu

    Eigen::Matrix<Scalar, 15,15> STM=Eigen::Matrix<Scalar, 15,15>::Zero();

    STM.template topLeftCorner<navStates,navStates>()=Anav_d;
    STM.template block<navStates, 6>(0,navStates)=Nnav*dt;
    STM.template block<6,6>(navStates, navStates).setIdentity();

    Eigen::Matrix<Scalar, 15,15> Qd=Eigen::Matrix<Scalar, 15,15>::Zero();

    Qd.template topLeftCorner<navStates,navStates>()=Qnav_d;
    Qd.template block<6,6>(navStates, navStates)=Qimu_d;
    Qd.template block<navStates,6>(0,navStates)=Nnav*Qimu_d*dt*Scalar(0.5);
    Qd.template block<6,navStates>(navStates,0)=Qd.template block<navStates,6>(0,navStates).template transpose();

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
// optionally, shape_matrices which includes random constants S_a, S_g, T_s, can be used to correct IMU measurements.

template<typename Scalar>
void predictStates(const Sophus::SE3Group<Scalar> &T_sk_to_w, const Eigen::Matrix<Scalar, 9,1>& speed_bias_k,
                   const Scalar * time_pair,
                   const std::vector<Eigen::Matrix<Scalar, 7,1> >& measurements, const Eigen::Matrix<Scalar, 6,1> & gwomegaw,
                   const Eigen::Matrix<Scalar, 12, 1>& q_n_aw_babw,
                   Sophus::SE3Group<Scalar>* pred_T_skp1_to_w, Eigen::Matrix<Scalar, 3,1>* pred_speed_kp1,
                   Eigen::Matrix<Scalar, 15,15> *P, const Eigen::Matrix<Scalar, 27,1> shape_matrices= Eigen::Matrix<Scalar, 27,1>::Zero())
{
    bool predict_cov=(P!=NULL);
    int every_n_reading=2;// update covariance every n IMU readings,
    // the eventual covariance has little to do with this param as long as it remains small
    Eigen::Matrix<Scalar, 3,1> r_new, r_old(T_sk_to_w.translation()), v_new, v_old(speed_bias_k.template head<3>());
    Eigen::Quaternion<Scalar> q_new, q_old(T_sk_to_w.unit_quaternion().conjugate());
    Scalar dt=measurements[1][0]-time_pair[0];
    Scalar covupt_time(time_pair[0]);//the time to which the covariance is updated. N.B. the initial covariance is updated to $t_k$
    Scalar maxTimeGap(0.1);
    assert(dt>Scalar(0)&&dt<=maxTimeGap); // dt should be smaller than the maximum time gap between back to back imu readings

    IMUErrorModel<Scalar> iem(shape_matrices, speed_bias_k.template block<6,1>(3,0));
    iem.estimate(measurements[0].template block<3,1>(4,0), measurements[0].template block<3,1>(1,0));

    const Eigen::Matrix<Scalar, 3,1> qna=q_n_aw_babw.template head<3>(), qnw=q_n_aw_babw.template segment<3>(3),
            qnba=q_n_aw_babw.template segment<3>(6),qnbw=q_n_aw_babw.template tail<3>();
    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                               dt, gwomegaw, &r_new, &v_new, &q_new);

    if(predict_cov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
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
        iem.estimate(measurements[i].template block<3,1>(4,0), measurements[i].template block<3,1>(1,0));
        strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est,
                                   iem.w_est, dt, gwomegaw, &r_new, &v_new, &q_new);
        if(predict_cov&&(i%every_n_reading==0))
        {
            sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est,
                               iem.w_est, measurements[i+1][0]-covupt_time,qna, qnw, qnba, qnbw,P);
            covupt_time=measurements[i+1][0];
        }
        r_old=r_new;
        v_old=v_new;
        q_old=q_new;
    }
    //assert(i==measurements.size()-1);
    dt=time_pair[1]-measurements[i][0];//the last measurement
    assert(dt>=Scalar(0)&& dt<maxTimeGap);
    iem.estimate(measurements[i].template block<3,1>(4,0), measurements[i].template block<3,1>(1,0));
    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                               dt, gwomegaw, &r_new, &v_new, &q_new);
    if(predict_cov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                           time_pair[1]-covupt_time,qna, qnw, qnba, qnbw,P);
        covupt_time=time_pair[1];
    }
    pred_T_skp1_to_w->setQuaternion(q_new.conjugate());
    pred_T_skp1_to_w->translation()=r_new;
    (*pred_speed_kp1)=v_new;
}
// g2oedgeimuconstraint with 4 vertices, g2overtexse3, g2overtexspeedbias at k, g2overtexse3, g2overtexspeedbias at k+1
// where g2overtexse3 is the transform from the world frame to the camera frame(a reference sensor frame), g2overtexspeedbias is
// the velocity of the IMU sensor in the world frame, and IMU acc and gyro biases
// the reference frame referred to as the camera frame, however, can be other well defined sensor frame depending on applications,
// for example, in integrating GPS/IMU, the reference sensor frame can be the IMU frame
// observations are the difference between predicted states at k+1 and the states at k+1
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

// extended g2oedgeimuconstraint with 5 vertices, g2overtexse3, g2overtexspeedbias at k, g2overtexse3, g2overtexspeedbias at k+1, and g2overtexshapematrices
// g2overtexshapematrices is assumed to be random constant
// where g2overtexse3 is the transform from the world frame to the camera frame(a reference sensor frame), g2overtexspeedbias is
// the velocity of the IMU sensor in the world frame, and IMU acc and gyro biases
// observations are the difference between predicted states at k+1 and the states at k+1

class G2oEdgeIMUConstraintEx : public  g2o::BaseMultiEdge<15, std::vector<Eigen::Matrix<double, 7,1> > >
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
    bool operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1,
                      const T* pXsbkp1, const T* pSags, T* error) const;

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
// $R_s^w$ inertial sensor to world rotation comes from G2oVertexSE3, i.e., transform from world to camera frame, i.e., the reference sensor frame
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
// here I used G2oVertexSE3 to represent transform from world frame to the reference sensor frame(the so-called cam frame in this case),
// G2oVertexLeverArm to represent antenna position in the reference sensor frame
// often the world frame is defined as a frame anchored to the earth which is convenient for navigation and has known constant transformation to the ECEF frame,
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
        _error = _measurement-v->estimate().inverse()*vl->estimate();
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
        //alternatively,
//        Eigen::Matrix<double, 3,1> temp_error;
//        (*this)(v->estimate().data(), vl->estimate().data(), temp_error.data());
//        _error = temp_error;
    }
    template <typename T>
    bool operator()(const T* const prior, const T* const posterior, T* residual) const {
        for(int jack=0; jack<3; ++jack)
            residual[jack]= prior[jack]- posterior[jack];
        return true;
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


//prior of pose, measurement is transform from camera(the reference sensor frame) to world frame, vertex states are transform from world to camera frame
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
        const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
        Eigen::Matrix<double,6,1> error= Sophus::SE3d::log(_measurement*v->estimate());
        _jacobianOplusXi = third(_measurement,  error);
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

// prior of S_a, S_g, T_s shape matrices of accelerometers and gyros
class G2oEdgeShapeMatrices : public g2o::BaseUnaryEdge<27, Eigen::Matrix<double ,27,1>, G2oVertexShapeMatrices>
{
public:
    G2oEdgeShapeMatrices()
    {
    }

    void computeError()
    {
        const G2oVertexShapeMatrices* v = static_cast<const G2oVertexShapeMatrices*>(_vertices[0]);
        _error =_measurement- v->estimate();
    }
    template <typename T>
    bool operator()(const T* const prior, const T* const posterior, T* residual) const {
        for(int jack=0; jack<27; ++jack)
            residual[jack]= prior[jack]- posterior[jack];
        return true;
    }
    void linearizeOplus()
    {
        _jacobianOplusXi= -Eigen::Matrix<double, 27, 27>::Identity();
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
// prior of lever arm between two sensors
class G2oUnaryEdgeLeverArm : public g2o::BaseUnaryEdge< 3, Eigen::Matrix<double ,3,1>, G2oVertexLeverArm>
{
public:
    G2oUnaryEdgeLeverArm()
    {
    }

    void computeError()
    {
        const G2oVertexLeverArm* v = static_cast<const G2oVertexLeverArm*>(_vertices[0]);
        _error =_measurement- v->estimate();
    }
    void linearizeOplus()
    {
        _jacobianOplusXi= -Eigen::Matrix<double, 3, 3>::Identity();
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
    IMUProcessor(const std::string imu_file, const double sample_interval,const G2oIMUParameters &imu,
                 vio::IMUFileType fileType=vio::PlainText);

    // output the transformation from previous to current camera frame
    // propagate multiple steps from the last time_frame to current time_frame
    // which is assumed to have a larger interval than the IMU sampling interval
    Sophus::SE3d propagate(const double time_frame);

    void printStateAndCov(std::ofstream &output, double time)const;

    // read each measurement from the IMU file and do free inertial integration
    void freeInertial(std::string output_file, double finish_time);

    // return true if init successful
    bool initStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1, const double timestamp,
                    Eigen::Matrix<double, 15, 15> *pCov=(Eigen::Matrix<double, 15, 15> *)NULL);
    void resetStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1);

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

    vio::IMUFileType ft;
    vio::IMUGrabber ig;
    G2oIMUParameters imu_;
    bool bStatesInitialized;
    Eigen::Matrix<double, 15, 15> P_; //covariance of error states as described in sys_local_dcm_bias()
    bool bPredictCov; //predict covariance or not
    std::ofstream imu_traj_stream; //log IMU states and covariances
};

// camera related parameters in visual inertial navigation, following Li et al ICRA 2014 eq (10)
// 14 dimensions are position of the IMU frame in the reference sensor, i.e., camera frame, c_x, c_y, f_x, f_y,
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
// $p_s^c$ is the IMU sensor in the camera frame, t_d is the offset
// between the IMU sensor time and the camera timestamp, i.e., time of image in IMU sensor time = time of image + t_d
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
// $T_s^c, t_d$, transform from IMU sensor frame to the camera frame, t_d is the time delay
// between the IMU sensor time and the camera timestamp, i.e., time of image in IMU sensor time = time of image + t_d
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

        Eigen::Map<const Eigen::Matrix<double,6,1> > update(update_p);
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
