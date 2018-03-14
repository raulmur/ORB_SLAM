
#include "vio_g2o/IMU_constraint.h"
#include "vio_common/ImuGrabber.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>      // std::setprecision

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace vio;

void TestIMUFuctions(){
    Matrix<double, 10, 1> rvqs0;
    Quaterniond quat(-3.14,0.7,0.9,-2);
    quat.normalize();

    rvqs0<<0.1,2,-3, 0.5,-0.9,1.5, quat.w(), quat.x(), quat.y(), quat.z();
    cout<<rvqs0<<endl;
    Vector3d a(-1,2,11);
    Vector3d w(0.2, -0.5, -0.6);
    double dt=2;
    Matrix<double, 6,1> gwomegaw;
    gwomegaw<<1,-2,9.8, 3, -4, 2*M_PI;
    Vector3d rs0_new, vs0_new;
    Quaterniond qs0_2_s_new;

    Eigen::Vector3d r0temp = rvqs0.head<3>();
    Eigen::Vector3d v0temp = rvqs0.segment<3>(3);

    vio::strapdown_local_quat_bias(r0temp, v0temp, quat, a, w, dt,
                              gwomegaw, &rs0_new, &vs0_new, &qs0_2_s_new);
    cout<<rs0_new<<endl;
    cout<<vs0_new<<endl;
    cout<<qs0_2_s_new.coeffs()<<endl;
    cout<<endl;
    Vector3d qna,qnw,qnba,qnbw;
    Matrix<double, 15, 15> P0;
    P0.setZero();
    double acc_bias_Tc=1800;       //sec

    double acc_bias_Q=std::pow(.04*1.0e-2, 2)*(2/acc_bias_Tc);

    double gyro_bias_Tc=1800;      //sec
    double gyro_bias_Q=std::pow(18*(M_PI/180.0)/3600,2)*(2/gyro_bias_Tc); // 18deg/hr


    double acc_vrw= std::pow(80*1.0e-5,2); // 80 ug with 1 Hz
    double gyro_arw = std::pow(0.03*M_PI/180,2); // 0.03 deg/sec/sqrt(Hz)
    qna<<acc_vrw, acc_vrw, acc_vrw;
    qnw<<gyro_arw, gyro_arw, gyro_arw;
    qnba<<acc_bias_Q, acc_bias_Q, acc_bias_Q;
    qnbw<<gyro_bias_Q, gyro_bias_Q, gyro_bias_Q;

    r0temp = rvqs0.head<3>();
    v0temp = rvqs0.segment<3>(3);

    sys_local_dcm_bias(r0temp, v0temp, quat,
                       a, w, dt,  qna,qnw,qnba,qnbw,&P0);
}
void TestIMURoutine(char * argv[])
{
    using namespace Sophus;
    cv::FileStorage fSettings(argv[1], cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr<<"Wrong path to settings file!"<<endl;
        return ;
    }
    double startTime = 500.401, finishTime = 512.329;
    // choose start time as the first entry of the imu data,
    // and finish time as the time of the entry before the last one


    SE3d T_s1_to_w;
    Matrix<double, 9,1> speed_bias_1=Matrix<double, 9,1>::Zero();
    Mat vs0inw;
    fSettings["vs0inw"]>>vs0inw;
    speed_bias_1[0]=vs0inw.at<double>(0);
    speed_bias_1[1]=vs0inw.at<double>(1);
    speed_bias_1[2]=vs0inw.at<double>(2);

    cv::Mat na, nw, acc_bias_var, gyro_bias_var;
    fSettings["acc_bias_var"]>>acc_bias_var;
    fSettings["gyro_bias_var"]>>gyro_bias_var;
    fSettings["na"]>>na;
    fSettings["nw"]>>nw;
    double acc_bias_Tc=fSettings["acc_bias_Tc"];
    double gyro_bias_Tc=fSettings["gyro_bias_Tc"];

    Eigen::Vector3d q_noise_acc;
    cv::cv2eigen(na, q_noise_acc);
    q_noise_acc=q_noise_acc.cwiseAbs2();
    Eigen::Vector3d q_noise_gyr;
    cv::cv2eigen(nw, q_noise_gyr);
    q_noise_gyr*=(M_PI/180);
    q_noise_gyr=q_noise_gyr.cwiseAbs2();
    Eigen::Vector3d q_noise_accbias;
    cv::cv2eigen(acc_bias_var, q_noise_accbias);
    q_noise_accbias=q_noise_accbias.cwiseAbs2();
    q_noise_accbias*=(2/acc_bias_Tc);
    Eigen::Vector3d q_noise_gyrbias;
    cv::cv2eigen(gyro_bias_var, q_noise_gyrbias);
    q_noise_gyrbias*=(M_PI/180);
    q_noise_gyrbias=q_noise_gyrbias.cwiseAbs2();
    q_noise_gyrbias*=(2/gyro_bias_Tc);
    Matrix<double, 12,1> q_n_aw_babw;
    q_n_aw_babw.head(3)=q_noise_acc;
    q_n_aw_babw.segment(3,3)=q_noise_gyr;
    q_n_aw_babw.segment(6,3)=q_noise_accbias;
    q_n_aw_babw.tail(3)=q_noise_gyrbias;

    vio::G2oIMUParameters imu_;

    imu_.q_n_aw_babw.head<3>()=q_noise_acc;
    imu_.q_n_aw_babw.segment<3>(3)=q_noise_gyr;
    imu_.q_n_aw_babw.segment<3>(6)=q_noise_accbias;
    imu_.q_n_aw_babw.tail<3>()=q_noise_gyrbias;


    cv::Mat Rs2c, tsinc;
    fSettings["Rs2c"]>>Rs2c; fSettings["tsinc"]>>tsinc;
    Eigen::Matrix3d tempRs2c;
    Eigen::Vector3d tempVec3d;
    cv::cv2eigen(Rs2c, tempRs2c);
    cv::cv2eigen(tsinc, tempVec3d);
    SE3d T_s_to_c(tempRs2c, tempVec3d);

    T_s1_to_w= T_s_to_c;

    imu_.T_imu_from_cam=T_s_to_c.inverse();


    cv::Mat gw, wiew;
    fSettings["gw"]>>gw;
    fSettings["wiew"]>>wiew;
    cv::cv2eigen(gw, tempVec3d);
    Matrix<double, 6,1> gwomegaw;
    gwomegaw.head(3)=tempVec3d;
    cv::cv2eigen(wiew, tempVec3d);
    gwomegaw.tail(3)=tempVec3d;

    imu_.gwomegaw = gwomegaw;

    Matrix<double, 15,15> P=Matrix<double, 15,15>::Zero();
    tempVec3d.setConstant(1e-6);
    P.block(0,0,3,3)=tempVec3d.asDiagonal();
    P.block(3,3,3,3)=tempVec3d.asDiagonal();
    tempVec3d[0]=0.5*M_PI/180;
    tempVec3d[1]=0.5*M_PI/180;
    tempVec3d[2]=1.5*M_PI/180;
    tempVec3d=tempVec3d.cwiseAbs2();
    P.block(6,6,3,3)=tempVec3d.asDiagonal();
    cv2eigen(acc_bias_var, tempVec3d);
    tempVec3d=tempVec3d.cwiseAbs2()*4;
    P.block(9,9,3,3)=tempVec3d.asDiagonal();
    cv2eigen(gyro_bias_var, tempVec3d);
    tempVec3d=tempVec3d.cwiseAbs2()*4;
    P.block(12,12,3,3)=tempVec3d.asDiagonal();

    string imu_file=fSettings["imu_file"];
    imu_.sampling_interval = fSettings["sample_interval"];
    std::shared_ptr<vio::IMUProcessor> imu_proc=new vio::IMUProcessor(imu_);

    std::shared_ptr<vio::ImuGrabber> ig = new vio::ImuGrabber(imu_file, vio::IndexedPlainText,
                                                              imu_.sampling_interval);

    if(!imu_proc->bStatesInitialized){
        if(ig->getObservation(startTime) == false) //inertial readings does not cover timestamp
        {
            std::cerr<<"start time not covered by inertial readings"<< std::endl;
            exit(-1);
        }

        imu_proc->initStates(T_s1_to_w, speed_bias_1, startTime, &P);
        //ASSUME the IMU measurements are continuous and covers longer than camera data
    }

    std::string output_file = fSettings["output_file"];
    imu_proc->freeInertial(*ig, output_file, finishTime);

  /*  vector<Matrix<double, 7,1> > measurements;
    ifstream imu_stream(imu_file.c_str());
    string tempStr;
    getline(imu_stream,tempStr);
    Matrix<double,7,1> transMat;
    double precursor=0;
    int lineNum=0;
    while(!imu_stream.eof()){
        imu_stream>>precursor;
        if(imu_stream.fail())
            break;
        transMat.setZero();
        transMat[0]=precursor;
        for (int j=1; j<7; ++j)
            imu_stream>>transMat[j];
        ++lineNum;
        measurements.push_back(transMat);
        getline(imu_stream, tempStr);
    }
    imu_stream.close();
    cout<<"total line number:"<<measurements.size()<<endl;*/




   /* bool predict_cov=true;
    int every_n_reading=3;// update covariance every n IMU readings
    Vector3d r_new, r_old(T_s1_to_w.translation()), v_new, v_old(speed_bias_1.head(3));
    Quaterniond q_new, q_old(T_s1_to_w.unit_quaternion().conjugate());
    SE3d temp_Tc2w=T_s1_to_w*T_imu_from_cam;
    imu_traj_stream<<time_pair[0]<<" "<<temp_Tc2w.translation().transpose()<<" "<<v_old.transpose() <<" "<< temp_Tc2w.unit_quaternion().coeffs().transpose()<<endl;

    double dt=measurements[1][0]-time_pair[0];
    assert(dt>0);
    Matrix<double, 6, 1> est_measurement=measurements[0].tail(6)-speed_bias_1.tail(6);
    Eigen::Vector3d acc = est_measurement.head<3>();
    Eigen::Vector3d gyro = est_measurement.tail<3>();
    strapdown_local_quat_bias( r_old, v_old, q_old, acc, gyro,
                               dt, gwomegaw, &r_new, &v_new, &q_new);

    temp_Tc2w=SE3(q_new.conjugate(), r_new)*T_imu_from_cam;
    imu_traj_stream<<measurements[1][0]<<" "<<temp_Tc2w.translation().transpose()<<" "<<v_new.transpose() <<" "<< temp_Tc2w.unit_quaternion().coeffs().transpose()<<endl;

    if(predict_cov)
    {
        Eigen::Vector3d acc = est_measurement.head<3>();
        Eigen::Vector3d gyro = est_measurement.tail<3>();
        Eigen::Vector3d natemp = q_n_aw_babw.head<3>();
        Eigen::Vector3d nwtemp = q_n_aw_babw.segment<3>(3);
        Eigen::Vector3d nbatemp = q_n_aw_babw.segment<3>(6);
        Eigen::Vector3d nbwtemp = q_n_aw_babw.tail<3>();
        sys_local_dcm_bias(r_old, v_old, q_old, acc, gyro,
                           dt,natemp, nwtemp, nbatemp, nbwtemp,&P);
    }
    r_old=r_new;
    v_old=v_new;
    q_old=q_new;
    int unsigned i=1;
    for (; i<measurements.size()-1;++i){
        dt=measurements[i+1][0]-measurements[i][0];
        est_measurement=measurements[i].tail(6)-speed_bias_1.tail(6);
        Eigen::Vector3d acc = est_measurement.head<3>();
        Eigen::Vector3d gyro = est_measurement.tail<3>();
        strapdown_local_quat_bias( r_old, v_old, q_old, acc, gyro,
                                   dt, gwomegaw, &r_new, &v_new, &q_new);
        temp_Tc2w=SE3(q_new.conjugate(), r_new)*T_imu_from_cam;
        imu_traj_stream<<measurements[i+1][0]<<" "<<temp_Tc2w.translation().transpose()<<" "<<v_new.transpose() <<" "<< temp_Tc2w.unit_quaternion().coeffs().transpose()<<endl;
        if(predict_cov&&(i%every_n_reading==0))
        {
            Eigen::Vector3d acc = est_measurement.head<3>();
            Eigen::Vector3d gyro = est_measurement.tail<3>();
            Eigen::Vector3d natemp = q_n_aw_babw.head<3>();
            Eigen::Vector3d nwtemp = q_n_aw_babw.segment<3>(3);
            Eigen::Vector3d nbatemp = q_n_aw_babw.segment<3>(6);
            Eigen::Vector3d nbwtemp = q_n_aw_babw.tail<3>();
            sys_local_dcm_bias(r_old, v_old, q_old, acc, gyro,
                               dt, natemp, nwtemp, nbatemp, nbwtemp, &P);
        }
        r_old=r_new;
        v_old=v_new;
        q_old=q_new;
    }

    dt=time_pair[1]-measurements[i][0];//the last measurement
    assert(dt>=0);
    est_measurement=measurements[i].tail(6)-speed_bias_1.tail(6);
    acc = est_measurement.head<3>();
    gyro = est_measurement.tail<3>();
    strapdown_local_quat_bias( r_old, v_old, q_old, acc, gyro,
                               dt, gwomegaw, &r_new, &v_new, &q_new);
    if(predict_cov)
    {
        Eigen::Vector3d acc = est_measurement.head<3>();
        Eigen::Vector3d gyro = est_measurement.tail<3>();
        Eigen::Vector3d natemp = q_n_aw_babw.head<3>();
        Eigen::Vector3d nwtemp = q_n_aw_babw.segment<3>(3);
        Eigen::Vector3d nbatemp = q_n_aw_babw.segment<3>(6);
        Eigen::Vector3d nbwtemp = q_n_aw_babw.tail<3>();
        sys_local_dcm_bias(r_old, v_old, q_old, acc, gyro,
                           dt,natemp, nwtemp, nbatemp, nbwtemp, &P);
    }
    temp_Tc2w=SE3(q_new.conjugate(), r_new)*T_imu_from_cam;
    imu_traj_stream<<time_pair[1]<<" "<<temp_Tc2w.translation().transpose()<<" "<<v_new.transpose() <<" "<< temp_Tc2w.unit_quaternion().coeffs().transpose()<<endl;

    pred_T_s2_to_w.setQuaternion(q_new.conjugate());
    pred_T_s2_to_w.translation()=r_new;
    pred_speed_2=v_new;
    imu_traj_stream.close();*/
    cout<<imu_proc->pred_T_s2_to_w.matrix3x4()<<endl<<endl;
    cout<<imu_proc->pred_speed_bias_2<<endl<<endl;
    cout<<imu_proc->P_.diagonal().transpose()<<endl;
}
int main(int argc, char *argv[])
{
    TestIMURoutine(argv);
    return 0;
}
