/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/
#include "Tracking.h"
#include "config.h"
#include "utils.h"
#include "g2o_types/global.h" //performance monitor
#include "g2o_types/IMU_constraint.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h> //for cholmod
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#include <opencv2/core/eigen.hpp>
#ifdef SLAM_USE_ROS
#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
#endif
#include "g2o_types/timegrabber.h"
#include<opencv2/opencv.hpp>

#include"ORBmatcher.h"
#include"FramePublisher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include <vikit/pinhole_camera.h>

#include<iostream>
#include<fstream>
#include <queue>
#include <utility>

using namespace std;
using namespace Sophus;
using namespace ScaViSLAM;

namespace ORB_SLAM
{
#ifdef SLAM_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

static bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}
#ifdef SLAM_USE_ROS
Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher*pFramePublisher, MapPublisher *pMapPublisher, Map *pMap, string strSettingPath):
    mState(NO_IMAGES_YET),mpCurrentFrame(NULL),mpLastFrame(NULL), mpInitialFrame(NULL), mpORBVocabulary(pVoc), mpInitializer(NULL),
    mnTemporalWinSize(Config::temporalWindowSize()),mnSpatialWinSize(Config::spatialWindowSize()),
    mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), mpMap(pMap),
    mfsSettings(strSettingPath, cv::FileStorage::READ),mpLastKeyFrame(NULL),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mVelocity(Sophus::SE3d()),
    mbUseIMUData(false), mnStartId(mfsSettings["startIndex"]), mnFeatures(mfsSettings["ORBextractor.nFeatures"]),
    mMotionModel(Eigen::Vector3d(0,0,0),Eigen::Quaterniond(1,0,0,0))
#else
Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher*pFramePublisher, /*MapPublisher *pMapPublisher,*/ Map *pMap, string strSettingPath):
    mState(NO_IMAGES_YET),mpCurrentFrame(NULL),mpLastFrame(NULL), mpInitialFrame(NULL), mpORBVocabulary(pVoc), mpInitializer(NULL),
    mnTemporalWinSize(Config::temporalWindowSize()),mnSpatialWinSize(Config::spatialWindowSize()),
    mpFramePublisher(pFramePublisher),/*mpMapPublisher(pMapPublisher),*/ mpMap(pMap),
    mfsSettings(strSettingPath, cv::FileStorage::READ),mpLastKeyFrame(NULL),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false), mVelocity(Sophus::SE3d()),
    mbUseIMUData(false), mnStartId(mfsSettings["startIndex"]), mnFeatures(mfsSettings["ORBextractor.nFeatures"]),
    mMotionModel(Eigen::Vector3d(0,0,0),Eigen::Quaterniond(1,0,0,0))
#endif
{
#ifdef SLAM_TRACE
    // Initialize Performance Monitor
    g_permon = new vk::PerformanceMonitor();
    g_permon->addTimer("extract_quadmatches");
    g_permon->addTimer("track_previous_frame");
    g_permon->addTimer("stereo_matching");
    g_permon->addTimer("create_frame");

    g_permon->addTimer("local_optimize");
    g_permon->addTimer("triangulate_new_mappoint");
    g_permon->addTimer("tot_time");
    g_permon->addTimer("local_mapper");
    g_permon->addTimer("loop_closer");
    g_permon->addLog("time_frame");
    string trace_name("slam_profile");
    string trace_dir= mfsSettings["trace_dir"];
    g_permon->init(trace_name, trace_dir);
#endif
    // Load camera parameters from settings file
    int width = mfsSettings["Camera.width"];
    int height = mfsSettings["Camera.height"];
    float fx = mfsSettings["Camera.fx"];
    float fy = mfsSettings["Camera.fy"];
    float cx = mfsSettings["Camera.cx"];
    float cy = mfsSettings["Camera.cy"];
    Eigen::Matrix<double, 4, 1> distCoef;
    distCoef(0) = mfsSettings["Camera.k1"];
    distCoef(1) = mfsSettings["Camera.k2"];
    distCoef(2) = mfsSettings["Camera.p1"];
    distCoef(3) = mfsSettings["Camera.p2"];

    cam_ = new vk::PinholeCamera( width, height, fx, fy, cx,cy,
                                  distCoef(0), distCoef(1),
                                  distCoef(2),distCoef(3));

#ifndef MONO
    cv::Mat matTl2r;
    mfsSettings["Stereo.se3Left2Right"]>>matTl2r;
    mTl2r= Converter::toSE3d(matTl2r);
    cv::Mat rightK, rightDistCoef;
    mfsSettings["Stereo.matRightK"]>>rightK;
    mfsSettings["Stereo.matRightDistCoef"]>>rightDistCoef;

    right_cam_ = new vk::PinholeCamera(width, height,
                                       rightK.at<float>(0,0), rightK.at<float>(1,1),
                                       rightK.at<float>(0,2), rightK.at<float>(1,2),
                                       rightDistCoef.at<float>(0), rightDistCoef.at<float>(1),
                                       rightDistCoef.at<float>(2),rightDistCoef.at<float>(3));

#endif
    mFps = mfsSettings["Camera.fps"];
    if(mFps==0)
        mFps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = 2;//must be at least 2, otherwise may cause second keyframe reinserted as keyframe//was 18*mFps/30;

    cout << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << distCoef(0) << endl;
    cout << "- k2: " << distCoef(1) << endl;
    cout << "- p1: " << distCoef(2) << endl;
    cout << "- p2: " << distCoef(3) << endl;
    cout << "- fps: " << mFps << endl;

    //IMU related parameters, assume the world frame is the LEFT camera frame at startIndex image, right down forward
    //given image indexed k and its timestamp t(k), assume we have IMU readings indexed p(k) s.t. t(p(k)-1)<=t(k)<t(p(k))
    mbUseIMUData=to_bool(mfsSettings["use_imu_data"]);
    ginw.setZero();
    if(mbUseIMUData)
    {
        imu_sample_interval=mfsSettings["sample_interval"];
        cv::Mat na, nw, acc_bias_var, gyro_bias_var;
        mfsSettings["na"]>>na;
        mfsSettings["nw"]>>nw;
        mfsSettings["acc_bias_var"]>>acc_bias_var;
        mfsSettings["gyro_bias_var"]>>gyro_bias_var;

        double acc_bias_Tc=mfsSettings["acc_bias_Tc"];        //half correlation time
        double gyro_bias_Tc=mfsSettings["gyro_bias_Tc"];

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

        imu_.q_n_aw_babw.head(3)=q_noise_acc;
        imu_.q_n_aw_babw.segment(3,3)=q_noise_gyr;
        imu_.q_n_aw_babw.segment(6,3)=q_noise_accbias;
        imu_.q_n_aw_babw.tail(3)=q_noise_gyrbias;

        cv::Mat Rs2c, tsinc;
        mfsSettings["Rs2c"]>>Rs2c; mfsSettings["tsinc"]>>tsinc;
        Eigen::Matrix3d tempRs2c;
        Eigen::Vector3d tempVec3d;
        cv::cv2eigen(Rs2c, tempRs2c);
        cv::cv2eigen(tsinc, tempVec3d);
        SE3d T_s_2_c(tempRs2c, tempVec3d);
        imu_.T_imu_from_cam=T_s_2_c.inverse();

        cv::Mat matGinw, omegaew;
        mfsSettings["gw"]>>matGinw;
        mfsSettings["wiew"]>>omegaew;

        imu_.gwomegaw.setZero();
        cv::cv2eigen(matGinw, ginw);
        imu_.gwomegaw.head(3)=ginw;
        cv::cv2eigen(omegaew, tempVec3d);
        imu_.gwomegaw.tail(3)=tempVec3d;
    }

    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    libviso2::VisualOdometryStereo::parameters param;
    param.calib.f  = mfsSettings["Camera.fx"]; // focal length in pixels
    param.calib.cu = mfsSettings["Camera.cx"]; // principal point (u-coordinate) in pixels
    param.calib.cv = mfsSettings["Camera.cy"]; // principal point (v-coordinate) in pixels
    param.base     = -mTl2r.translation()[0]; // baseline in meters
    param.inlier_threshold =sqrt(5.991);
    mVisoStereo.setParameters(param);
    cout<<"Refinedment viso2: "<<param.match.refinement<<endl;
    mPose=libviso2::Matrix::eye(4);
    // use external saved visual odometry
    mStereoSFM.init(mfsSettings["qcv_tracks"], mfsSettings["qcv_deltas"]);

    int nRGB = mfsSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    float fScaleFactor = mfsSettings["ORBextractor.scaleFactor"];
    int nLevels = mfsSettings["ORBextractor.nLevels"];
    int fastTh = mfsSettings["ORBextractor.fastTh"];
    int Score = mfsSettings["ORBextractor.nScoreType"];

    assert(Score==1 || Score==0);

    mpORBextractor = new ORBextractor(mnFeatures,fScaleFactor,nLevels,Score,fastTh);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << mnFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Fast Threshold: " << fastTh << endl;
    if(Score==0)
        cout << "- Score: HARRIS" << endl;
    else
        cout << "- Score: FAST" << endl;


    // ORB extractor for initialization
    // Initialization uses only points from the finest scale level
    mpIniORBextractor = new ORBextractor(mnFeatures*2,1.2,8,Score,fastTh);

#ifdef SLAM_USE_ROS
    tf::Transform tfT;
    tfT.setIdentity();
    mTfBr.sendTransform(tf::StampedTransform(tfT,ros::Time::now(), "/ORBSLAM_DWO/World", "/ORBSLAM_DWO/Camera"));
#endif
}
Tracking::~Tracking()
{
#ifdef SLAM_TRACE
    delete g_permon;
#endif
    delete cam_;
#ifndef MONO
    delete right_cam_;
#endif
    if(mpInitialFrame)
        delete mpInitialFrame;
    mpInitialFrame=NULL;
    if(mpCurrentFrame){
        if(mpLastFrame == mpCurrentFrame){
            mpLastFrame=NULL;
        }
        delete mpCurrentFrame;
        mpCurrentFrame=NULL;
    }
    if(mpLastFrame){
        delete mpLastFrame;
        mpLastFrame=NULL;
    }
}
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetKeyFrameDatabase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

void Tracking::Run()
{
    //    ros::NodeHandle nodeHandler;
    //    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &Tracking::GrabImage, this);

    //    ros::spin();
    enum dataset_type {KITTIOdoSeq=0, Tsukuba, MalagaUrbanExtract6, CrowdSourcedData } experim=Tsukuba;

    string dataset=mfsSettings["dataset"];
    if (dataset.compare("KITTIOdoSeq")==0)
        experim =KITTIOdoSeq;
    else if (dataset.compare("Tsukuba")==0)
        experim=Tsukuba;
    else if(dataset.compare("MalagaUrbanExtract6")==0)
        experim=MalagaUrbanExtract6;
    else if(dataset.compare("CrowdSourcedData")==0)
        experim=CrowdSourcedData;
    else{
        cerr<<"Unsupported dataset:"<<dataset<<endl;
        return;
    }

    int numImages=mnStartId;
    int totalImages=mfsSettings["finishIndex"];
    string dir = mfsSettings["input_path"]; // sequence directory
    string output_file=mfsSettings["output_file"];
    string output_point_file= mfsSettings["output_point_file"];
    ofstream out_stream(output_file.c_str(), std::ios::out);
    out_stream<<"%Each row is timestamp, pos of camera in world frame, rotation to world from camera frame in quaternion xyzw format"<<endl;
    out_stream << fixed;

    ScaViSLAM::IMUProcessor* imu_proc=NULL;
    Sophus::SE3d T_s1_to_w;
    Eigen::Matrix<double, 9, 1> speed_bias_1=Eigen::Matrix<double, 9, 1>::Zero();
    Sophus::SE3d predTcp;
    if(mbUseIMUData){
        string imu_file=mfsSettings["imu_file"];
        imu_proc=new ScaViSLAM::IMUProcessor(imu_file, imu_sample_interval, imu_);

        //initialize IMU states
        T_s1_to_w=imu_.T_imu_from_cam.inverse();
        cv::Mat vs0inw;
        mfsSettings["vs0inw"]>>vs0inw;

        speed_bias_1[0]=vs0inw.at<double>(0);
        speed_bias_1[1]=vs0inw.at<double>(1);
        speed_bias_1[2]=vs0inw.at<double>(2);
    }

    vk::Timer timer_;             //!< Stopwatch to measure time to process frame.
    timer_.start();

    double time_frame(-1);                  //timestamp of current frame
    double time_pair[2]={-1,-1};              // timestamps of the previous and current images

    if( experim == CrowdSourcedData){
        cv::VideoCapture cap(dir);

        double rate = cap.get(CV_CAP_PROP_FPS);
        if(!rate) cerr<<"Error opening video file "<<dir<<endl;
        cap.set(CV_CAP_PROP_POS_FRAMES, numImages); //start from numImages, 0 based index
        totalImages =std::min(totalImages, (int)cap.get(CV_CAP_PROP_FRAME_COUNT));
        int width= cap.get(CV_CAP_PROP_FRAME_WIDTH), height= cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        int downscale = GetDownScale(width, height, 1280);
        cv::Mat left_img, dst;
#ifdef SLAM_USE_ROS
        ros::Rate r(mFps);
        while(ros::ok()&& numImages<=totalImages)
#else
        while(numImages<=totalImages)
#endif
        {
            assert(cap.get(CV_CAP_PROP_POS_FRAMES) == numImages);
            time_frame= cap.get(CV_CAP_PROP_POS_MSEC)/1000.0;
            cap.read(left_img);

            if(downscale>1){
                cv::pyrDown(left_img, dst, cv::Size((width+1)/2, (height+1)/2));
                left_img= dst;
            }
            time_pair[0]=time_pair[1];
            time_pair[1]=time_frame;

            if(left_img.cols != cam_->width() || left_img.rows!= cam_->height())
            {
                cerr<<"Incompatible image size, check setting file Camera.width .height fields or the end of video!"<<endl;
                return;
            }
            if(left_img.channels()==3)
            {
                cv::Mat temp;
                if(mbRGB)
                    cvtColor(left_img, temp, CV_RGB2GRAY);
                else
                    cvtColor(left_img, temp, CV_BGR2GRAY);
                left_img=temp;
            }
            SLAM_LOG(time_frame);
            SLAM_DEBUG_STREAM("processing frame "<< numImages-mnStartId);
            SLAM_START_TIMER("tot_time");

            if(mbUseIMUData){
                if(!imu_proc->bStatesInitialized){
                    imu_proc->initStates(T_s1_to_w, speed_bias_1, time_frame);//ASSUME the IMU measurements are continuous and covers longer than camera data
                    ProcessFrameMono(left_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                     NULL, speed_bias_1);
                }
                else{
                    predTcp=imu_proc->propagate(time_frame);
                    ProcessFrameMono(left_img, time_frame,
                                     imu_proc->getMeasurements(), &predTcp, imu_proc->speed_bias_1);

                    if(mState == WORKING){
                        imu_proc->resetStates((imu_.T_imu_from_cam*mpLastFrame->mTcw).inverse(), mpLastFrame->speed_bias);
                    }
                }
            }else if(Config::useDecayVelocityModel()){
                Eigen::Vector3d trans;
                Eigen::Quaterniond quat;
                mMotionModel.PredictNextCameraMotion(trans,quat);
                predTcp= SE3d(quat,trans);
                ProcessFrameMono(left_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                 &predTcp);
                if(mState == WORKING){
                    SE3d Twc= mpLastFrame->mTcw.inverse();
                    mMotionModel.UpdateCameraPose(Twc.translation(), Twc.unit_quaternion());
                }
            }
            else
                ProcessFrameMono(left_img, time_frame);

            SLAM_STOP_TIMER("tot_time");
            //output position for debug
#ifdef SLAM_TRACE
            g_permon->writeToFile();
#endif
            if(mState == WORKING){
                Eigen::Matrix<double,4,1> q = mpLastFrame->GetPose().unit_quaternion().conjugate().coeffs();
                Eigen::Vector3d t = mpLastFrame->GetCameraCenter();
                out_stream << setprecision(6) << mpLastFrame->mTimeStamp << " " << t.transpose()<<
                              " " << q.transpose() <<" "<< mpLastFrame->speed_bias.transpose()<<endl;
            }
            ++numImages;

            mpFramePublisher->Refresh();
            CheckResetByPublishers();
#ifdef SLAM_USE_ROS
            mpMapPublisher->Refresh();
            r.sleep();
#endif
        }
    }else{
#ifdef SLAM_OUTPUT_VISO2
        std::size_t pos = output_file.find(".txt");
        string viso2_output_file= output_file.substr(0, pos) +"_viso2.txt";
        ofstream viso2_stream(viso2_output_file);
#endif
        string time_filename=mfsSettings["time_file"]; //timestamps for frames
        TimeGrabber tg(time_filename);

        char base_name[256];                // input file names
        string left_img_file_name;
        string right_img_file_name;



#ifdef SLAM_USE_ROS
        ros::Rate r(mFps);
        while(ros::ok()&& numImages<=totalImages)
#else
        while(numImages<=totalImages)
#endif
        {
            switch(experim){
            case KITTIOdoSeq:
                sprintf(base_name,"%06d.png",numImages);
                left_img_file_name  = dir + "/image_0/" + base_name;
                right_img_file_name = dir + "/image_1/" + base_name;
                time_frame=tg.readTimestamp(numImages);
                break;
            case Tsukuba:
                sprintf(base_name,"%05d.png",numImages);
                left_img_file_name  = dir + "/tsukuba_daylight_L_" + base_name;
                right_img_file_name = dir + "/tsukuba_daylight_R_" + base_name;
                time_frame=(numImages-1)/30.0;
                break;
            case MalagaUrbanExtract6:
                time_frame=tg.extractTimestamp(numImages);
                left_img_file_name=tg.last_left_image_name;
                right_img_file_name=left_img_file_name.substr(0, 30)+ "right"+left_img_file_name.substr(left_img_file_name.length()-4, 4);
                left_img_file_name=dir+ "/"+ left_img_file_name;
                right_img_file_name=dir+ "/"+ right_img_file_name;
            default:
                cerr<<"Please implement interface fot this dataset!"<<endl;
                break;
            }
            time_pair[0]=time_pair[1];
            time_pair[1]=time_frame;
            cv::Mat left_img=cv::imread(left_img_file_name, 0);
            cv::Mat right_img=cv::imread(right_img_file_name, 0);

            if(left_img.cols != cam_->width() || left_img.rows!= cam_->height())
            {
                cerr<<"Incompatible image size, check setting file Camera.width .height fields!"<<endl;
                return;
            }

            SLAM_LOG(time_frame);
            SLAM_DEBUG_STREAM("processing frame "<< numImages-mnStartId);
            SLAM_START_TIMER("tot_time");
#ifdef MONO
            if(mbUseIMUData){
                if(!imu_proc->bStatesInitialized){
                    imu_proc->initStates(T_s1_to_w, speed_bias_1, time_frame);//ASSUME the IMU measurements are continuous and covers longer than camera data
                    ProcessFrameMono(left_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                     NULL, speed_bias_1);
                }
                else{
                    predTcp=imu_proc->propagate(time_frame);
                    ProcessFrameMono(left_img, time_frame,
                                     imu_proc->getMeasurements(), &predTcp, imu_proc->speed_bias_1);

                    if(mState == WORKING){
                        imu_proc->resetStates((imu_.T_imu_from_cam*mpLastFrame->mTcw).inverse(), mpLastFrame->speed_bias);
                    }
                }
            }else if(Config::useDecayVelocityModel()){
                Eigen::Vector3d trans;
                Eigen::Quaterniond quat;
                mMotionModel.PredictNextCameraMotion(trans,quat);
                predTcp= SE3d(quat,trans);
                ProcessFrameMono(left_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                 &predTcp);
                if(mState == WORKING){
                    SE3d Twc= mpLastFrame->mTcw.inverse();
                    mMotionModel.UpdateCameraPose(Twc.translation(), Twc.unit_quaternion());
                }
            }
            else
                ProcessFrameMono(left_img, time_frame);
#else
            // either processframe or processframeQCV is supposed to work in this section
            if(mbUseIMUData){
                if(!imu_proc->bStatesInitialized){
                    imu_proc->initStates(T_s1_to_w, speed_bias_1, time_frame);//ASSUME the IMU measurements are continuous and covers longer than camera data
                    ProcessFrame(left_img, right_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                 NULL, speed_bias_1);
                }
                else{
                    predTcp=imu_proc->propagate(time_frame);
                    assert(imu_proc->getMeasurements().size());
                    ProcessFrame(left_img, right_img, time_frame,
                                 imu_proc->getMeasurements(), &predTcp, imu_proc->speed_bias_1);
                }
                if(mState == WORKING){
                    imu_proc->resetStates((imu_.T_imu_from_cam*mpLastFrame->mTcw).inverse(), mpLastFrame->speed_bias);
                }
            }
            else if(Config::useDecayVelocityModel()){
                Eigen::Vector3d trans;
                Eigen::Quaterniond quat;
                mMotionModel.PredictNextCameraMotion(trans,quat);
                predTcp= SE3d(quat,trans);
                ProcessFrame(left_img, right_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                             &predTcp);
                if(mState == WORKING){
                    SE3d Twc= mpLastFrame->mTcw.inverse();
                    mMotionModel.UpdateCameraPose(Twc.translation(), Twc.unit_quaternion());
                }
            }
            else{
                ProcessFrame(left_img, right_img, time_frame);
            }
#endif
            SLAM_STOP_TIMER("tot_time");
            //output position for debug
#ifdef SLAM_TRACE
            g_permon->writeToFile();
#endif
#ifdef SLAM_OUTPUT_VISO2
            if(mVisoStereo.Tr_valid)
                viso2_stream<<mpLastFrame->mTimeStamp<<" "<<mPose.getMat(0, 0, 0,3)<<" "<<
                              mPose.getMat(1, 0, 1,3)<<" "<<mPose.getMat(2, 0, 2,3)<<endl;
#endif
            if(mState == WORKING){
                Eigen::Matrix<double,4,1> q = mpLastFrame->GetPose().unit_quaternion().conjugate().coeffs();
                Eigen::Vector3d t = mpLastFrame->GetCameraCenter();
                out_stream << setprecision(6) << mpLastFrame->mTimeStamp << " " << t.transpose()<<
                              " " << q.transpose() <<" "<< mpLastFrame->speed_bias.transpose()<<endl;
            }
            ++numImages;

            mpFramePublisher->Refresh();
            CheckResetByPublishers();
#ifdef SLAM_USE_ROS
            mpMapPublisher->Refresh();
            r.sleep();
#endif
        }
    }
#ifdef SLAM_OUTPUT_VISO2
    viso2_stream.close();
#endif
    //do we need to wait for loop closing
    while(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()){
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
    double calc_time =  timer_.stop();
    double time_per_frame=calc_time/(numImages-mnStartId+1);
    cout<<"Calc_time:"<<calc_time<<";"<<"time per frame:"<<time_per_frame<<endl;
    // Save keyframe poses at the end of the execution
    vector<ORB_SLAM::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

    SLAM_INFO_STREAM("Saving Keyframe Trajectory to "<<output_file);
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        Eigen::Matrix<double,4,1> q = pKF->GetPose().unit_quaternion().conjugate().coeffs();
        Eigen::Vector3d t = pKF->GetCameraCenter();
        out_stream << setprecision(6) << pKF->mTimeStamp << " " << t.transpose()<<
                    " " << q.transpose() <<" "<< pKF->speed_bias.transpose()<<endl;
    }
    out_stream.close();
    out_stream.open(output_point_file.c_str(), std::ios::out);
    out_stream<<"%Each row is point id, position xyz in world frame"<<endl;
    out_stream << fixed;
    vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    for(size_t i=0; i<vpMPs.size(); ++i)
    {
        ORB_SLAM::MapPoint* pMP = vpMPs[i];
        if(pMP->isBad())
            continue;
        Eigen::Vector3d t= pMP->GetWorldPos();
        out_stream << setprecision(6) << pMP->mnId<< " " << t.transpose()<<endl;
    }
    out_stream.close();
    SLAM_INFO_STREAM("Saved MapPoints to "<<output_point_file);
}

void Tracking::GrabImage(cv::Mat& im, double timeStampSec)
{
    if(im.channels()==3)
    {
        cv::Mat temp;
        if(mbRGB)
            cvtColor(im, temp, CV_RGB2GRAY);
        else
            cvtColor(im, temp, CV_BGR2GRAY);
        im=temp;
    }
    if(mState==WORKING || mState==LOST)
        mpCurrentFrame=new Frame(im,timeStampSec,mpORBextractor,mpORBVocabulary, cam_);
    else
        mpCurrentFrame=new Frame(im,timeStampSec,mpIniORBextractor,mpORBVocabulary, cam_);

    // Depending on the state of the Tracker we perform different tasks

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        Initialize();
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;
        bool bForcedReloc = RelocalisationRequested();
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        if(mState==WORKING && !bForcedReloc)
        {
            //huai: TODO: predict poses from an IMU thread
            if(!Config::useDecayVelocityModel() || mpMap->KeyFramesInMap()<4 || mpCurrentFrame->mnId<mnLastRelocFrameId+2)
                bOK = TrackPreviousFrame();
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackPreviousFrame();
            }
        }
        else
        {            
            bOK = Relocalisation();
        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
            bOK = TrackLocalMap();

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
#ifdef SLAM_USE_ROS
            mpMapPublisher->SetCurrentCameraPose(mpCurrentFrame->mTcw);
#endif
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(size_t i=0; i<mpCurrentFrame->mvbOutlier.size();i++)
            {
                if(mpCurrentFrame->mvpMapPoints[i] && mpCurrentFrame->mvbOutlier[i])
                    mpCurrentFrame->mvpMapPoints[i]=NULL;
            }
        }

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }

        // Update motion model
      /*  if(mbMotionModel)
        {
            if(bOK)
            {
                //huai: TODO: use a Kalman filter to update se3 velocity
                mVelocity = mpCurrentFrame->mTcw*mpLastFrame->mTcw.inverse();
            }
            else
                mVelocity = Sophus::SE3d();
        }*/
        delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }

    // Update drawer
    mpFramePublisher->Update(this, im);
#ifdef SLAM_USE_ROS

    Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
    Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
    tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                    Rwc(1,0),Rwc(1,1),Rwc(1,2),
                    Rwc(2,0),Rwc(2,1),Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));

    tf::Transform tfTcw(M,V);
    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World", "ORBSLAM_DWO/Camera"));
#endif
}

// Triangulate new map points based on quadmatches between current frame and its preceding frame which is a new keyframe
// a grid is used to ensure uniform distribution of map points in the new keyframe,
// alternatively, a quad tree as in scavislam by Strasdat or rslam by Mei may be used.
// alternatively, a tank of tracks of features which is generalized quad matches
// can be used to create new mappoints of low uncertainty. It requires updating tracks with new quad matches
void Tracking::CreateNewMapPoints(const std::vector<p_match>& vQuadMatches)
{
    int numNewPoints=0;
    //mpLastKeyFrame is the new keyframe, mvpTemporalFrames.back() is its previous frame(can be a keyframe)
    mpLastKeyFrame->mpFG= new FeatureGrid(30,mnFeatures, mpLastKeyFrame->mnMaxX,
                                          mpLastKeyFrame->mnMinX, mpLastKeyFrame->mnMaxY, mpLastKeyFrame->mnMinY);
    FeatureGrid *pFG= mpLastKeyFrame->mpFG;
    mpLastKeyFrame->setExistingFeatures(*pFG);

    Frame* pCurrFrame = mpCurrentFrame;
    Sophus::SE3d proxy= mpLastKeyFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c1= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c1= Tw2c1.topLeftCorner<3,3>();
    Eigen::Vector3d twinc1=Tw2c1.col(3);

    proxy = pCurrFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c2= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c2= Tw2c2.topLeftCorner<3,3>();
    Eigen::Vector3d twinc2=Tw2c2.col(3);

    Eigen::Matrix<double,3,4> Tw2c1r= mpLastKeyFrame->GetPose(false).matrix3x4();
    Eigen::Matrix<double,3,4> Tw2c2r= pCurrFrame->GetPose(false).matrix3x4();

    const float ratioFactor = 1.5f*mpLastKeyFrame->GetScaleFactor();
    Eigen::Vector3d Ow1 = mpLastKeyFrame->GetCameraCenter();
    Eigen::Vector3d Ow2 = pCurrFrame->GetCameraCenter();

    std::vector< Eigen::Vector3d> obs(4);
    std::vector<Sophus::SE3d> frame_poses(4);
    frame_poses[0]= Sophus::SE3d(Tw2c1.topLeftCorner<3,3>(), Tw2c1.col(3));
    frame_poses[1]= Sophus::SE3d(Tw2c1r.topLeftCorner<3,3>(), Tw2c1r.col(3));
    frame_poses[2]= Sophus::SE3d(Tw2c2.topLeftCorner<3,3>(), Tw2c2.col(3));
    frame_poses[3]= Sophus::SE3d(Tw2c2r.topLeftCorner<3,3>(), Tw2c2r.col(3));

    Eigen::Matrix<int,8,1> rejectHisto=Eigen::Matrix<int,8,1>::Zero();//for debug

    for(auto itQM= vQuadMatches.begin(), itQMend= vQuadMatches.end();
        itQM!=itQMend; ++itQM)
    {
        const p_match& qMatch= *itQM;
        if(mpLastKeyFrame->mvpMapPoints[qMatch.i1p]!=NULL ||
                pCurrFrame->mvpMapPoints[qMatch.i1c]!=NULL)
            continue;
        cv::KeyPoint kpUn=mpLastKeyFrame->mvKeysUn[qMatch.i1p];
        int posX, posY;
        if(pFG->IsPointEligible(kpUn, posX, posY))
        {
            const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[qMatch.i1p];
            const cv::KeyPoint &kp2 = mpLastKeyFrame->mvRightKeysUn[qMatch.i1p];
            const cv::KeyPoint &kp3 = pCurrFrame->mvKeysUn[qMatch.i1c];
            const cv::KeyPoint &kp4 = pCurrFrame->mvRightKeysUn[qMatch.i1c];
            // Check parallax between left and right rays
            Vector3d xn1((kp1.pt.x-mpLastKeyFrame->cam_->cx())/mpLastKeyFrame->cam_->fx(),
                         (kp1.pt.y-mpLastKeyFrame->cam_->cy())/mpLastKeyFrame->cam_->fy(), 1.0 ),
                    ray1(xn1);

            Vector3d xn3((kp3.pt.x-pCurrFrame->cam_->cx())/pCurrFrame->cam_->fx(),
                         (kp3.pt.y-pCurrFrame->cam_->cy())/pCurrFrame->cam_->fy(), 1.0 );

            Vector3d ray3 = Rw2c1*Rw2c2.transpose()*xn3;
            float cosParallaxRays = ray1.dot(ray3)/(ray1.norm()*ray3.norm());

            if((cosParallaxRays<0 || cosParallaxRays> Config::triangMaxCosRays())
                    && (kp1.pt.x -kp2.pt.x< Config::triangMinDisp()))//parallax
            {
                ++rejectHisto[1];
                    continue;
            }
            // Linear Triangulation Method
            Vector3d xn2((kp2.pt.x-mpLastKeyFrame->right_cam_->cx())/mpLastKeyFrame->right_cam_->fx(),
                         (kp2.pt.y-mpLastKeyFrame->right_cam_->cy())/mpLastKeyFrame->right_cam_->fy(), 1.0 );

            Vector3d xn4((kp4.pt.x-pCurrFrame->right_cam_->cx())/pCurrFrame->right_cam_->fx(),
                         (kp4.pt.y-pCurrFrame->right_cam_->cy())/pCurrFrame->right_cam_->fy(), 1.0 );

            Eigen::Matrix<double,8,4> A;
            A.row(0) = xn1(0)*Tw2c1.row(2)-Tw2c1.row(0);
            A.row(1) = xn1(1)*Tw2c1.row(2)-Tw2c1.row(1);
            A.row(2) = xn2(0)*Tw2c1r.row(2)-Tw2c1r.row(0);
            A.row(3) = xn2(1)*Tw2c1r.row(2)-Tw2c1r.row(1);
            A.row(4) = xn3(0)*Tw2c2.row(2)-Tw2c2.row(0);
            A.row(5) = xn3(1)*Tw2c2.row(2)-Tw2c2.row(1);
            A.row(6) = xn4(0)*Tw2c2r.row(2)-Tw2c2r.row(0);
            A.row(7) = xn4(1)*Tw2c2r.row(2)-Tw2c2r.row(1);
            cv::Mat Aprime, w,u,vt;
            cv::eigen2cv(A, Aprime);
            cv::SVD::compute(Aprime,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

            cv::Mat x3D = vt.row(3).t();

            if(x3D.at<double>(3)==0)
            {
                 ++rejectHisto[2];
                continue;
            }

            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
            Eigen::Vector3d x3Dt;
            cv::cv2eigen (x3D, x3Dt);

       /*     obs[0]= xn1; obs[1]= xn2; obs[2]= xn3; obs[3]= xn4;
            double pdop=0;
            MapPoint::optimize(obs,frame_poses,  x3Dt, pdop,mpCurrentFrame->cam_, 5);
            if(pdop> Config::PDOPThresh()){
                 ++rejectHisto[3];
                continue;
            }*/
            //Check triangulation in front of cameras
            float z1 = Rw2c1.row(2)*x3Dt+ twinc1(2);
            if(z1<=0)
            {
                 ++rejectHisto[4];
                continue;
            }
            float z2 = Rw2c2.row(2)*x3Dt+twinc2(2);
            if(z2<=0)
            {
                 ++rejectHisto[4];
                continue;
            }

            //Check reprojection error in first keyframe
            float sigmaSquare1 = mpLastKeyFrame->GetSigma2(kp1.octave);
            float x1 = Rw2c1.row(0)*x3Dt+twinc1(0);
            float y1 = Rw2c1.row(1)*x3Dt+twinc1(1);
            float invz1 = 1.0/z1;
            float u1 = mpLastKeyFrame->cam_->fx()*x1*invz1+mpLastKeyFrame->cam_->cx();
            float v1 = mpLastKeyFrame->cam_->fy()*y1*invz1+mpLastKeyFrame->cam_->cy();
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>Config::reprojThresh2()*sigmaSquare1)
            {
                 ++rejectHisto[5];
                continue;
            }

            //Check reprojection error in second frame
            float sigmaSquare2 = pCurrFrame->GetSigma2(kp3.octave);
            float x2 = Rw2c2.row(0)*x3Dt+twinc2(0);
            float y2 = Rw2c2.row(1)*x3Dt+twinc2(1);
            float invz2 = 1.0/z2;
            float u2 = pCurrFrame->cam_->fx()*x2*invz2+pCurrFrame->cam_->cx();
            float v2 = pCurrFrame->cam_->fy()*y2*invz2+pCurrFrame->cam_->cy();
            float errX2 = u2 - kp3.pt.x;
            float errY2 = v2 - kp3.pt.y;
            if((errX2*errX2+errY2*errY2)>Config::reprojThresh2()*sigmaSquare2)
            {
                 ++rejectHisto[5];
                continue;
            }

            //Check scale consistency
            Eigen::Vector3d normal1 = x3Dt-Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3d normal2 = x3Dt-Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
            {
                 ++rejectHisto[6];
                continue;
            }

            float ratioDist = dist1/dist2;
            if(ratioDist*ratioFactor<1.f || ratioDist>ratioFactor)
            {
                 ++rejectHisto[7];
                continue;
            }

            // Triangulation is succesful
            MapPoint* pMP = new MapPoint(x3Dt,mpLastKeyFrame, qMatch.i1p, mpMap);

            mpLastKeyFrame->mvpMapPoints[qMatch.i1p] = pMP;
            mpCurrentFrame->mvpMapPoints[qMatch.i1c] = pMP;
            pMP->UpdateNormalAndDepth();
            pMP->ComputeDistinctiveDescriptors();

            mpMap->AddMapPoint(pMP);
            pFG->AddMapPoint(posX, posY, qMatch.i1p);
            ++numNewPoints;
        }
        else
            ++rejectHisto[0];
    }
    SLAM_DEBUG_STREAM("created new points from quad matches:"<<numNewPoints<<" "<<vQuadMatches.size());
    if(numNewPoints< 6)
        SLAM_DEBUG_STREAM("Rejected quad matches for fullCell, cosRay, SVD0, PDOP, negZ, reproj, distZero, distRatio"<< rejectHisto.transpose());
}
// for now, we only check left image to get matches
// Triangulate new MapPoints between the penultimate keyframe, pKF2, and the last keyframe
// TODO: map points can be created between the last keyframe and other keyframes/frames
// for now, map points are created only between last keyframe and its preceding keyframe
// a grid is used to ensure uniform distribution of map points observed in current frame,
// alternatively, a quad tree as in scavislam by Strasdat or rslam by Mei may be used.
void Tracking::CreateNewMapPoints(KeyFrame* pKF2, KeyFrame* pCurrentKeyFrame)
{
    int counter=0;
    ORBmatcher matcher(0.6,false);

    Sophus::SE3d proxy= pCurrentKeyFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c1=proxy.matrix3x4();
    Eigen::Matrix3d Rw2c1= Tw2c1.topLeftCorner<3,3>();
    Eigen::Vector3d twinc1= Tw2c1.col(3);

    Eigen::Vector3d Ow1 = pCurrentKeyFrame->GetCameraCenter();
    const float fx1 = pCurrentKeyFrame->cam_->fx();
    const float fy1 = pCurrentKeyFrame->cam_->fy();
    const float cx1 = pCurrentKeyFrame->cam_->cx();
    const float cy1 = pCurrentKeyFrame->cam_->cy();
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;
    const float ratioFactor = 1.5f*pCurrentKeyFrame->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    // Check first that baseline is not too short
    // Small translation errors for short baseline keyframes make scale to diverge
    Eigen::Vector3d Ow2 = pKF2->GetCameraCenter();
    Eigen::Vector3d vBaseline = Ow2-Ow1;
    const float baseline = vBaseline.norm();
    const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);

    assert(baseline/medianDepthKF2>=0.002);//ratio between Baseline and Depth

    // Compute Fundamental Matrix
    Eigen::Matrix3d F12 = ComputeF12(pCurrentKeyFrame,pKF2);

    // Search matches that fulfil epipolar constraint
    vector<cv::KeyPoint> vMatchedKeysUn1;
    vector<cv::KeyPoint> vMatchedKeysUn2;
    vector<pair<size_t,size_t> > vMatchedIndices;
    matcher.SearchForTriangulation(pCurrentKeyFrame,pKF2,F12,vMatchedKeysUn1,vMatchedKeysUn2,vMatchedIndices);

    proxy= pKF2->GetPose();
    Eigen::Matrix<double,3,4> Tw2c2=proxy.matrix3x4();
    Eigen::Matrix3d Rw2c2= Tw2c2.topLeftCorner<3,3>();
    Eigen::Vector3d twinc2=Tw2c2.col(3);

    const float fx2 = pKF2->cam_->fx();
    const float fy2 = pKF2->cam_->fy();
    const float cx2 = pKF2->cam_->cx();
    const float cy2 = pKF2->cam_->cy();
    const float invfx2 = 1.0f/fx2;
    const float invfy2 = 1.0f/fy2;
    //use a grid to control point initialization
    //mpLastKeyFrame is the new keyframe
    pCurrentKeyFrame->mpFG= new FeatureGrid(30,mnFeatures, pCurrentKeyFrame->mnMaxX,
                                           pCurrentKeyFrame->mnMinX, pCurrentKeyFrame->mnMaxY, pCurrentKeyFrame->mnMinY);
    FeatureGrid *pFG= pCurrentKeyFrame->mpFG;
    pCurrentKeyFrame->setExistingFeatures(*pFG);

    // Triangulate each match based on stereo observations
    for(size_t ikp=0, iendkp=vMatchedKeysUn1.size(); ikp<iendkp; ++ikp)
    {
        const int idx1 = vMatchedIndices[ikp].first; // indices of features in current keyframe
        const int idx2 = vMatchedIndices[ikp].second;// indices of features in the other keyframe
        const cv::KeyPoint &kp1 = vMatchedKeysUn1[ikp];
        int posX, posY;
        if(pFG->IsPointEligible(kp1, posX, posY))
        {
            const cv::KeyPoint &kp2 = vMatchedKeysUn2[ikp];

            // Check parallax between rays
            Eigen::Vector3d xn1((kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0 );
            Eigen::Vector3d ray1 = Rw2c1.transpose()*xn1;
            Eigen::Vector3d xn2((kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0 );
            Eigen::Vector3d ray2 = Rw2c2.transpose()*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm()*ray2.norm());
            if(cosParallaxRays<0 || cosParallaxRays>Config::triangMaxCosRays())
                continue;
            // Linear Triangulation Method
            Eigen::Matrix<double,4,4> A;
            A.row(0) = xn1(0)*Tw2c1.row(2)-Tw2c1.row(0);
            A.row(1) = xn1(1)*Tw2c1.row(2)-Tw2c1.row(1);
            A.row(2) = xn2(0)*Tw2c2.row(2)-Tw2c2.row(0);
            A.row(3) = xn2(1)*Tw2c2.row(2)-Tw2c2.row(1);

            cv::Mat Aprime, w,u,vt;
            cv::eigen2cv(A, Aprime);
            cv::SVD::compute(Aprime,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

            cv::Mat x3D = vt.row(3).t();

            if(x3D.at<double>(3)==0)
                continue;

            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
            Eigen::Vector3d x3Dt;
            cv::cv2eigen( x3D, x3Dt);

            //Check triangulation in front of cameras
            float z1 = Rw2c1.row(2)*x3Dt+twinc1(2);
            if(z1<=0)
                continue;

            float z2 = Rw2c2.row(2)*x3Dt+twinc2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            float sigmaSquare1 = pCurrentKeyFrame->GetSigma2(kp1.octave);
            float x1 = Rw2c1.row(0)*x3Dt+twinc1(0);
            float y1 = Rw2c1.row(1)*x3Dt+twinc1(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1+cx1;
            float v1 = fy1*y1*invz1+cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>Config::reprojThresh2()*sigmaSquare1)
                continue;

            //Check reprojection error in second keyframe
            float sigmaSquare2 = pKF2->GetSigma2(kp2.octave);
            float x2 = Rw2c2.row(0)*x3Dt+twinc2(0);
            float y2 = Rw2c2.row(1)*x3Dt+twinc2(1);
            float invz2 = 1.0/z2;
            float u2 = fx2*x2*invz2+cx2;
            float v2 = fy2*y2*invz2+cy2;
            float errX2 = u2 - kp2.pt.x;
            float errY2 = v2 - kp2.pt.y;
            if((errX2*errX2+errY2*errY2)>Config::reprojThresh2()*sigmaSquare2)
                continue;

            //Check scale consistency
            Eigen::Vector3d normal1 = x3Dt-Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3d normal2 = x3Dt-Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            float ratioDist = dist1/dist2;
            float ratioOctave = pCurrentKeyFrame->GetScaleFactor(kp1.octave)/pKF2->GetScaleFactor(kp2.octave);
            if(ratioDist*2.f<ratioOctave || ratioDist>ratioOctave*2.f)
                continue;

            // Triangulation is succesful
            MapPoint* pMP = new MapPoint(x3Dt,pCurrentKeyFrame, idx1, mpMap);

            pMP->AddObservation(pKF2, idx2);

            pCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);
            //Fill Current Frame structure
            mpCurrentFrame->mvpMapPoints[idx1] = pMP;

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
            mpMap->AddMapPoint(pMP);
            pFG->AddMapPoint(posX, posY, idx1);
            ++counter;
        }
    }
    cout<< "created new points in cur KF "<<counter<<" out of matches:"<<vMatchedKeysUn1.size()<<endl;
}

//using semi direct visual odometry for stereo cameras
// for right image, we only do map point reprojection
// assume exactly two stereo cameras, scale factor is 2 in building pyramid
void  Tracking::ProcessFrameSVO(cv::Mat &im, cv::Mat& right_im, double timeStampSec,                               
                                const std::vector<Eigen::Matrix<double, 7,1> >& imu_measurements,
                                const Sophus::SE3d *pred_Tr_delta, const Eigen::Matrix<double, 9,1> sb)
{
    /*    Sophus::SE3d Tcp =(pred_Tr_delta==NULL? Sophus::SE3d(): (*pred_Tr_delta)); // current left frame from previous frame

    // compute gravity direction in current camera frame
    Eigen::Vector3d ginc= ginw;
    if(mpLastFrame!=NULL && ginw.norm()>1e-6){
        Eigen::Matrix3d Rw2p= mpLastFrame->GetRotation();
        ginc= Tcp.rotationMatrix()*Rw2p*ginw;
    }
    // Depending on the state of the Tracker we perform different tasks
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    mpCurrentFrame=new Frame(im, timeStampSec, 0, mpORBextractor, mpORBVocabulary, cam_, ginc, sb);
    mpCurrentFrame->SetIMUObservations(imu_measurements);

    mLastProcessedState=mState;
    if(mState==NOT_INITIALIZED)
    {
        //extract uniform keypoints
        mpCurrentFrame->mpORBextractor->ComputeBlurredPyramid(mpCurrentFrame->img_pyr_, mpCurrentFrame->blurred_img_pyr_);
        // create keyframe and initializeSeeds
        mpCurrentFrame->SetPose( Sophus::SE3d());
        mState = INITIALIZING;
        mpLastKeyFrame=new KeyFrame(*mpCurrentFrame,mpMap,mpKeyFrameDB);
        mnLastKeyFrameId= mpLastKeyFrame->mnId;

        mpCurrentRightFrame.reset(new Frame(right_im, timeStampSec, 1, mpORBextractor, mpORBVocabulary, right_cam_, mTl2r.rotationMatrix()*ginc, NULL));
        mpCurrentRightFrame->SetPose(mTl2r);

        std::vector<cv::KeyPoint> vNewRightKeys=
                mpLocalMapper->initializeSeeds(mpLastKeyFrame, mpCurrentRightFrame.get(), mvIniMatches);

#if 0
        cv::Mat outimg;
        cv::drawKeypoints(im, mpLastKeyFrame->mvKeys,outimg);
        cv::imshow("candidate points of first frame", outimg);
        cv::waitKey();
#endif
        // Check if current frame has enough keypoints, otherwise reset initialization process
        if(mpLastKeyFrame->mvKeysUn.size()<=100)
        {
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }
        assert(mpCurrentRightFrame->mvKeys.size()==0 && mpLastKeyFrame->mvKeys.size()==mvIniMatches.size());
        mpCurrentRightFrame->mvKeys= vNewRightKeys;
        // keys(Un) without angle are computed, we need to compute angle, descriptors, and fts_
        mpCurrentRightFrame->CreateFtsFromKPs(true);
#if 0
        int nmatches= mvIniMatches.size();
        cv::Mat drawImg3;
        vector<cv::DMatch> matches122_cv; matches122_cv.reserve(nmatches);
        for(size_t i=0; i<mvIniMatches.size(); ++i)
        {
            if(mvIniMatches[i]==-1) continue;
            cv::DMatch temp;
            temp.queryIdx= i;
            temp.trainIdx= mvIniMatches[i];
            temp.distance =1;
            matches122_cv.push_back(temp);
        }
        drawMatches(im, mpLastKeyFrame->mvKeys,
                    right_im, mpCurrentRightFrame->mvKeys, matches122_cv,
                    drawImg3, cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255));
        cv::imshow("matched", drawImg3);
        cv::waitKey();
#endif
        // Check if there are enough correspondences
        if(vNewRightKeys.size()<100)
        {
            mState = NOT_INITIALIZED;
            return;
        }

        mpReferenceKF= mpLastKeyFrame;
        delete mpLastFrame;
        mpLastFrame=new Frame(*mpLastKeyFrame);
        mpLastKeyFrame->ComputeBoW();
        mpMap->AddKeyFrame(mpLastKeyFrame);

        mvpTemporalFrames.push_back(mpLastKeyFrame);
        mvpTemporalFrames.push_back(new Frame(*mpCurrentRightFrame));
        mState = WORKING;
    }
    else
    {

    //    mpMap->point_candidates_.emptyTrash();
        // System is initialized. Track Frame.
        bool bOK(false);
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        // Huai: if loop closure is just finished, relocalize
        if(mState==WORKING && !RelocalisationRequested())
        {
            // Set initial pose use prior
            if(!Config::useDecayVelocityModel() || mpMap->KeyFramesInMap()<4 || mpCurrentFrame->mnId<mnLastRelocFrameId+4)
                mpCurrentFrame->SetPose(mpLastFrame->GetPose());
            else
            {
                mVelocity= Tcp;// use IMU prediction or stereo relative pose
                mpCurrentFrame->SetPose(mVelocity*mpLastFrame->GetPose());
            }
            // sparse image align for left image
            SLAM_START_TIMER("sparse_img_align");
            SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                                     30, SparseImgAlign::GaussNewton, false, false);
            size_t img_align_n_tracked = img_align.run(mpLastFrame, mpCurrentFrame);
            SLAM_STOP_TIMER("sparse_img_align");
            SLAM_LOG(img_align_n_tracked);
            SLAM_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

            // This is for visualization
            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
            // Update Local Map only for left image
            UpdateReferenceKeyFramesAndPoints();
            setCoreKfs(core_kfs_);

            // map reprojection & feature alignment for left frame
            SLAM_START_TIMER("reproject");
            reprojector_->reprojectMap(mpCurrentFrame, mvpLocalMapPoints);
            SLAM_STOP_TIMER("reproject");
            size_t repr_n_new_references = reprojector_->n_matches_;
            size_t repr_n_mps = reprojector_->n_trials_;
            SLAM_LOG2(repr_n_mps, repr_n_new_references);
            SLAM_DEBUG_STREAM("Reprojection left image:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);

            if(repr_n_new_references < Config::qualityMinFts())
            {
                SLAM_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");

                mpCurrentFrame->SetPose( mpLastFrame->GetPose()); // reset to avoid crazy pose jumps
                mState=LOST;
                if(mpMap->KeyFramesInMap()<=5)
                {
                    Reset();
                }
                return;
            }
            bOK=true;
            // pose optimization for left image
            SLAM_START_TIMER("pose_optimizer");
            size_t sfba_n_edges_final;
            double sfba_thresh, sfba_error_init, sfba_error_final;
            pose_optimizer::optimizeGaussNewton(
                        Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
                        mpCurrentFrame, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
            SLAM_STOP_TIMER("pose_optimizer");
            SLAM_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
            SLAM_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
            SLAM_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
            mnMatchesInliers= sfba_n_edges_final;
            if(sfba_n_edges_final < 20)
            {
                mState=LOST;
                if(mpMap->KeyFramesInMap()<=5)
                {
                    Reset();
                }
                return;
            }
            mpCurrentFrame->CreateKPsFromFts();
            // structure optimization with left image
            if((!mpLocalMapper->isStopped()) && (!mpLocalMapper->stopRequested()))
            {
                SLAM_START_TIMER("point_optimizer");
                optimizeStructure(mpCurrentFrame, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
                SLAM_STOP_TIMER("point_optimizer");
            }
            //process right image
            mpCurrentRightFrame.reset(new Frame(right_im, timeStampSec, 1, mpORBextractor, mpORBVocabulary, right_cam_, mTl2r.rotationMatrix()*ginc, NULL));
            mpCurrentRightFrame->SetPose(mTl2r*mpCurrentFrame->GetPose());
            // map reprojection & feature alignment for right frame
            SLAM_START_TIMER("reproject");
            reprojector_->reprojectMap(mpCurrentRightFrame, mvpLocalMapPoints);
            SLAM_STOP_TIMER("reproject");
            repr_n_new_references = reprojector_->n_matches_;
            repr_n_mps = reprojector_->n_trials_;
            SLAM_LOG2(repr_n_mps, repr_n_new_references);
            SLAM_DEBUG_STREAM("Right reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);

            mpCurrentRightFrame->CreateKPsFromFts();
            if((!mpLocalMapper->isStopped()) && (!mpLocalMapper->stopRequested()))
            {
                int nBad = LocalOptimizeSVO();
                mnMatchesInliers= repr_n_new_references-nBad;
                SLAM_DEBUG_STREAM("Inliers after LO:"<< mnMatchesInliers);

                // Decide if the tracking was successful
                // More restrictive if there was a relocalization recently
                if(mpCurrentFrame->mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
                    bOK= false;

                if(mnMatchesInliers<12)
                    bOK=false;
            }
            point_stats.updatePointStatistics(mpCurrentFrame);
        }
        else
        {
            // only use left image
            //TODO: use semi direct approach for relocalisation
            //extract uniform keypoints
            mpCurrentFrame->mpORBextractor->ComputeBlurredPyramid(mpCurrentFrame->img_pyr_, mpCurrentFrame->blurred_img_pyr_);

            (*(mpCurrentFrame->mpORBextractor))(mpCurrentFrame->mvKeys,mpCurrentFrame->img_pyr_,
                                                mpCurrentFrame->blurred_img_pyr_, mpCurrentFrame->mDescriptors, 20.0);

            // keys with angle and descriptors are computed, we only need to compute fts_
            mpCurrentFrame->CreateFtsFromKPs(false);
            bOK = Relocalisation();
        }

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
#ifdef SLAM_USE_ROS
            mpMapPublisher->SetCurrentCameraPose(mpCurrentFrame->mTcw);
#endif
            Frame * pLastFrame(NULL), *pLastRightFrame(NULL);
            bool bNeedMorePoints= NeedNewKeyFrameStereo();

            if(bNeedMorePoints){
                mpCurrentFrame->MakeKeyFrame_Rest();
                CreateNewKeyFrame();
                pLastFrame=mpLastKeyFrame;

                // add observations for points observed by the new keyframe
                size_t jack=0, zinc=0;
                for(auto it=mpCurrentFrame->mvpMapPoints.begin();
                    it!=mpCurrentFrame->mvpMapPoints.end(); ++it, ++jack){
                    if(*it){
                        (*it)->AddObservation(mpLastKeyFrame, jack);
                        if((*it)->GetType()==MapPoint::TYPE_CANDIDATE)
                            ++zinc;
                    }
                }
                cout<<"add observations for candiate points:"<< zinc<<endl;
                //we also update reference map points which are used in double window optimizer
                mpMap->point_candidates_.addCandidatePointToFrame(mpLastKeyFrame);

                // optional bundle adjustment
                if(Config::lobaNumIter() > 0 && mvpLocalMapPoints.size())
                {
                    SLAM_START_TIMER("local_ba");
                    core_kfs_.push_back(mpLastKeyFrame);
                    size_t loba_n_erredges_init, loba_n_erredges_fin;
                    double loba_err_init, loba_err_fin;
                    ba::localBA(mpLastKeyFrame, &core_kfs_,mvpLocalMapPoints, mpMap,
                                loba_n_erredges_init, loba_n_erredges_fin,
                                loba_err_init, loba_err_fin);
                    SLAM_STOP_TIMER("local_ba");
                    SLAM_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
                    SLAM_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                                                                                                                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
                }
                pLastRightFrame= new Frame(*mpCurrentRightFrame);
                mpLocalMapper->addFrame(pLastRightFrame);
                // init new depth-filters
                double depth_mean, depth_min;
                getSceneDepth(*mpLastKeyFrame, depth_mean, depth_min);
                mpLocalMapper->addKeyframe(mpLastKeyFrame, depth_mean, 0.5*depth_min);

            }
            else{
                pLastFrame= new Frame(*mpCurrentFrame);
                pLastRightFrame= new Frame(*mpCurrentRightFrame);
            }
            if(mvpTemporalFrames.size()>=2){
                Frame * pTempF=*(++mvpTemporalFrames.rbegin());
                assert( pTempF->cam_id_ ==0);
                pLastFrame->SetPrevNextFrame( pTempF);
            }
            mvpTemporalFrames.push_back(pLastFrame);
            mvpTemporalFrames.push_back(pLastRightFrame);
        }

        if(bOK)
            mState = WORKING;
        else
            mState=LOST;

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }      
        delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }*/
    // Update drawer
    mpFramePublisher->Update(this, im);
#ifdef SLAM_USE_ROS
    Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
    Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
    tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                    Rwc(1,0),Rwc(1,1),Rwc(1,2),
                    Rwc(2,0),Rwc(2,1),Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));
    tf::Transform tfTcw(M,V);
    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World", "ORBSLAM_DWO/Camera"));
#endif
}
// im is left image at frame k+1, imu_measurements from k to k+1,
// sb(speed of imu sensor in world frame and bias) are predicted values
// use only ORB features and stereo matches based on ORB, drawback: few matches
// N.B. different to ProcessFrame, the current frame is added as a keyframe if not enough features are tracked,
// and the last frame is included in the temporal window
void  Tracking::ProcessFrameMono(cv::Mat &im, double timeStampSec,
                                 const std::vector<Eigen::Matrix<double, 7,1> >& imu_measurements,
                                 const Sophus::SE3d *pred_Tr_delta, const Eigen::Matrix<double, 9,1> sb)
{
    Sophus::SE3d Tcp =(pred_Tr_delta==NULL? Sophus::SE3d(): (*pred_Tr_delta)); // current frame from previous frame

    // compute gravity direction in current camera frame
    Eigen::Vector3d ginc= ginw;
    if(mpLastFrame!=NULL && (ginw.norm()>1e-6)){
        Eigen::Matrix3d Rw2p= mpLastFrame->GetRotation(); // world to previous left frame
        ginc= Tcp.rotationMatrix()*Rw2p*ginw;
    }
    //compute ORB descriptors
    if(mState==WORKING || mState==LOST)
        mpCurrentFrame=new Frame(im,timeStampSec,mpORBextractor,mpORBVocabulary, cam_, ginc);
    else
        mpCurrentFrame=new Frame(im,timeStampSec,mpIniORBextractor,mpORBVocabulary,cam_, ginc);

    mpCurrentFrame->SetIMUObservations(imu_measurements);

    // Depending on the state of the Tracker we perform different tasks
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    mLastProcessedState=mState;
    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        // Check if current frame has enough keypoints, otherwise reset initialization process
        if(mpCurrentFrame->mvKeysUn.size()<=100)
        {
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(*mpInitialFrame, *mpCurrentFrame,
                                                       mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            mState = NOT_INITIALIZED;
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(*mpCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;++i)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }
            double scale=0;
            if(mbUseIMUData)
                scale= Tcp.inverse().translation().norm();
            CreateInitialMap(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw), scale);
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK(false);
        bool bForcedReloc = RelocalisationRequested();
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        // Huai: if loop closure is just finished, relocalize
        if(mState==WORKING && !bForcedReloc)
        {
            if(!Config::useDecayVelocityModel() || mpMap->KeyFramesInMap()<4 || mpCurrentFrame->mnId<mnLastRelocFrameId+2)
                bOK = TrackPreviousFrame();
            else
            {
                if(pred_Tr_delta!=NULL)
                    mVelocity= Tcp;// use IMU or motion model prediction
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackPreviousFrame();
            }
        }
        else
        {            
            bOK = Relocalisation();
            SLAM_DEBUG_STREAM("Relocalisation in tracking thread: "<< bOK);
            //update poses, speed and bias of frames in temporal window and last frame
            if(bForcedReloc)//forced relocalisation
            {
                Sophus::Sim3d Snew2old= GetSnew2old();
                if(!mpLastFrame->isKeyFrame())
                    mpLastFrame->SetPose( Converter::toSE3d( Converter::toSim3d(mpLastFrame->mTcw)*Snew2old));
                for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                {
                    if((*it)->isKeyFrame())//a keyframe's pose should have been optimized and updated in LC
                        continue;
                    Sophus::Sim3d tempSw2c = Converter::toSim3d((*it)->mTcw)*Snew2old;
                    (*it)->SetPose( Converter::toSE3d(tempSw2c));
                }
                if(mbUseIMUData)
                {
                    mpLastFrame->speed_bias.head<3>() = Snew2old.rotationMatrix().inverse()*mpLastFrame->speed_bias.head<3>();
                    mpCurrentFrame->speed_bias.head<3>() = Snew2old.rotationMatrix().inverse()*mpCurrentFrame->speed_bias.head<3>();
                    for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                    {//note we donot update first estimate here
                        (*it)->speed_bias.head<3>() = Snew2old.rotationMatrix().inverse()*(*it)->speed_bias.head<3>();
                    }
                }
                // update poses of the current frame if needed
                if(!bOK){
                    Tcp.translation()/= Snew2old.scale();
                    mpCurrentFrame->SetPose(Tcp*mpLastFrame->mTcw);
                }
                SLAM_DEBUG_STREAM("last frame and curr frame id:"<< mpLastFrame->mnId <<" "<<mpCurrentFrame->mnId);
            }
        }

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        SLAM_START_TIMER("local_optimize");
        if(bOK)
            bOK = TrackLocalMapDWO();
        SLAM_STOP_TIMER("local_optimize");
        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
#ifdef SLAM_USE_ROS
            mpMapPublisher->SetCurrentCameraPose(mpCurrentFrame->mTcw);
#endif
            Frame * pLastFrame(NULL); //the frame pointer to be put in temporal window
            mpCurrentFrame->updatePointStatistics(&point_stats);
            bool bNeedMorePoints=NeedNewKeyFrameStereo();

            if(bNeedMorePoints){
                KeyFrame* pKF = new KeyFrame(*mpCurrentFrame,mpMap,mpKeyFrameDB);
                pKF->SetNotErase(DoubleWindowKF);//because we want to ensure that keyframes in temporal window are not bad
                pKF->ComputeBoW();
                mnLastKeyFrameId = pKF->mnId;
                KeyFrame *penultimateKF(mpLastKeyFrame);
                mpLastKeyFrame = pKF;
                pLastFrame = pKF;

                SLAM_START_TIMER("triangulate_new_mappoint");
                size_t chris=0;
                for(auto it= mpLastKeyFrame->mvpMapPoints.begin(), ite= mpLastKeyFrame->mvpMapPoints.end(); it!=ite; ++it, ++chris)
                {
                    MapPoint* pMP = *it;
                    if(pMP&&(!pMP->isBad()))
                    {
                        pMP->AddObservation(mpLastKeyFrame, chris);
                    }
                }

                CreateNewMapPoints(penultimateKF, mpLastKeyFrame);
                SLAM_STOP_TIMER("triangulate_new_mappoint");
                mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
            }else
                 pLastFrame = new Frame(*mpCurrentFrame);

            if(mvpTemporalFrames.size())
            {
                pLastFrame->SetPrevNextFrame( mvpTemporalFrames.back());
            }
            mvpTemporalFrames.push_back(pLastFrame);
            //update temporal window
            if(mvpTemporalFrames.size() <= mnTemporalWinSize)
            {
                //         assert(mvpOldLocalKeyFrames.size()==0 && mvpLocalKeyFrames.size()==0);
            }
            else  if(mnTemporalWinSize)
            {
                //remove observations for the frame that slips out
                Frame *pFrontFrame=mvpTemporalFrames.front();
                if(pFrontFrame->isKeyFrame()){
                    ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                }
                else
                {
                    delete pFrontFrame;
                }
                mvpTemporalFrames.pop_front();
                mvpTemporalFrames.front()->SetFirstEstimate();
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }else
            {
                while(mvpTemporalFrames.size()>mnTemporalWinSize){
                    //remove observations for the frame that slips out
                    Frame *pFrontFrame=mvpTemporalFrames.front();
                    if(pFrontFrame->isKeyFrame()){
                        ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                        pFrontFrame->SetFirstEstimate();
                    }
                    else
                    {
                        delete pFrontFrame;
                    }
                    mvpTemporalFrames.pop_front();
                }
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }
            mState = WORKING;
        }
        else{
            mState=LOST;
            // Reset if the camera get lost soon after initialization
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }
        if(!mpLastFrame->isKeyFrame())
            delete mpLastFrame;
        if(mpCurrentFrame->mnId == mnLastKeyFrameId){
            delete mpCurrentFrame;
            mpCurrentFrame =mpLastKeyFrame;
            mpLastFrame = mpLastKeyFrame;
        }
        else
            mpLastFrame = mpCurrentFrame;
    }
    // Update drawer
    mpFramePublisher->Update(this, im);
#ifdef SLAM_USE_ROS
    Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
    Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
    tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                    Rwc(1,0),Rwc(1,1),Rwc(1,2),
                    Rwc(2,0),Rwc(2,1),Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));
    tf::Transform tfTcw(M,V);
    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World", "ORBSLAM_DWO/Camera"));
#endif
}

// im is left image at frame k+1, imu_measurements from k to k+1,
// sb(speed of imu sensor in world frame and bias) are predicted values
// similar to ProcessFrame, but it uses external saved feature tracks and
// incremental motion estimate by stereo SFM(qcv) to drive DWO
void  Tracking::ProcessFrameQCV(cv::Mat &im, cv::Mat &right_img, double timeStampSec,
                             const std::vector<Eigen::Matrix<double, 7,1> >& imu_measurements,
                             const Sophus::SE3d *pred_Tr_delta, const Eigen::Matrix<double, 9,1> sb)
{
    Sophus::SE3d Tcp =pred_Tr_delta==NULL? Sophus::SE3d(): (*pred_Tr_delta); // current frame from previous frame

    // get external visual odometry of qcv
    SLAM_START_TIMER("extract_quadmatches");

    mStereoSFM.nextFrame();
    vector<p_match> vQuadMatches;
    mStereoSFM.getQuadMatches(vQuadMatches);
    SLAM_START_TIMER("track_previous_frame");   
    if (mStereoSFM.mb_Tr_valid) {
        Tcp= mStereoSFM.getDeltaMotion();
    }
    else{
        if(mpCurrentFrame)// not the first frame
            mState= LOST;
        cerr<<"Stereo SFM odometry failed for images at time:"<<timeStampSec
           <<" of quad matches:"<<vQuadMatches.size()<<endl;
    }
    SLAM_STOP_TIMER("track_previous_frame");
    //stereo matching
    SLAM_START_TIMER("stereo_matching");    
    vector<p_match> vStereoMatches;
    mStereoSFM.getStereoMatches(vStereoMatches);

    SLAM_STOP_TIMER("stereo_matching");
    // compute gravity direction in current camera frame
    Eigen::Vector3d ginc=ginw;
    if(mpLastFrame!=NULL && (ginw.norm()>1e-6)){
        Eigen::Matrix3d Rw2p= mpLastFrame->GetRotation(); // world to previous left frame
        ginc= Tcp.rotationMatrix()*Rw2p*ginw;
    }
    SLAM_START_TIMER("create_frame");
    //compute ORB descriptors of vStereoMatches
    mpCurrentFrame=new Frame(im, timeStampSec, mStereoSFM.getNumDenseFeatures(),
                                   right_img, mStereoSFM.getNumDenseFeatures(),
                                   vStereoMatches, mpORBextractor, mpORBVocabulary, cam_, right_cam_,
                                   mTl2r, ginc, sb);

    // also test whether a quad match satisfy stereo matches
    remapQuadMatches(vQuadMatches,
                     mpCurrentFrame->viso2LeftId2StereoId,   mpCurrentFrame->viso2RightId2StereoId,
                     mpLastFrame->viso2LeftId2StereoId,   mpLastFrame->viso2RightId2StereoId);

#if 0
    //draw stereo matches
        int nmatches= vStereoMatches.size();
        cv::Mat drawImg;
        vector<cv::DMatch> matches; matches.reserve(nmatches);
        for(size_t i=0; i<vStereoMatches.size()/5; ++i)
        {
            cv::DMatch temp;
            temp.queryIdx= i*5;
            temp.trainIdx= i*5;
            temp.distance =1;
            matches.push_back(temp);
        }
        drawMatches(im, mpCurrentFrame->mvKeys,
                    right_img, mpCurrentFrame->mvRightKeys, matches,
                    drawImg, cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255));
        cv::imshow("stereo matches", drawImg);
        cv::waitKey();
#endif
    mpCurrentFrame->SetIMUObservations(imu_measurements);

    SLAM_STOP_TIMER("create_frame");
    // Depending on the state of the Tracker we perform different tasks
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    mLastProcessedState=mState;
    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        // Check if current frame has enough matches with its previous frame, otherwise reset initialization process
        if(vQuadMatches.size()<=20)
        {
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }
        CreateInitialMapStereo(Tcp, vQuadMatches);
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK(false);
        bool bForcedReloc= RelocalisationRequested();
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        // Huai: if loop closure is just finished, relocalize
        if(mState==WORKING && !bForcedReloc)
        {
            bOK = TrackPreviousFrame(Tcp, vQuadMatches);
        }
        else
        {            
            bOK = Relocalisation();
            SLAM_DEBUG_STREAM("Relocalisation in tracking thread: "<< bOK);
            //update poses, speed and bias of frames in temporal window and last frame
            if(bForcedReloc)//forced relocalisation
            {
                Sophus::SE3d Tnew2old= GetTnew2old();
                mpLastFrame->SetPose(mpLastFrame->mTcw*Tnew2old);

                 for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                 {
                     if((*it)->isKeyFrame())//a keyframe's pose should have been optimized and updated in LC
                            continue;
                     (*it)->SetPose((*it)->mTcw*Tnew2old);
                 }
                 if(mbUseIMUData)
                 {
                     mpLastFrame->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*mpLastFrame->speed_bias.head<3>();
                     mpCurrentFrame->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*mpCurrentFrame->speed_bias.head<3>();
                     for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                     {//note we donot update first estimate here
                         (*it)->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*(*it)->speed_bias.head<3>();
                     }
                 }
                 // update poses of the current frame if needed
                 if(!bOK)
                    mpCurrentFrame->SetPose(Tcp*mpLastFrame->mTcw);
                 SLAM_DEBUG_STREAM("last frame and curr frame id:"<< mpLastFrame->mnId <<" "<<mpCurrentFrame->mnId);
            }
        }
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        SLAM_START_TIMER("local_optimize");
        if(bOK)
            bOK = TrackLocalMapDWO();
        SLAM_STOP_TIMER("local_optimize");
        if(mStereoSFM.mb_Tr_valid){
#ifdef SLAM_USE_ROS
            mpMapPublisher->SetCurrentCameraPose(mpCurrentFrame->mTcw);
#endif
            Frame * pLastFrame(NULL);
            mpCurrentFrame->updatePointStatistics(&point_stats);
            bool bNeedMorePoints= NeedNewKeyFrameStereo();
            if(!bNeedMorePoints){
                if(mpLastFrame->mnId==1){
                    mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
                    pLastFrame= mpLastKeyFrame;
                }
                else
                    pLastFrame= new Frame(*mpLastFrame);
            }
            else{
                assert(mpLastFrame->mnId>1);//because we added second frame as keyframe
                KeyFrame* pKF = new KeyFrame(*mpLastFrame,mpMap,mpKeyFrameDB);
                pKF->SetNotErase(DoubleWindowKF);//because we want to ensure that keyframes in temporal window are not bad
                mnLastKeyFrameId = mpLastFrame->mnId;
                mpLastKeyFrame = pKF;
                pLastFrame=pKF;

                SLAM_START_TIMER("triangulate_new_mappoint");
                size_t chris=0;
                for(auto it= mpLastKeyFrame->mvpMapPoints.begin(), ite= mpLastKeyFrame->mvpMapPoints.end(); it!=ite; ++it, ++chris)
                {
                    MapPoint* pMP = *it;
                    if(pMP&&(!pMP->isBad()))
                    {
                        pMP->AddObservation(mpLastKeyFrame, chris);
                        pMP->AddObservation(mpLastKeyFrame, chris, false);
                    }
                }
                CreateNewMapPoints(vQuadMatches);
                SLAM_STOP_TIMER("triangulate_new_mappoint");
                mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
            }

            if(mvpTemporalFrames.size())
            {
                pLastFrame->SetPrevNextFrame( mvpTemporalFrames.back());
            }
            mvpTemporalFrames.push_back(pLastFrame);
            //update temporal window
            if(mvpTemporalFrames.size() <= mnTemporalWinSize)
            {
                //         assert(mvpOldLocalKeyFrames.size()==0 && mvpLocalKeyFrames.size()==0);
            }
            else if(mnTemporalWinSize)
            {
                //remove observations for the frame that slips out
                Frame *pFrontFrame=mvpTemporalFrames.front();
                if(pFrontFrame->isKeyFrame()){
                    ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                }
                else
                {
                    delete pFrontFrame;
                }
                mvpTemporalFrames.pop_front();
                mvpTemporalFrames.front()->SetFirstEstimate();
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }
            else
            {
                while(mvpTemporalFrames.size()>mnTemporalWinSize){
                //remove observations for the frame that slips out
                Frame *pFrontFrame=mvpTemporalFrames.front();
                if(pFrontFrame->isKeyFrame()){
                    ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                    pFrontFrame->SetFirstEstimate();
                }
                else
                {
                    delete pFrontFrame;
                }
                mvpTemporalFrames.pop_front();
                }
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }
            mState = WORKING;
        }
        else{
            mState=LOST;
        // Reset if the camera get lost soon after initialization
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }

        if(mpLastFrame->mnId!=1)
            delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }   
    // Update drawer
    mpFramePublisher->Update(this, im);
#ifdef SLAM_USE_ROS
    Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
    Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
    tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                    Rwc(1,0),Rwc(1,1),Rwc(1,2),
                    Rwc(2,0),Rwc(2,1),Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));
    tf::Transform tfTcw(M,V);
    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World", "ORBSLAM_DWO/Camera"));
#endif
}
// im is left image at frame k+1, imu_measurements from k to k+1,
// sb(speed of imu sensor in world frame and bias) are predicted values

// for each current frame k, keypoints are created from quad matches between k-1 and k, and between k and k+1
// mvpTemporalFrames only contains frame up to previous frame, excluding it.
// The previous frame will be added as a keyframe, if there are not enough tracked mappoints in current frame.
// New map points are created using quad matches between the previous frame k-1 and current frame k
// Therefore, the last frame cannot be a keyframe and used in local mapping until the current frame is processed

void  Tracking::ProcessFrame(cv::Mat &im, cv::Mat &right_img, double timeStampSec,
                             const std::vector<Eigen::Matrix<double, 7,1> >& imu_measurements,
                             const Sophus::SE3d *pred_Tr_delta,const Eigen::Matrix<double, 9,1> sb)
{
    Sophus::SE3d Tcp =pred_Tr_delta==NULL? Sophus::SE3d(): (*pred_Tr_delta); // current frame from previous frame

    // compute visual odometry with libviso2
    int32_t dims[] = {im.cols,im.rows,im.cols};
    SLAM_START_TIMER("extract_quadmatches");
    // push back images, compute features
    mVisoStereo.matcher->pushBack(im.data,right_img.data,dims,false);

    // match features
    mVisoStereo.Tr_valid=false; //do we use stereo or IMU prior information for tracking?
    // CAUTION: Prior motion from IMU noisy data and stereo prior often leads to worse results.
    //I believe the reason is prior motion is not necessary in feature matching. E.g., StereoSFM did not use such motion prior to aid feature matching
    // but when features are matched to points in local map, Stereo PTAM used a prior motion.
    // On the other hand, Prior motion should be helpful in initializing pose optimization
    if (mVisoStereo.Tr_valid) mVisoStereo.matcher->matchFeatures(2,&mVisoStereo.Tr_delta);
    else          mVisoStereo.matcher->matchFeatures(2);

    vector<p_match> vQuadMatches = cropMatches( mVisoStereo.matcher->getMatches(), Config::cropROIXL(), Config::cropROIXR());
    libviso2::VisualOdometryStereo::parameters param=mVisoStereo.getParameters();
    mVisoStereo.matcher->bucketFeatures(param.bucket.max_features, param.bucket.bucket_width,param.bucket.bucket_height);
    vector<p_match> p_matched = cropMatches(mVisoStereo.matcher->getMatches(), Config::cropROIXL(), Config::cropROIXR());
    SLAM_STOP_TIMER("extract_quadmatches");

    SLAM_START_TIMER("track_previous_frame");
    vector<double> tr_delta = mVisoStereo.transformationMatrixToVector (Converter::toViso2Matrix(Tcp));
    const enum Optimizer {RANSAC_Geiger, RANSAC_5Point, ROBUST_Klein} approach=RANSAC_5Point;
    switch (approach){
    case RANSAC_Geiger:
        tr_delta= mVisoStereo.estimateMotion(p_matched, tr_delta);
        // on failure
        if (tr_delta.size()!=6)
            mVisoStereo.Tr_valid=false;
        else{
            // set transformation matrix (previous to current frame)
            mVisoStereo.Tr_delta = mVisoStereo.transformationVectorToMatrix(tr_delta);
            mVisoStereo.Tr_valid = true;
        }
        break;
    case RANSAC_5Point:
        mVisoStereo.estimateMotion5Point(p_matched);
        break;
    case ROBUST_Klein:
        mVisoStereo.estimateMotionKlein(p_matched, vector<vector<float> >());
        break;
    }

    if (mVisoStereo.Tr_valid) {
        mPose = mPose * libviso2::Matrix::inv(mVisoStereo.Tr_delta);     // update current pose
        Tcp= Converter::toSE3d(mVisoStereo.Tr_delta);
        // sieve inliers from quad matches, this does not matter much
//        vector<bool> vInliers;
//        mVisoStereo.getAllInlier(vQuadMatches, mVisoStereo.Tr_delta, vInliers);
//        //cout<<"Inliers out of quad matches :"<< std::accumulate(vInliers.begin(), vInliers.end(), 0)<<" of "<< vQuadMatches.size()<<endl;
//        //apply vInliers
//        size_t jack=0;
//        for(vector<p_match>::iterator vIt=vQuadMatches.begin(); vIt!=vQuadMatches.end(); ++vIt, ++jack)
//        {
//            if(!vInliers[jack])
//                vIt->i1c=-1;
//        }
//        assert(jack==vInliers.size());
    }
    else{
        if(mpCurrentFrame)// not the first frame
            mState= LOST;
        cerr<<"libviso2 odometry failed for images at time:"<<timeStampSec
           <<" of quad matches:"<<vQuadMatches.size()<<endl;
    }
    SLAM_STOP_TIMER("track_previous_frame");
    //stereo matching
    SLAM_START_TIMER("stereo_matching");
    mVisoStereo.matcher->matchFeatures(1);
    vector<p_match> vStereoMatches = cropMatches(mVisoStereo.matcher->getMatches(), Config::cropROIXL(), Config::cropROIXR());
    //    cout<<"stereo matches in image:"<< vStereoMatches.size() <<endl;
    // mVisoStereo.matcher->refineFeatures(vStereoMatches);
    SLAM_STOP_TIMER("stereo_matching");
    // compute gravity direction in current camera frame
    Eigen::Vector3d ginc=ginw;
    if(mpLastFrame!=NULL && (ginw.norm()>1e-6)){
        Eigen::Matrix3d Rw2p= mpLastFrame->GetRotation(); // world to previous left frame
        ginc= Tcp.rotationMatrix()*Rw2p*ginw;
    }
    SLAM_START_TIMER("create_frame");
    //compute ORB descriptors of vStereoMatches
    mpCurrentFrame=new Frame(im, timeStampSec, mVisoStereo.matcher->getNumDenseFeatures(true),
                                   right_img, mVisoStereo.matcher->getNumDenseFeatures(false),
                                   vStereoMatches, mpORBextractor, mpORBVocabulary, cam_, right_cam_,
                                   mTl2r, ginc, sb);

    // also test whether a quad match satisfy stereo matches
    remapQuadMatches(vQuadMatches,
                     mpCurrentFrame->viso2LeftId2StereoId,   mpCurrentFrame->viso2RightId2StereoId,
                     mpLastFrame->viso2LeftId2StereoId,   mpLastFrame->viso2RightId2StereoId);

    mpCurrentFrame->SetIMUObservations(imu_measurements);

    SLAM_STOP_TIMER("create_frame");
    // Depending on the state of the Tracker we perform different tasks
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    mLastProcessedState=mState;
    if(mState==NOT_INITIALIZED)
    {
        FirstInitialization();
    }
    else if(mState==INITIALIZING)
    {
        // Check if current frame has enough matches with its previous frame, otherwise reset initialization process
        if(vQuadMatches.size()<=20)
        {
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            mState = NOT_INITIALIZED;
            return;
        }
        CreateInitialMapStereo(Tcp, vQuadMatches);
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK(false);
        bool bForcedReloc =RelocalisationRequested();
        // Initial Camera Pose Estimation from Previous Frame (Motion Model or Coarse) or Relocalisation
        // Huai: if loop closure is just finished, relocalize
        if(mState==WORKING && !bForcedReloc)
        {
            bOK = TrackPreviousFrame(Tcp, vQuadMatches);
        }
        else
        {            
            bOK = Relocalisation();
            SLAM_DEBUG_STREAM("Relocalisation in tracking thread: "<< bOK);
            //update poses, speed and bias of frames in temporal window and last frame
            if(bForcedReloc)//forced relocalisation
            {
                Sophus::SE3d Tnew2old= GetTnew2old();
                mpLastFrame->SetPose(mpLastFrame->mTcw*Tnew2old);

                 for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                 {
                     if((*it)->isKeyFrame())//a keyframe's pose should have been optimized and updated in LC
                            continue;
                     (*it)->SetPose((*it)->mTcw*Tnew2old);
                 }
                 if(mbUseIMUData)
                 {
                     mpLastFrame->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*mpLastFrame->speed_bias.head<3>();
                     mpCurrentFrame->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*mpCurrentFrame->speed_bias.head<3>();
                     for(auto it= mvpTemporalFrames.begin(), ite= mvpTemporalFrames.end(); it!=ite; ++it)
                     {//note we donot update first estimate here
                         (*it)->speed_bias.head<3>() = Tnew2old.rotationMatrix().inverse()*(*it)->speed_bias.head<3>();
                     }
                 }
                 // update poses of the current frame if needed
                 if(!bOK)
                    mpCurrentFrame->SetPose(Tcp*mpLastFrame->mTcw);
                 SLAM_DEBUG_STREAM("last frame and curr frame id:"<< mpLastFrame->mnId <<" "<<mpCurrentFrame->mnId);
            }
        }
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        SLAM_START_TIMER("local_optimize");
        if(bOK)
            bOK = TrackLocalMapDWO();
        SLAM_STOP_TIMER("local_optimize");

        if(mVisoStereo.Tr_valid){
#ifdef SLAM_USE_ROS
            mpMapPublisher->SetCurrentCameraPose(mpCurrentFrame->mTcw);
#endif
            Frame * pLastFrame(NULL);
            mpCurrentFrame->updatePointStatistics(&point_stats);
            bool bNeedMorePoints= NeedNewKeyFrameStereo();
            if(!bNeedMorePoints){
                if(mpLastFrame->mnId==1){
                    mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
                    pLastFrame= mpLastKeyFrame;
                }
                else
                    pLastFrame= new Frame(*mpLastFrame);
            }
            else{
                assert(mpLastFrame->mnId>1);//because we added second frame as keyframe
                KeyFrame* pKF = new KeyFrame(*mpLastFrame,mpMap,mpKeyFrameDB);
                pKF->SetNotErase(DoubleWindowKF);//because we want to ensure that keyframes in temporal window are not bad
                mnLastKeyFrameId = mpLastFrame->mnId;
                mpLastKeyFrame = pKF;
                pLastFrame=pKF;

                SLAM_START_TIMER("triangulate_new_mappoint");
                size_t chris=0;
                for(auto it= mpLastKeyFrame->mvpMapPoints.begin(), ite= mpLastKeyFrame->mvpMapPoints.end(); it!=ite; ++it, ++chris)
                {
                    MapPoint* pMP = *it;
                    if(pMP&&(!pMP->isBad()))
                    {
                        pMP->AddObservation(mpLastKeyFrame, chris);
                        pMP->AddObservation(mpLastKeyFrame, chris, false);
                    }
                }
                CreateNewMapPoints(vQuadMatches);
                SLAM_STOP_TIMER("triangulate_new_mappoint");
                mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
            }

            if(mvpTemporalFrames.size())
            {
                pLastFrame->SetPrevNextFrame( mvpTemporalFrames.back());
            }
            mvpTemporalFrames.push_back(pLastFrame);
            //update temporal window
            if(mvpTemporalFrames.size() <= mnTemporalWinSize)
            {
                //         assert(mvpOldLocalKeyFrames.size()==0 && mvpLocalKeyFrames.size()==0);
            }
            else  if(mnTemporalWinSize)
            {
                //remove observations for the frame that slips out
                Frame *pFrontFrame=mvpTemporalFrames.front();
                if(pFrontFrame->isKeyFrame()){
                    ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                }
                else
                {
                    delete pFrontFrame;
                }
                mvpTemporalFrames.pop_front();
                mvpTemporalFrames.front()->SetFirstEstimate();
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }else
            {
                while(mvpTemporalFrames.size()>mnTemporalWinSize){
                //remove observations for the frame that slips out
                Frame *pFrontFrame=mvpTemporalFrames.front();
                if(pFrontFrame->isKeyFrame()){
                    ((KeyFrame*)pFrontFrame)->SetErase(DoubleWindowKF);
                    pFrontFrame->SetFirstEstimate();
                }
                else
                {
                    delete pFrontFrame;
                }
                mvpTemporalFrames.pop_front();
                }
                assert(mvpTemporalFrames.size()==mnTemporalWinSize);
            }
            mState = WORKING;
        }
        else{
            mState=LOST;
        // Reset if the camera get lost soon after initialization
            if(mpMap->KeyFramesInMap()<=5)
            {
                Reset();
                return;
            }
        }
        if(mpLastFrame->mnId!=1)
            delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }  
    // Update drawer
    mpFramePublisher->Update(this, im);
#ifdef SLAM_USE_ROS
    Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
    Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
    tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                    Rwc(1,0),Rwc(1,1),Rwc(1,2),
                    Rwc(2,0),Rwc(2,1),Rwc(2,2));
    tf::Vector3 V(twc(0), twc(1), twc(2));
    tf::Transform tfTcw(M,V);
    mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World", "ORBSLAM_DWO/Camera"));
#endif
}

void Tracking::FirstInitialization()
{    
    //We ensure a minimum ORB features to continue, otherwise discard frame
    if(mpCurrentFrame->mvKeysUn.size()>100)
    {
        mpCurrentFrame->SetPose( Sophus::SE3d());       
        mpInitialFrame= mpCurrentFrame;
        delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
        mvbPrevMatched.resize(mpInitialFrame->mvKeysUn.size());
        for(size_t i=0; i<mpInitialFrame->mvKeysUn.size(); i++)
            mvbPrevMatched[i]=mpInitialFrame->mvKeysUn[i].pt;

        if(mpInitializer)
            delete mpInitializer;

        mpInitializer =  new Initializer(*mpInitialFrame,1.0,200);
        mState = INITIALIZING;
    }
}
// not used for stereo processing case
void Tracking::Initialize()
{
    // Check if current frame has enough keypoints, otherwise reset initialization process
    if(mpCurrentFrame->mvKeysUn.size()<=100)
    {
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        mState = NOT_INITIALIZED;
        return;
    }

    // Find correspondences
    ORBmatcher matcher(0.9,true);
    int nmatches = matcher.SearchForInitialization(*mpInitialFrame,*mpCurrentFrame,mvbPrevMatched,mvIniMatches,100);

    // Check if there are enough correspondences
    if(nmatches<100)
    {
        mState = NOT_INITIALIZED;
        return;
    }

    cv::Mat Rcw; // Current Camera Rotation
    cv::Mat tcw; // Current Camera Translation
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if(mpInitializer->Initialize(*mpCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    {
        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        {
            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
            {
                mvIniMatches[i]=-1;
                nmatches--;
            }
        }

        CreateInitialMap(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw));
    }
}

//create initial map with two consecutive pairs of frames of the stereo sequence
// both frames become keyframes
void Tracking::CreateInitialMapStereo(const Sophus::SE3d &Tcw, const std::vector<p_match> &vQuadMatches)
{
    // Set Frame Poses
    mpCurrentFrame->SetPose(Tcw);

    // Create KeyFrames
    KeyFrame* pPrevFrame = new KeyFrame(*mpLastFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(*mpCurrentFrame,mpMap,mpKeyFrameDB);
    pPrevFrame->ComputeBoW();
    pKFcur->ComputeBoW();
    // Insert KFs in the map
    mpMap->AddKeyFrame(pPrevFrame);
    mpMap->AddKeyFrame(pKFcur);
    assert(pKFcur->mnFrameId==1);
    //use vMPGrid to control map point distribution and number
    FeatureGrid fg(30,mnFeatures, pKFcur->mnMaxX,
                   pKFcur->mnMinX, pKFcur->mnMaxY, pKFcur->mnMinY);
    Sophus::SE3d proxy=pKFcur->GetPose();
    Eigen::Matrix<double,3,4> Tw2c1= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c1= Tw2c1.topLeftCorner<3,3>();
    Eigen::Vector3d twinc1=Tw2c1.col(3);

    const float fx1 = pKFcur->cam_->fx();
    const float fy1 = pKFcur->cam_->fy();
    const float cx1 = pKFcur->cam_->cx();
    const float cy1 = pKFcur->cam_->cy();

    proxy= pPrevFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c2= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c2= Tw2c2.topLeftCorner<3,3>();
    Eigen::Vector3d twinc2=Tw2c2.col(3);

    const float fx2 = pPrevFrame->cam_->fx();
    const float fy2 = pPrevFrame->cam_->fy();
    const float cx2 = pPrevFrame->cam_->cx();
    const float cy2 = pPrevFrame->cam_->cy();

    Eigen::Matrix<double,3,4> Tw2c1r= pKFcur->GetPose(false).matrix3x4();
    Eigen::Matrix<double,3,4> Tw2c2r= pPrevFrame->GetPose(false).matrix3x4();

    std::vector< Eigen::Vector3d> obs(4);
    std::vector<Sophus::SE3d> frame_poses(4);
    frame_poses[0]= Sophus::SE3d(Tw2c1.topLeftCorner<3,3>(), Tw2c1.col(3));
    frame_poses[1]= Sophus::SE3d(Tw2c1r.topLeftCorner<3,3>(), Tw2c1r.col(3));
    frame_poses[2]= Sophus::SE3d(Tw2c2.topLeftCorner<3,3>(), Tw2c2.col(3));
    frame_poses[3]= Sophus::SE3d(Tw2c2r.topLeftCorner<3,3>(), Tw2c2r.col(3));

    const float ratioFactor = 1.5f*pKFcur->GetScaleFactor();
    Eigen::Vector3d Ow1 = pKFcur->GetCameraCenter();
    Eigen::Vector3d Ow2 = pPrevFrame->GetCameraCenter();
    for(auto itQM= vQuadMatches.begin(), itQMend= vQuadMatches.end(); itQM!=itQMend; ++itQM){
        const p_match& pQM= *itQM;
        // Create MapPoints and asscoiate to keyframes
        cv::KeyPoint kpUn=pKFcur->mvKeysUn[pQM.i1c];
        int posX, posY;
        if(fg.IsPointEligible(kpUn, posX, posY))
        {// Triangulate each match
            const cv::KeyPoint &kp1 = pKFcur->mvKeysUn[pQM.i1c];
            const cv::KeyPoint &kp2 = pKFcur->mvRightKeysUn[pQM.i1c];
            const cv::KeyPoint &kp3 = pPrevFrame->mvKeysUn[pQM.i1p];
            const cv::KeyPoint &kp4 = pPrevFrame->mvRightKeysUn[pQM.i1p];
#if 1
            // Check parallax between left and right rays
            Vector3d xn1((kp1.pt.x-cx1)/fx1,
                         (kp1.pt.y-cy1)/fy1, 1.0 ),
                    ray1(xn1);
            Vector3d xn3((kp3.pt.x-cx2)/fx2,
                         (kp3.pt.y-cy2)/fy2, 1.0 );

            Vector3d ray3 = Rw2c1*Rw2c2.transpose()*xn3;
            float cosParallaxRays = ray1.dot(ray3)/(ray1.norm()*ray3.norm());

            if((cosParallaxRays<0 || cosParallaxRays>Config::triangMaxCosRays())
                    && (kp1.pt.x -kp2.pt.x< Config::triangMinDisp()))
                continue;

            // Linear Triangulation Method
            Vector3d xn2((kp2.pt.x-pKFcur->right_cam_->cx())/pKFcur->right_cam_->fx(),
                         (kp2.pt.y-pKFcur->right_cam_->cy())/pKFcur->right_cam_->fy(), 1.0 );

            Vector3d xn4((kp4.pt.x-pPrevFrame->right_cam_->cx())/pPrevFrame->right_cam_->fx(),
                         (kp4.pt.y-pPrevFrame->right_cam_->cy())/pPrevFrame->right_cam_->fy(), 1.0 );

            Eigen::Matrix<double,8,4> A;
            A.row(0) = xn1(0)*Tw2c1.row(2)-Tw2c1.row(0);
            A.row(1) = xn1(1)*Tw2c1.row(2)-Tw2c1.row(1);
            A.row(2) = xn2(0)*Tw2c1r.row(2)-Tw2c1r.row(0);
            A.row(3) = xn2(1)*Tw2c1r.row(2)-Tw2c1r.row(1);
            A.row(4) = xn3(0)*Tw2c2.row(2)-Tw2c2.row(0);
            A.row(5) = xn3(1)*Tw2c2.row(2)-Tw2c2.row(1);
            A.row(6) = xn4(0)*Tw2c2r.row(2)-Tw2c2r.row(0);
            A.row(7) = xn4(1)*Tw2c2r.row(2)-Tw2c2r.row(1);
            cv::Mat Aprime, w,u,vt;
            cv::eigen2cv(A, Aprime);
            cv::SVD::compute(Aprime,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
            cv::Mat x3D = vt.row(3).t();
            if(x3D.at<double>(3)==0)
                continue;

            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
            Eigen::Vector3d x3Dt;
            cv::cv2eigen (x3D, x3Dt);

       /*     obs[0]= xn1; obs[1]= xn2; obs[2]= xn3; obs[3]= xn4;
            double pdop=0;
            MapPoint::optimize(obs,frame_poses,  x3Dt, pdop,pPrevFrame->cam_, 5);
            if(pdop> Config::PDOPThresh())
                continue;*/

            //Check triangulation in front of cameras
            float z1 = Rw2c1.row(2)*x3Dt+ twinc1(2);
            if(z1<=0)
                continue;
            float z2 = Rw2c2.row(2)*x3Dt+twinc2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            float sigmaSquare1 = pKFcur->GetSigma2(kp1.octave);
            float x1 = Rw2c1.row(0)*x3Dt+twinc1(0);
            float y1 = Rw2c1.row(1)*x3Dt+twinc1(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1+cx1;
            float v1 = fy1*y1*invz1+cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>Config::reprojThresh2()*sigmaSquare1)
                continue;

            //Check reprojection error in second frame
            float sigmaSquare2 = pPrevFrame->GetSigma2(kp3.octave);
            float x2 = Rw2c2.row(0)*x3Dt+twinc2(0);
            float y2 = Rw2c2.row(1)*x3Dt+twinc2(1);
            float invz2 = 1.0/z2;
            float u2 = fx2*x2*invz2+cx2;
            float v2 = fy2*y2*invz2+cy2;
            float errX2 = u2 - kp3.pt.x;
            float errY2 = v2 - kp3.pt.y;
            if((errX2*errX2+errY2*errY2)>Config::reprojThresh2()*sigmaSquare2)
                continue;

            //Check scale consistency
            Eigen::Vector3d normal1 = x3Dt-Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3d normal2 = x3Dt-Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            float ratioDist = dist1/dist2;
            if(ratioDist*ratioFactor<1.f || ratioDist>ratioFactor)
                continue;
#else
            //Assume left right image rectified and no distortion

            if(kp1.pt.x -kp2.pt.x< Config::triangMinDisp() || kp3.pt.x- kp4.pt.x<Config::triangMinDisp())//parallax
                continue;

            float base= -pKFcur->mTl2r.translation()[0];
            float base_disp = base/(kp1.pt.x -kp2.pt.x);
            Eigen::Vector3d x3D1;
            x3D1(0) = (kp1.pt.x- cx1)*base_disp;
            x3D1(1) = ((kp1.pt.y+ kp2.pt.y)/2 - cy1)*base_disp;
            x3D1(2) = fx1*base_disp;
            x3D1= Rw2c1.transpose()*(x3D1- twinc1);
            base_disp = base/(kp3.pt.x -kp4.pt.x);
            Eigen::Vector3d x3D2;
            x3D2(0) = (kp3.pt.x- cx2)*base_disp;
            x3D2(1) = ((kp3.pt.y+ kp4.pt.y)/2 - cy2)*base_disp;
            x3D2(2) = fx2*base_disp;
            x3D2= Rw2c2.transpose()*(x3D2 - twinc2);
            if(abs(x3D1(2)- x3D2(2))>0.8)
                continue;
            Eigen::Vector3d x3Dt= (x3D1+ x3D2)/2;
#endif
            // Triangulation is succesful
            MapPoint* pMP = new MapPoint(x3Dt,pKFcur, pQM.i1c, mpMap);

            pMP->AddObservation(pPrevFrame,pQM.i1p);
            pMP->AddObservation(pPrevFrame,pQM.i2p, false);

            pKFcur->AddMapPoint(pMP,pQM.i1c);
            pPrevFrame->AddMapPoint(pMP,pQM.i1p);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            fg.AddMapPoint(posX, posY, pQM.i1c);
        }
    }

    SLAM_DEBUG_STREAM("created initial map of points from quad matches:"<<mpMap->MapPointsInMap()<<" "<<vQuadMatches.size());
    //    Optimizer::GlobalBundleAdjustemnt(mpMap,20);
    float medianDepth = pKFcur->ComputeSceneMedianDepth(2);

    if(medianDepth<0 || pKFcur->TrackedMapPoints()<Config::initMinTracked())
    {
        SLAM_DEBUG_STREAM("Wrong initialization, reseting...");
        Reset();
        return;
    }
    // Update Connections
    pPrevFrame->UpdateConnections();
    pKFcur->UpdateConnections();
    mpLocalMapper->InsertKeyFrame(pPrevFrame);

  //  mpInitialFrame=NULL;// do not delete last frame because it is to be used in framepublisher
    mpLastFrame = pKFcur;
    delete mpCurrentFrame;
    mpCurrentFrame = mpLastFrame;// set for framepublisher
    mnLastKeyFrameId=mpLastFrame->mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId=mpLastFrame->mnId;// Huai: increase lastreloc frameid so we can reduce nMaxFrames to 2 without possibly causing inserting the second kf twice
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mvpTemporalFrames.push_back(pPrevFrame);
    assert(mvpTemporalFrames.size()==1);

#ifdef SLAM_USE_ROS
    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());
#endif
    mState=WORKING;
}

void Tracking::CreateInitialMap(Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, double norm_tcinw)
{
    // Set Frame Poses
    mpCurrentFrame->SetPose(Rcw, tcw);
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(*mpLastFrame, mpMap, mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(*mpCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();
    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);
    assert(pKFcur->mnFrameId==1);
    //use a grid to control map point distribution and number
    FeatureGrid fg(30,mnFeatures, pKFcur->mnMaxX,
                   pKFcur->mnMinX, pKFcur->mnMaxY, pKFcur->mnMinY);
    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();++i)
    {
        if(mvIniMatches[i]<0)
            continue;
        // Create MapPoints and asscoiate to keyframes
        cv::KeyPoint kpUn=pKFcur->mvKeysUn[mvIniMatches[i]];
        int posX, posY;
        if(fg.IsPointEligible(kpUn, posX, posY))
        {
            //Create MapPoint.
            Eigen::Vector3d worldPos; worldPos<<mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
            MapPoint* pMP = new MapPoint(worldPos,pKFcur,mvIniMatches[i], mpMap);

            pKFini->AddMapPoint(pMP,i);
            pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

            pMP->AddObservation(pKFini,i);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            //Fill Current Frame structure
            mpCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;

            //Add to Map
            mpMap->AddMapPoint(pMP);
            fg.AddMapPoint(posX, posY, mvIniMatches[i]);
        }
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    char buffer[300];
    sprintf(buffer, "New Map created with %d points",mpMap->MapPointsInMap());
    cout<<buffer<<endl;

    // Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    assert((pKFini->GetCameraCenter().norm())==0);

    float invMedianDepth = 1.0f/medianDepth;

    if(norm_tcinw!=0)
        invMedianDepth= norm_tcinw/ pKFcur->GetPose().translation().norm();

    if(medianDepth<0 || pKFcur->TrackedMapPoints()<60)
    {
        SLAM_INFO_STREAM("Wrong initialization, reseting...");
        Reset();
        return;
    }

    // Scale initial baseline
    Sophus::SE3d Tcw = pKFcur->GetPose();
    Tcw.translation()*=invMedianDepth;
    pKFcur->SetPose(Tcw);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    //mpInitialFrame=NULL;//do not delete last frame as it is used in framepublisher
    mpLastFrame = pKFcur;
    delete mpCurrentFrame;
    mpCurrentFrame=mpLastFrame; // set for framepublisher
    mnLastKeyFrameId=mpLastFrame->mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId=mpLastFrame->mnId;

    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mvpTemporalFrames.push_back(pKFini);
    mvpTemporalFrames.push_back(pKFcur);

#ifdef SLAM_USE_ROS
    mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());
#endif

    mState=WORKING;
}

// extend trails/ associate map points from previous frame to current frame
bool Tracking::TrackPreviousFrame(const Sophus::SE3d & Tcp,
                                  const std::vector<p_match>&vQuadMatches)
{
    vector<MapPoint*>& vpMapPointMatches=mpCurrentFrame->mvpMapPoints;
    int nmatches=0;
    for (auto it= vQuadMatches.begin(), ite=vQuadMatches.end(); it!=ite ;++it)
    {
        MapPoint* pMP1 =  mpLastFrame->mvpMapPoints[it->i1p];
        if(!pMP1 || pMP1->isBad())
            continue;
        vpMapPointMatches[it->i1c] = pMP1;
        ++nmatches;
    }

    mpCurrentFrame->SetPose(Tcp*mpLastFrame->mTcw);

    return nmatches>=10;
}
bool Tracking::TrackPreviousFrame()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mpCurrentFrame->GetScaleLevels()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;

    int nmatches = matcher.WindowSearch(*mpLastFrame,*mpCurrentFrame,200,vpMapPointMatches,minOctave);

    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(*mpLastFrame,*mpCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mpCurrentFrame->mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
        }
    }

    mpCurrentFrame->SetPose(mpLastFrame->mTcw);
    mpCurrentFrame->mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(mpCurrentFrame);

        for(size_t i =0; i<mpCurrentFrame->mvbOutlier.size(); i++)
            if(mpCurrentFrame->mvbOutlier[i])
            {
                mpCurrentFrame->mvpMapPoints[i]=NULL;
                mpCurrentFrame->mvbOutlier[i]=false;
                nmatches--;
            }

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(*mpLastFrame,*mpCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(*mpLastFrame,*mpCurrentFrame,50,vpMapPointMatches);


    mpCurrentFrame->mvpMapPoints=vpMapPointMatches;

    if(nmatches<10)
        return false;

    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(mpCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mpCurrentFrame->mvbOutlier.size(); i++)
        if(mpCurrentFrame->mvbOutlier[i])
        {
            mpCurrentFrame->mvpMapPoints[i]=NULL;
            mpCurrentFrame->mvbOutlier[i]=false;
            nmatches--;
        }

    return nmatches>=10;
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Compute current pose by motion model
    mpCurrentFrame->SetPose( mVelocity*mpLastFrame->mTcw);

    fill(mpCurrentFrame->mvpMapPoints.begin(),mpCurrentFrame->mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int nmatches = matcher.SearchByProjection(*mpCurrentFrame,*mpLastFrame,15);

    if(nmatches<20)
        return false;

    // Optimize pose with all correspondences
    Optimizer::PoseOptimization(mpCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mpCurrentFrame->mvpMapPoints.size(); i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(mpCurrentFrame->mvbOutlier[i])
            {
                mpCurrentFrame->mvpMapPoints[i]=NULL;
                mpCurrentFrame->mvbOutlier[i]=false;
                nmatches--;
            }
        }
    }

    return nmatches>=10;
}

bool Tracking::TrackLocalMap()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    UpdateReference();

    // Search Local MapPoints
    SearchReferencePointsInFrustum();

    // Optimize Pose
    mnMatchesInliers = Optimizer::PoseOptimization(mpCurrentFrame);

    // Update MapPoints Statistics
    for(size_t i=0; i<mpCurrentFrame->mvpMapPoints.size(); i++)
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(!mpCurrentFrame->mvbOutlier[i])
                mpCurrentFrame->mvpMapPoints[i]->IncreaseFound();
        }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mpCurrentFrame->mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}
void Tracking::setupG2o(G2oCameraParameters * g2o_cam,
                        G2oCameraParameters*g2o_cam_right,
                        G2oIMUParameters * g2o_imu,
                        g2o::SparseOptimizer * optimizer) const
{
    if(mbUseIMUData)
    {   //block solver_6_3 has errors in eigen in optimizing stereo+IMU observations,
        // About choosing block solvers, g2o has some examples
        //in g2o/examples/ba_anchored_inverse_depth/ba_anchored_inverse_depth_demo.cpp,
        // schur trick case: BlockSolver_6_3 and marginalizing 3D point vertices
        // otherwise: BlockSolverX and no marginalization
        // in g2o/examples/ba/ba_demo.cpp, BlockSolver_6_3 and marginalizing 3D point vertices
        //in g2o/examples/icp/gicp_sba_demo.cpp, BlockSolverX are used together with marginalizing 3D point vertices
        // in g2o/examples/bal/bal_example.cpp, g2o::BlockSolver< g2o::BlockSolverTraits<9, 3> > is used with marginalization
        // About linear solvers, g2o provides LinearSolverDense, LinearSolverCSparse, LinearSolverCholmod, and LinearSolverPCG

        optimizer->setVerbose(false);
        g2o::BlockSolverX::LinearSolverType * linearSolver=
                new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * block_solver =
                new g2o::BlockSolverX(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* lm =
                new g2o::OptimizationAlgorithmLevenberg(block_solver);

        lm->setMaxTrialsAfterFailure(5);
        optimizer->setAlgorithm(lm);
    }else{
        optimizer->setVerbose(false);
        typename g2o::BlockSolver_6_3::LinearSolverType * linearSolver
                = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * block_solver
                = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg * lm
                = new g2o::OptimizationAlgorithmLevenberg(block_solver);
        lm->setMaxTrialsAfterFailure(5);
        optimizer->setAlgorithm(lm);
    }
    if (!optimizer->addParameter(g2o_cam))
    {
        assert(false);
    }
#ifndef MONO
    assert(g2o_cam_right!=NULL);

    if (!optimizer->addParameter(g2o_cam_right))
    {
        assert(false);
    }
#endif
    if(mbUseIMUData&&g2o_imu!=NULL){
        if (!optimizer->addParameter(g2o_imu))
        {
            assert(false);
        }
    }
}

G2oVertexSE3* Tracking::addPoseToG2o(const SE3d & T_me_from_w,
                                     int pose_id,
                                     bool fixed,
                                     g2o::SparseOptimizer * optimizer,
                                     const Sophus::SE3d* first_estimate) const
{
    G2oVertexSE3 * v_se3 = new G2oVertexSE3();

    v_se3->setId(pose_id);
    v_se3->setEstimate(T_me_from_w);
    v_se3->setFixed(fixed);
    if(first_estimate!=NULL)
        v_se3->setFirstEstimate(*first_estimate);
    optimizer->addVertex(v_se3);
    return v_se3;
}
G2oVertexSpeedBias* Tracking::addSpeedBiasToG2o(const Matrix<double, 9,1> & vinw_bias,
                                                int sb_id,
                                                bool fixed,
                                                g2o::SparseOptimizer * optimizer,
                                                const Eigen::Matrix<double, 9,1> * first_estimate) const
{
    G2oVertexSpeedBias * v_sb = new G2oVertexSpeedBias();
    v_sb->setId(sb_id);
    v_sb->setEstimate(vinw_bias);
    v_sb->setFixed(fixed);
    // v_sb->setMarginalized(true); // this line causes error
    if(first_estimate!=NULL)
        v_sb->setFirstEstimate(*first_estimate);
    optimizer->addVertex(v_sb);
    return v_sb;
}

//N.B. g2o refuses negative indices
// copy frames in temporal and spatial window and the previous and current frame to g2o
// return next unused possible vertex id
// i found that fix the first frame does not have much effect
size_t Tracking::copyAllPosesToG2o(g2o::SparseOptimizer * optimizer)
{
    size_t max_vertex_id=0;
    size_t vertexid= 0;
    for (vector <KeyFrame*>::const_iterator  it_win = mvpLocalKeyFrames.begin(), it_end_win=mvpLocalKeyFrames.end();
         it_win!=it_end_win; ++it_win)
    {
        SE3d Tw2cref= (*it_win)->GetPose(); //transform from world to reference camera (i.e. left camera) frame
        vertexid= (*it_win)->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;
        if((*it_win)->mbFixedLinearizationPoint){
            SE3d Tw2cref_fe=(*it_win)->mTcw_first_estimate;
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false, optimizer, &Tw2cref_fe);
        }
        else
        {
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false, optimizer);
        }
    }
    for (deque<Frame*>::const_iterator  it_win = mvpTemporalFrames.begin(), it_end_win=mvpTemporalFrames.end();
         it_win!=it_end_win; ++it_win)
    {
        SE3d Tw2cref= (*it_win)->GetPose();  //transform from world to reference camera (i.e. left camera) frame
        vertexid= (*it_win)->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;

        if((*it_win)->mbFixedLinearizationPoint){
            SE3d Tw2cref_fe=(*it_win)->mTcw_first_estimate;
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false,  optimizer, &Tw2cref_fe);
            if(mbUseIMUData)
            {
                (*it_win)->v_sb_=addSpeedBiasToG2o((*it_win)->speed_bias, vertexid+1, false,
                                                   optimizer, &(*it_win)->speed_bias_first_estimate);
            }
        }
        else
        {
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid,  false, optimizer);
            if(mbUseIMUData)
            {
                (*it_win)->v_sb_=addSpeedBiasToG2o((*it_win)->speed_bias, vertexid+1, false,
                                                   optimizer);
            }
        }
    }
    SE3d Tw2cref;
    // previous frame
#ifndef MONO
        assert( mvpTemporalFrames.size()==0 || mvpTemporalFrames.back()->mnId+1== mpLastFrame->mnId);
        Tw2cref= mpLastFrame->mTcw; //transform from world to reference camera (i.e. left camera) frame
        vertexid= mpLastFrame->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;
        assert(!mpLastFrame->mbFixedLinearizationPoint);
        mpLastFrame->v_kf_=addPoseToG2o(Tw2cref, vertexid, false, optimizer);
        if(mbUseIMUData)
        {
            mpLastFrame->v_sb_=addSpeedBiasToG2o(mpLastFrame->speed_bias, vertexid+1, false,
                                                 optimizer);
        }
#endif
    //current frame
    Tw2cref= mpCurrentFrame->mTcw; //transform from world to reference camera (i.e. left camera) frame
    vertexid= mpCurrentFrame->mnId*MAGIC2;
    if(max_vertex_id< vertexid)
        max_vertex_id=vertexid;
    mpCurrentFrame->v_kf_=addPoseToG2o(Tw2cref, vertexid, false, optimizer);
    if(mbUseIMUData)
    {
        mpCurrentFrame->v_sb_=addSpeedBiasToG2o(mpCurrentFrame->speed_bias, vertexid+1, false,
                                                optimizer);
    }
    return max_vertex_id+2;
}

inline G2oVertexPointXYZ* Tracking::addPointToG2o( MapPoint* pPoint,
                                                   int g2o_point_id, bool fixed,
                                                   g2o::SparseOptimizer * optimizer) const
{
    G2oVertexPointXYZ * v_point = new G2oVertexPointXYZ;
    v_point->setId(g2o_point_id);
    v_point->setFixed(fixed);
    v_point->setEstimate(pPoint->GetWorldPos());
    v_point->setMarginalized(true);//if true, Schur trick is used to marginalize this vertex during optimization
    if(pPoint->mbFixedLinearizationPoint)
        v_point->setFirstEstimate(pPoint->mWorldPos_first_estimate);
    optimizer->addVertex(v_point);
    return v_point;
}

G2oEdgeProjectXYZ2UV* Tracking::addObsToG2o(const Vector2d & obs,
                                            const Matrix2d & Lambda,
                                            ScaViSLAM::G2oVertexPointXYZ* point_vertex,
                                            ScaViSLAM::G2oVertexSE3* pose_vertex,
                                            bool robustify,
                                            double huber_kernel_width,
                                            g2o::SparseOptimizer * optimizer, SE3d * pTs2c)
{
    G2oEdgeProjectXYZ2UV * e=NULL;
    if(pTs2c)
        e = new G2oExEdgeProjectXYZ2UV(*pTs2c);
    else
        e = new G2oEdgeProjectXYZ2UV();
    e->resize(2);

    assert(point_vertex->dimension()==3);
    e->vertices()[0] = point_vertex;

    assert(pose_vertex->dimension()==6);
    e->vertices()[1] = pose_vertex;

    e->setMeasurement(obs);
    e->information() = Lambda;

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(huber_kernel_width);

    e->resizeParameters(1);
    bool param_status = e->setParameterId(0, pTs2c==NULL ? 0:1);
    assert(param_status);
    optimizer->addEdge(e);
    return e;
}
/// Temporary container to hold the g2o edge with reference to frame and point.
struct EdgeContainerSE3d
{
    G2oEdgeProjectXYZ2UV*     edge;
    Frame*          frame;
    size_t id_; // id in frame feature vector
    bool            is_deleted;
    EdgeContainerSE3d(G2oEdgeProjectXYZ2UV* e, Frame* frame, size_t id) :
        edge(e), frame(frame),id_(id),is_deleted(false)
    {}
};
// given the local temporal frames (left and right frame in stereo case), the local keyframes,
// and the current frame, optimize poses of left frames of temporal frames and local keyframes,
// and positions of points observed in both windows.
// we do not need to check pMP->isBad() or pKF->isBad,
// because localMapper will not setbadflag for points and keyframes in double window
//note this function only works for stereo case
int Tracking::LocalOptimizeSVO()
{
 /*   g2o::SparseOptimizer optimizer;
    ScaViSLAM::G2oCameraParameters  * g2o_cam
            = new G2oCameraParameters(Vector2d(cam_->cx(), cam_->cy()),
                                      Vector2d(cam_->fx(), cam_->fy()));
    g2o_cam->setId(0);
    ScaViSLAM::G2oCameraParameters  * g2o_cam_right
            = new G2oCameraParameters(Vector2d(right_cam_->cx(), right_cam_->cy()),
                                      Vector2d(right_cam_->fx(), right_cam_->fy()));
    g2o_cam_right->setId(1);
    G2oIMUParameters  * g2o_imu  = new  G2oIMUParameters(imu_);
    g2o_imu->setId(2);
    setupG2o(g2o_cam, g2o_cam_right, g2o_imu, &optimizer);
    std::map<unsigned int, Frame*> vertexid_2_frame;
    std::map<unsigned int, MapPoint*> vertexid_2_mappoint;
    size_t nOffset=copyAllPosesToG2o(&optimizer, vertexid_2_frame);
#ifdef SLAM_DEBUG_OUTPUT
    map<MapPoint*, size_t> histogram;
#endif    
    // add constraint by IMU observations for frames in the temporal window and the current frame
    if(mbUseIMUData){
        for (std::deque<Frame*>::const_iterator  it_win = mvpTemporalFrames.begin(), it_end_win=mvpTemporalFrames.end();
             it_win!=it_end_win; ++it_win)
        {
            if((*it_win)->cam_id_!=0 ) continue;
            G2oEdgeIMUConstraint * e = new G2oEdgeIMUConstraint();
            e->setParameterId(0,2);
            e->resize(4);
            if((*it_win)->next_frame ==NULL) // the last frame in the temporal window
            {
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
                e->vertices()[2]
                        = mpCurrentFrame->v_kf_;
                e->vertices()[3]
                        = mpCurrentFrame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= mpCurrentFrame->mTimeStamp;
                e->setMeasurement(mpCurrentFrame->imu_observ);
            }
            else{
                assert((*it_win)->next_frame== (*(it_win+2)));
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
                e->vertices()[2]
                        = (*it_win)->next_frame->v_kf_;
                e->vertices()[3]
                        = (*it_win)->next_frame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= (*it_win)->next_frame->mTimeStamp;
                e->setMeasurement((*it_win)->next_frame->imu_observ);
            }
            e->calcAndSetInformation(*g2o_imu);
            optimizer.addEdge(e);
        }
    }

    // Now go throug all the points and add a measurement. Add a fixed neighbour
    // Keyframe if it is not in the set of core kfs
    // TODO: check that mvpLocalMapPoints contains all mappoints observed in double window

    int nInitialCorrespondences=0; //how many map points are tracked in current frame

    list<EdgeContainerSE3d> edges;

    const double delta2 = 5.991;
    const double delta = sqrt(delta2);

    size_t n_mps = 0;
    size_t n_fix_kfs=0;
    list<KeyFrame*> neib_kfs;
    for(auto it_pt = mvpLocalMapPoints.begin(), ite=mvpLocalMapPoints.end(); it_pt!=ite; ++it_pt)
    {
        // Create point vertex
        MapPoint* pMP = *it_pt;
        if(pMP->isBad()) continue;
        G2oVertexPointXYZ*v_pt= addPointToG2o( pMP, pMP->mnId + nOffset, false, &optimizer, vertexid_2_mappoint);
        pMP->v_pt_= v_pt;
        ++n_mps;
        // Add edges
        std::map<KeyFrame*, size_t> obs_copy= (*it_pt)->GetObservations();
        std::map<KeyFrame*, size_t>::const_iterator mit_obs=obs_copy.begin();
        while(mit_obs!=obs_copy.end())
        {
            KeyFrame * pKF= mit_obs->first;
            if(pKF->v_kf_ == NULL)
            {
                // frame does not have a vertex yet -> it belongs to the neib kfs and
                // is fixed. create one:
                ++n_fix_kfs;
                SE3d Tw2cref= pKF->GetPose(); //transform from world to reference camera (i.e. left camera) frame
                size_t vertexid= pKF->mnId*MAGIC2;

                if( pKF->mbFixedLinearizationPoint){
                    SE3d Tw2cref_fe= pKF->mTcw_first_estimate;
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer, &Tw2cref_fe);
                }
                else
                {
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer);
                }
                neib_kfs.push_back(pKF);
            }
            // create edge
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKF->mvKeysUn[mit_obs->second];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pKF->GetInvSigma2(kpUn.octave);

            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, Eigen::Matrix2d::Identity()*invSigma2,
                                                     v_pt, pKF->v_kf_, true, delta, &optimizer);
            histogram[pMP]++;
            edges.push_back(EdgeContainerSE3d(porter, pKF, mit_obs->second));
            ++mit_obs;
        }
    }

    // add points' observations for each non keyframe in the temporal window and the current frame
    // add observations of frames in the temporal window
    Frame * pLastRefFrame=NULL;
    for(deque<Frame*>::const_iterator qIt= mvpTemporalFrames.begin(), qItEnd=mvpTemporalFrames.end();
        qIt!=qItEnd; ++qIt)
    {
        if((*qIt)->isKeyFrame())
        {
            assert((*qIt)->cam_id_==0);
            pLastRefFrame= *qIt;
            continue; //keyframe observations are already added for mappoints
        }
        vector<MapPoint*> vpMPs = (*qIt)->GetMapPointMatches();
        size_t jim=0;
        if((*qIt)->cam_id_==0){
            pLastRefFrame= *qIt;
            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end();
                itMP!=itEndMP; ++itMP, ++jim)
            {
                MapPoint* pMP = *itMP;
                if(pMP && (!pMP->isBad()) && pMP->v_pt_!=NULL)
                    // if valid pointer, and not marked as bad by optimizer,
                    // and has been added to optimizer, i.e., in mvpLocalMapPoints and is used as track reference for current frame
                {
                    Eigen::Matrix<double,2,1> obs;
                    cv::KeyPoint kpUn = pLastRefFrame->mvKeysUn[jim];
                    obs << kpUn.pt.x, kpUn.pt.y;
                    const float invSigma2 = pLastRefFrame->GetInvSigma2(kpUn.octave);
                    G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, Eigen::Matrix2d::Identity()*invSigma2,
                                                             pMP->v_pt_, pLastRefFrame->v_kf_, true, delta, &optimizer);
                    histogram[pMP]++;
                    edges.push_back(EdgeContainerSE3d(porter, pLastRefFrame, jim));
                }
            }
        }
        else
        {
            assert((*qIt)->mnId-1==pLastRefFrame->mnId);
            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end();
                itMP!=itEndMP; ++itMP, ++jim)
            {
                MapPoint* pMP = *itMP;
                if(pMP && !pMP->isBad() && pMP->v_pt_!=NULL)
                    // if valid pointer, and not marked as bad by optimizer, and has been added to optimizer
                {
                    Eigen::Matrix<double,2,1> obs;
                    cv::KeyPoint kpUn = (*qIt)->mvKeysUn[jim];
                    obs << kpUn.pt.x, kpUn.pt.y;
                    const float invSigma2 = (*qIt)->GetInvSigma2(kpUn.octave);
                    G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, Eigen::Matrix2d::Identity()*invSigma2, pMP->v_pt_,
                                                             pLastRefFrame->v_kf_, true, delta, &optimizer, &mTl2r );
                    histogram[pMP]++;
                    edges.push_back(EdgeContainerSE3d(porter, *qIt, jim));
                }
            }
        }
    }
    assert(mpCurrentFrame->cam_id_==0);
    pLastRefFrame= mpCurrentFrame;

    vector<MapPoint*> vpMPs = mpCurrentFrame->GetMapPointMatches();
    size_t jim=0;
    for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end();
        itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && (!(pMP->isBad())))// if valid pointer, and not marked as bad by optimizer
        {
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pLastRefFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pLastRefFrame->GetInvSigma2(kpUn.octave);
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, Eigen::Matrix2d::Identity()*invSigma2,
                                                     pMP->v_pt_, pLastRefFrame->v_kf_, true, delta, &optimizer);
            histogram[pMP]++;
            edges.push_back(EdgeContainerSE3d(porter, pLastRefFrame, jim));
            ++nInitialCorrespondences;
        }
    }
    vpMPs = mpCurrentRightFrame->GetMapPointMatches();
    jim=0;
    for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end();
        itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && (!(pMP->isBad())))
            // if valid pointer, and not marked as bad by optimizer
        {
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = mpCurrentRightFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = mpCurrentRightFrame->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;

            // set right edge
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat, pMP->v_pt_,
                                                     pLastRefFrame->v_kf_, true, delta, &optimizer, &mTl2r );
            histogram[pMP]++;
            edges.push_back(EdgeContainerSE3d(porter, mpCurrentRightFrame.get(), jim));

        }
    }
#if 1
    for(auto it= histogram.begin(), ite=histogram.end(); it!=ite; ++it)
    {
        MapPoint *pMP=it->first;
        assert(it->second>=2);
    }
#endif
    optimizer.initializeOptimization();

    //    cout << "Performing structure-only BA:" << endl;
    g2o::StructureOnlySolver<3> structure_only_ba;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
        if (v->dimension() == 3)
            points.push_back(v);
    }
    structure_only_ba.calc(points, 5);

    optimizer.optimize(5);

    int nBad=0;
    // Check inlier observations, this section draws inspiration from LocalBundleAdjustment and PoseOptimization by Raul
#ifdef MONO
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        G2oEdgeProjectXYZ2UV* e = vpEdges[i];
        MapPoint* pMP = vpMapPointEdge[i];
        if(!e)
            continue;
        if(pMP->isBad())
            continue;
        Frame* pFi = vnIndexEdge[i].first;
        int idx= vnIndexEdge[i].second;
        if(pFi==mpCurrentFrame)
        {
            if(pFi->mvbOutlier[idx]){
                const float invSigma2 = pFi->GetInvSigma2(0);
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                e->computeError();
            }
            if(e->chi2()>delta2)
            {
                pFi->mvbOutlier[idx]=true;
                // e->setInformation(Eigen::Matrix2d::Identity()*1e-10);
                nBad++;
            }
            else
            {
                pFi->mvbOutlier[idx]=false;
            }
        }
        else{
            if(e->chi2()>delta2 || !e->isDepthPositive())
            {
                pFi->EraseMapPointMatch(idx);
                pMP->EraseObservation(pFi);
            }
        }
    }
#else   
    for(list<EdgeContainerSE3d>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        G2oEdgeProjectXYZ2UV* e = it->edge;
        if(e->chi2()>delta2)// || !e->isDepthPositive())
        {
            Frame* pFi = it->frame;
            cout<< it->id_<<" "<< pFi->mvpMapPoints.size()<<endl;
            MapPoint* pMP= pFi->GetMapPoint(it->id_);
            if(pMP==NULL || (pMP->isBad()))
                continue;

            if(pFi->isKeyFrame())
            {
                pFi->EraseMapPointMatch(it->id_);
                pMP->EraseObservation((KeyFrame*)pFi);
            }
            else{
                pFi->EraseMapPointMatch(it->id_);
                if(pFi==mpCurrentFrame)
                    ++nBad;
            }
        }
    }
#endif    
    //update current frame and current right frame
    mpCurrentFrame->SetPose((static_cast<G2oVertexSE3*>(mpCurrentFrame->v_kf_))->estimate());
    mpCurrentFrame->v_kf_ = NULL;
    mpCurrentFrame->speed_bias = (static_cast<G2oVertexSpeedBias*>(mpCurrentFrame->v_sb_))->estimate();
    mpCurrentFrame->v_sb_=NULL;
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return nBad;
    // Update Keyframes
    for(deque<Frame*>::iterator it = mvpTemporalFrames.begin(); it != mvpTemporalFrames.end(); ++it)
    {
        if((*it)->v_kf_==NULL) continue;
        (*it)->SetPose((static_cast<G2oVertexSE3*>((*it)->v_kf_))->estimate());
        (*it)->v_kf_ = NULL;
        if((*it)->v_sb_!=NULL)
        {
            (*it)->speed_bias = (static_cast<G2oVertexSpeedBias*>((*it)->v_sb_))->estimate();
            (*it)->v_sb_=NULL;
        }
    }
    for(vector<KeyFrame*>::iterator it = mvpLocalKeyFrames.begin(); it != mvpLocalKeyFrames.end(); ++it)
    {
        (*it)->SetPose((static_cast<G2oVertexSE3*>((*it)->v_kf_))->estimate());
        (*it)->v_kf_ = NULL;
        assert((*it)->v_sb_==NULL);
    }
    for(list<KeyFrame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
        (*it)->v_kf_ = NULL;

    // Update Mappoints
    for(vector<MapPoint*>::iterator it = mvpLocalMapPoints.begin(); it != mvpLocalMapPoints.end(); ++it)
    {
        if(((*it)==NULL) || ((*it)->isBad())) continue;
        (*it)->SetWorldPos((static_cast<G2oVertexPointXYZ*>((*it)->v_pt_))->estimate());
        (*it)->v_pt_ = NULL;
    }
    return nBad;*/
    return 0;
    //N.B. g2o will delete _algorithm, _parameters, EdgeSet and VertexIDMap upon destruction of SparseOptimizer
}

// For all map points in the local map, add them into the optimizer if they are observed by at least 2 keyframes
// return the number of outlier/bad observations made by the current frame
// we do not need to check pKF->isBad, because localMapper will not setbadflag for keyframes in double window
int Tracking::LocalOptimize()
{
    g2o::SparseOptimizer optimizer;
    ScaViSLAM::G2oCameraParameters  * g2o_cam
            = new G2oCameraParameters(Vector2d(cam_->cx(), cam_->cy()),
                                      Vector2d(cam_->fx(), cam_->fy()));
    g2o_cam->setId(0);
    ScaViSLAM::G2oCameraParameters  * g2o_cam_right=NULL;
#ifndef MONO
    g2o_cam_right
            = new G2oCameraParameters(Vector2d(right_cam_->cx(), right_cam_->cy()),
                                      Vector2d(right_cam_->fx(), right_cam_->fy()));
    g2o_cam_right->setId(1);
#endif
    G2oIMUParameters  * g2o_imu  = new  G2oIMUParameters(imu_);
    g2o_imu->setId(2);
    setupG2o(g2o_cam, g2o_cam_right, g2o_imu, &optimizer);
    size_t nOffset=copyAllPosesToG2o(&optimizer);
    // add constraint by IMU observations for frames in the temporal window and the current frame
    if(mbUseIMUData){
        for (std::deque<Frame*>::const_iterator  it_win = mvpTemporalFrames.begin(), it_end_win=mvpTemporalFrames.end();
             it_win!=it_end_win; ++it_win)
        {
            G2oEdgeIMUConstraint * e = new G2oEdgeIMUConstraint();
            e->setParameterId(0,2);
            e->resize(4);
            if((*it_win)->next_frame ==NULL)//must be the last frame in the temporal window
            {
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
#ifdef MONO
                e->vertices()[2]
                        = mpCurrentFrame->v_kf_;
                e->vertices()[3]
                        = mpCurrentFrame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= mpCurrentFrame->mTimeStamp;
                assert(mpCurrentFrame->imu_observ.size());
                e->setMeasurement(mpCurrentFrame->imu_observ);
#else
                e->vertices()[2]
                        = mpLastFrame->v_kf_;
                e->vertices()[3]
                        = mpLastFrame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= mpLastFrame->mTimeStamp;
                assert(mpLastFrame->imu_observ.size());
                e->setMeasurement(mpLastFrame->imu_observ);
#endif

            }
            else{
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
                e->vertices()[2]
                        = (*it_win)->next_frame->v_kf_;
                e->vertices()[3]
                        = (*it_win)->next_frame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= (*it_win)->next_frame->mTimeStamp;
#ifdef SLAM_DEBUG_OUTPUT
                if((*it_win)->next_frame->isKeyFrame())
                {
                    KeyFrame * larry= (KeyFrame*)((*it_win)->next_frame);
                    if(larry->isBad()){
                        SLAM_DEBUG_STREAM("Bad keyframe in temporal window mnId,N, mbNotErase, keys size:"
                                          <<larry->mnId<<" "<<larry->N<<" "<<larry->GetNotErase()<<" "<< larry->mvKeys.size());

//                        assert(false);
                    }
                }
#endif
                e->setMeasurement((*it_win)->next_frame->imu_observ);
            }
            e->calcAndSetInformation(*g2o_imu);
            optimizer.addEdge(e);
        }
#ifndef MONO
        // the previous frame and current frame
        G2oEdgeIMUConstraint * e = new G2oEdgeIMUConstraint();
        e->setParameterId(0,2);
        e->resize(4);   
        e->vertices()[0]
                = mpLastFrame->v_kf_;
        e->vertices()[1]
                = mpLastFrame->v_sb_;
        e->vertices()[2]
                = mpCurrentFrame->v_kf_;
        e->vertices()[3]
                = mpCurrentFrame->v_sb_;
        e->time_frames[0]= mpLastFrame->mTimeStamp;
        e->time_frames[1]= mpCurrentFrame->mTimeStamp;
        e->setMeasurement(mpCurrentFrame->imu_observ);
        e->calcAndSetInformation(*g2o_imu);
        optimizer.addEdge(e);
#endif
    }
    // Now go throug all the points and add observations. Add a fixed neighbour
    // Keyframe if it is not in the set of core kfs
    const double delta2 = 5.991;
    const double delta = sqrt(delta2);
    int nInitialCorrespondences=0; //how many map points are tracked in current frame
    list<EdgeContainerSE3d> edges;
    size_t n_mps = 0;
    size_t n_fix_kfs=0;

    list<KeyFrame*> neib_kfs;
    for(auto it_pt = mvpLocalMapPoints.begin(), ite=mvpLocalMapPoints.end(); it_pt!=ite; ++it_pt)
    {
        // Create point vertex
        MapPoint* pMP = *it_pt;
        if(pMP->isBad()) { (*it_pt)=NULL; continue;}
        std::map<KeyFrame*, size_t> obs_copy= pMP->GetObservations();
        if(obs_copy.size()<2) { (*it_pt)=NULL; continue;}
        G2oVertexPointXYZ*v_pt= addPointToG2o(pMP, pMP->mnId + nOffset, false, &optimizer);
        pMP->v_pt_= v_pt;
        ++n_mps;
        // Add edges
        std::map<KeyFrame*, size_t>::const_iterator mit_obs=obs_copy.begin();
        while(mit_obs!=obs_copy.end())
        {
            KeyFrame * pKF= mit_obs->first;
            if(pKF->v_kf_ == NULL)
            {
                // frame does not have a vertex yet -> it belongs to the neib kfs and
                // is fixed. create one:
                ++n_fix_kfs;
                SE3d Tw2cref= pKF->GetPose(); //transform from world to reference camera (i.e. left camera) frame
                size_t vertexid= pKF->mnId*MAGIC2;

                if( pKF->mbFixedLinearizationPoint){
                    SE3d Tw2cref_fe= pKF->mTcw_first_estimate;
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer, &Tw2cref_fe);
                }
                else
                {
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer);
                }
                neib_kfs.push_back(pKF);
            }
            // create edge
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKF->mvKeysUn[mit_obs->second];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pKF->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                     v_pt, pKF->v_kf_, true, delta, &optimizer);
            edges.push_back(EdgeContainerSE3d(porter, pKF, mit_obs->second));

#ifndef MONO
            // set right edge
            kpUn = pKF->mvRightKeysUn[mit_obs->second];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, v_pt, pKF->v_kf_, true, delta, &optimizer, &mTl2r );
            edges.push_back(EdgeContainerSE3d(porter, pKF,  mit_obs->second));
#endif
            ++mit_obs;
        }
    }

    // add points' observations for each non keyframe in the temporal window and the previous and current frame
    // add observations of frames in the temporal window
    for(deque<Frame*>::const_iterator qIt= mvpTemporalFrames.begin(), qItEnd=mvpTemporalFrames.end();
        qIt!=qItEnd; ++qIt)
    {
        if((*qIt)->isKeyFrame())
        {
            continue; //keyframe observations are already added for mappoints
        }
        size_t jim=0;
        for(vector<MapPoint*>::const_iterator itMP=(*qIt)->mvpMapPoints.begin(), itEndMP=(*qIt)->mvpMapPoints.end();
            itMP!=itEndMP; ++itMP, ++jim)
        {
            MapPoint* pMP = *itMP;
            if(pMP && pMP->v_pt_!=NULL)    // if valid pointer, and has been added to optimizer
            {
                //SET left EDGE
                Eigen::Matrix<double,2,1> obs;
                cv::KeyPoint kpUn = (*qIt)->mvKeysUn[jim];
                obs << kpUn.pt.x, kpUn.pt.y;
                const float invSigma2 = (*qIt)->GetInvSigma2(kpUn.octave);
                Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
                G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                         pMP->v_pt_, (*qIt)->v_kf_, true, delta, &optimizer);

                edges.push_back(EdgeContainerSE3d(porter, *qIt, jim));
#ifndef MONO
                // set right edge
                kpUn = (*qIt)->mvRightKeysUn[jim];
                obs << kpUn.pt.x, kpUn.pt.y;
                assert(kpUn.octave==0);
                porter=addObsToG2o(obs, infoMat,
                                   pMP->v_pt_, (*qIt)->v_kf_, true, delta, &optimizer, &mTl2r );
                edges.push_back(EdgeContainerSE3d(porter, *qIt, jim));
#endif
            }
        }
    }
    size_t jim=0;
#ifndef MONO
    for(vector<MapPoint*>::const_iterator itMP=mpLastFrame->mvpMapPoints.begin(), itEndMP=mpLastFrame->mvpMapPoints.end();
        itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && pMP->v_pt_!=NULL)
        {
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = mpLastFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = mpLastFrame->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                     pMP->v_pt_, mpLastFrame->v_kf_, true, delta, &optimizer);
            edges.push_back(EdgeContainerSE3d(porter, mpLastFrame, jim));

            // set right edge
            kpUn = mpLastFrame->mvRightKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, pMP->v_pt_, mpLastFrame->v_kf_, true, delta, &optimizer, &mTl2r );
            edges.push_back(EdgeContainerSE3d(porter, mpLastFrame, jim));

        }
    }
#endif
    jim=0;
    for(vector<MapPoint*>::const_iterator itMP=mpCurrentFrame->mvpMapPoints.begin(),
        itEndMP=mpCurrentFrame->mvpMapPoints.end(); itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && pMP->v_pt_!=NULL)
        {
            //SET left EDGE
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = mpCurrentFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = mpCurrentFrame->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;

            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat, pMP->v_pt_,
                                                     mpCurrentFrame->v_kf_, true, delta, &optimizer );
            edges.push_back(EdgeContainerSE3d(porter, mpCurrentFrame, jim));

#ifndef MONO
            // set right edge
            kpUn = mpCurrentFrame->mvRightKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, pMP->v_pt_, mpCurrentFrame->v_kf_, true, delta, &optimizer, &mTl2r );
            edges.push_back(EdgeContainerSE3d(porter, mpCurrentFrame, jim));
#endif
            mpCurrentFrame->mvbOutlier[jim] = false;
            ++nInitialCorrespondences;
        }
    }
    if(edges.size()==0)
    {
        cout<<"Graph with no edges."<<endl;
        return 0;
    }
    optimizer.initializeOptimization();

    //    cout << "Performing structure-only BA:" << endl;
    g2o::StructureOnlySolver<3> structure_only_ba;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
        if (v->dimension() == 3)
            points.push_back(v);
    }
    structure_only_ba.calc(points, 5);
#ifndef MONO
// remove the right observations, because the extrinsic calibration error often causes drift
    for(auto it_edge= edges.begin(), ite_edge= edges.end(); it_edge!=ite_edge;)
    {
        if(dynamic_cast<G2oExEdgeProjectXYZ2UV*>(it_edge->edge))
            it_edge= edges.erase(it_edge);
        else
            ++it_edge;
    }

    for(auto eg_it=optimizer.edges().begin(), eg_end= optimizer.edges().end(); eg_it!=eg_end; )
    {
        if(dynamic_cast<G2oExEdgeProjectXYZ2UV*>(*eg_it))
        {
            eg_it= optimizer.edges().erase(eg_it);
        }
        else
            ++eg_it;
    }
#endif
    optimizer.optimize(5);
    int nBad=0;
    /// Emprically the following outlier removal process does not contribute much
#if 0
    // Check inlier observations, draw inspiration from LocalBundleAdjustment and PoseOptimization
    for(list<EdgeContainerSE3d>::iterator it = edges.begin(); it != edges.end();)
    {
        G2oEdgeProjectXYZ2UV* e1 = it->edge;
//N.B. right observations are removed for stereo odometry
        if(e1->chi2()>delta2)
        {
            Frame* pFi = it->frame;
            MapPoint* pMP= pFi->GetMapPoint(it->id_);
            if(pMP==NULL || (pMP->isBad())){
                ++it;
                continue;
            }
            if(pFi->isKeyFrame())
            {
                pFi->EraseMapPointMatch(it->id_);
                pMP->EraseObservation((KeyFrame*)pFi);
            }
            else{
                pFi->EraseMapPointMatch(it->id_);
                if(pFi==mpCurrentFrame){
                    pFi->mvbOutlier[it->id_]=true;
                    ++nBad;
                }
            }
            optimizer.removeEdge(e1);
            it =edges.erase(it);
        }
        else
            ++it;
    }

    // Optimize again without the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(3);
#endif    
    // Check inlier observations, this section draws inspiration from LocalBundleAdjustment and PoseOptimization by Raul
    for(list<EdgeContainerSE3d>::iterator it = edges.begin(); it != edges.end(); )
    {
        G2oEdgeProjectXYZ2UV* e1 = it->edge;
#if 0 //right observations are removed
        ++it;
        G2oExEdgeProjectXYZ2UV* e2 = (G2oExEdgeProjectXYZ2UV*)it->edge;
        if(e1->chi2()>delta2 || e2->chi2()>delta2)// || !e->isDepthPositive())
#else
        if(e1->chi2()>delta2)
#endif
        {
            Frame* pFi = it->frame;
            MapPoint* pMP= pFi->GetMapPoint(it->id_);
            if(pMP==NULL || (pMP->isBad()))
            {
                ++it;
                continue;
            }
            if(pFi->isKeyFrame())
            {
                pFi->EraseMapPointMatch(it->id_);
                pMP->EraseObservation((KeyFrame*)pFi);
            }
            else{
                pFi->EraseMapPointMatch(it->id_);
                if(pFi==mpCurrentFrame)
                    ++nBad;
            }
        }
        ++it;
    }
    //update current frame
    mpCurrentFrame->SetPose(mpCurrentFrame->v_kf_->estimate());
    mpCurrentFrame->v_kf_ = NULL;
#ifndef MONO
    mpLastFrame->SetPose(mpLastFrame->v_kf_->estimate());
    mpLastFrame->v_kf_ = NULL;
#endif
    if(mbUseIMUData){
        mpCurrentFrame->speed_bias = mpCurrentFrame->v_sb_->estimate();
        mpCurrentFrame->v_sb_=NULL;
#ifndef MONO
        mpLastFrame->speed_bias = mpLastFrame->v_sb_->estimate();
        mpLastFrame->v_sb_=NULL;
#endif
    }

    // Update Keyframes
    for(deque<Frame*>::iterator it = mvpTemporalFrames.begin(); it != mvpTemporalFrames.end(); ++it)
    {
        assert((*it)->v_kf_!=NULL);
        (*it)->SetPose((*it)->v_kf_->estimate());
        (*it)->v_kf_ = NULL;
        if(mbUseIMUData){
        assert((*it)->v_sb_!=NULL);
        (*it)->speed_bias = (*it)->v_sb_->estimate();
        (*it)->v_sb_=NULL;
        }
    }
    for(vector<KeyFrame*>::iterator it = mvpLocalKeyFrames.begin(); it != mvpLocalKeyFrames.end(); ++it)
    {
        (*it)->SetPose((*it)->v_kf_->estimate());
        (*it)->v_kf_ = NULL;
        assert((*it)->v_sb_==NULL);
    }
    for(list<KeyFrame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
        (*it)->v_kf_ = NULL;
    // Update Mappoints
    for(vector<MapPoint*>::iterator it = mvpLocalMapPoints.begin(); it != mvpLocalMapPoints.end(); ++it)
    {
        if((*it)==NULL) continue;
        (*it)->SetWorldPos((*it)->v_pt_->estimate());
        (*it)->v_pt_ = NULL;
    }
    return nBad;
    //N.B. g2o will delete _algorithm, _parameters, EdgeSet and VertexIDMap upon destruction of SparseOptimizer
}

// task:double window optimization to refine poses and points of frames
// in temporal window of T consecutive frames
// (may include keyframes), spatial window of S keyframes, and the current frame
// vStereoMatches are matched point features between left and right image of current stereo pair

bool Tracking::TrackLocalMapDWO()
{
    // Tracking from previous frame or relocalisation was succesfull and we have an estimation
    // of the camera pose and some map points tracked in the frame.
    // Update Local Map and Track

    // Update Local Map
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    // Update
    UpdateReferenceKeyFramesAndPoints();

    // Search Local MapPoints
    int nObs=-1;
#ifdef MONO
    nObs=SearchReferencePointsInFrustum();
#else
    nObs=SearchReferencePointsInFrustumStereo();
#endif
    // Optimize Pose
    if(mnLastRelocFrameId== mpCurrentFrame->mnId || mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()){
//        mnMatchesInliers = Optimizer::PoseOptimization(mpCurrentFrame);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        mnMatchesInliers= nObs;
        SLAM_DEBUG_STREAM("Inliers after local search:"<< mnMatchesInliers);
    }
    else{
        int nBad = LocalOptimize();
        mnMatchesInliers= nObs-nBad;
        SLAM_DEBUG_STREAM("Inliers after LO without bads:"<< mnMatchesInliers<<" "<<nBad);
    }

#if 0
    if(nObs!=mnMatchesInliers)
    {
        int yum=0, zinc=0, iota=0;
        for(size_t i=0; i<mpCurrentFrame->mvpMapPoints.size(); ++i)
        {
            MapPoint *pMP=mpCurrentFrame->mvpMapPoints[i];
            if(pMP && !pMP->isBad())
            {
                ++yum;
                if(pMP->mnTrackReferenceForFrame == mpCurrentFrame->mnId)
                {
                    ++zinc;
                    if(pMP->mnObservationsInDoubleWindow >=3)
                        ++iota;
                }
            }
        }
        cout<<"yum zinc iota:"<<yum<<" "<<zinc<<" "<<iota<<endl;

    }
#endif
    // Update MapPoints Statistics
    for(size_t i=0; i<mpCurrentFrame->mvpMapPoints.size(); ++i){
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(!mpCurrentFrame->mvbOutlier[i])
                mpCurrentFrame->mvpMapPoints[i]->IncreaseFound();
        }
    }
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mpCurrentFrame->mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<12)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // Huai: TODO: is it possible to process new keyframes when loop closing is running? But it would be quite involved
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mpCurrentFrame->mnId<mnLastRelocFrameId+mMaxFrames && mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mpCurrentFrame->mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mpCurrentFrame->mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than 90% of points than reference keyframe and enough inliers
    const float fPercent =0.9;
    const bool c2 = mnMatchesInliers<nRefMatches*fPercent && mnMatchesInliers>15;

    if((c1a||c1b)&&c2)
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}
bool Tracking::NeedNewKeyFrameStereo()
{
    // FAQ: can keyframes be created during loop closure but not processing by local mapping?
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // Not insert keyframes if not enough frames from last relocalisation have passed
    if(mpCurrentFrame->mnId<mnLastRelocFrameId+mMaxFrames) //&& mpMap->KeyFramesInMap()>mMaxFrames)
        return false;

    // Reference KeyFrame MapPoints
    int nRefMatches = mpReferenceKF->TrackedMapPoints();

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mpCurrentFrame->mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mpCurrentFrame->mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle;
    // Condition 2: Less than fPercent of points than reference keyframe and enough inliers and not too many
    const float fPercent =0.6;
    const bool c2 = (mnMatchesInliers< nRefMatches*fPercent) && mnMatchesInliers>15 && mnMatchesInliers <200;
    int num_matchless_cells= point_stats.numFeatureLessCorners3x3(4);
    // need KF if 2/9 of the image is void of matches as in ScaViSLAM
    if((c1a||c1b)&&(c2 || num_matchless_cells>3))
    {
        // If the mapping accepts keyframes insert, otherwise send a signal to interrupt BA, but not insert yet
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame* pKF = new KeyFrame(*mpCurrentFrame,mpMap,mpKeyFrameDB);
    mnLastKeyFrameId = mpCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
}

int Tracking::SearchReferencePointsInFrustum()
{
    int numObservs=0;
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mpCurrentFrame->mvpMapPoints.begin(), vend=mpCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mpCurrentFrame->mnId;
                pMP->mbTrackInView = false;
                ++numObservs;
            }
        }
    }

    mpCurrentFrame->UpdatePoseMatrices();// because in isInFrustum mRcw and mtcw is used

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mpCurrentFrame->mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mpCurrentFrame->isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }


    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mpCurrentFrame->mnId<mnLastRelocFrameId+2)
            th=5;
        int nMatches=matcher.SearchByProjection(*mpCurrentFrame,mvpLocalMapPoints,th);
        numObservs+= nMatches;
    }
    return numObservs;
}
// match local map points with stereo matches between stereo images of current frame
// return how many points in the local map is observed in the current frame
// for now, we first check image space consistency to remove outlier,
// then match features between local map and current frame
// TODO: alternatively, we may first match features, then check image space consistency
// of these matches as suggest by Leutenegger et al., IJRR 2014
int Tracking::SearchReferencePointsInFrustumStereo()
{
    int numObservs=0;
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mpCurrentFrame->mvpMapPoints.begin(), vend=mpCurrentFrame->mvpMapPoints.end();
        vit!=vend; ++vit)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = NULL;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mpCurrentFrame->mnId;
                pMP->mbTrackInView = false;
                //                ++pMP->mnObservationsInDoubleWindow; //Huai: count in the observation of current frame
                ++numObservs;
            }
        }
    }

    mpCurrentFrame->UpdatePoseMatrices();// because in isInFrustum mRcw and mtcw is used

    int nToMatch=0;
    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; ++vit)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mpCurrentFrame->mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mpCurrentFrame->isInFrustumStereo(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mpCurrentFrame->mnId<mnLastRelocFrameId+2)
            th=5;
        int nMatches= matcher.SearchByProjectionStereo(*mpCurrentFrame,mvpLocalMapPoints,th);
        numObservs+= nMatches;
    }
    return numObservs;
}

void Tracking::UpdateReference()
{    
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateReferenceKeyFrames();
    UpdateReferencePoints();
}

void Tracking::UpdateReferencePoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mpCurrentFrame->mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
            }
        }
    }
}


void Tracking::UpdateReferenceKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(size_t i=0, iend=mpCurrentFrame->mvpMapPoints.size(); i<iend;i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            MapPoint* pMP = mpCurrentFrame->mvpMapPoints[i];
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mpCurrentFrame->mvpMapPoints[i]=NULL;
            }
        }
    }

    int max=0;
    KeyFrame* pKFmax=NULL;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mpCurrentFrame->mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mpCurrentFrame->mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                    break;
                }
            }
        }

    }

    mpReferenceKF = pKFmax;
}

// update the temporal window,
// construct the spatial window of S keyframes excluding keyframes that are in the temporal window
// mark first estimates for variables
// A map point will be marked as having first estimate as soon as
// the last keyframe that observes it slips out of the spatial window
void Tracking::UpdateReferenceKeyFramesAndPoints()
{
    struct CompareKeyframe
    {
        bool operator()(std::pair<int, void*> a, std::pair<int, void*> b)
        {
            return a.first<b.first;
        }
    };

    //construct spatial window based on MapPoints in a Frame
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> frameCounter;

    for(auto it=mpCurrentFrame->mvpMapPoints.begin(),
        ite= mpCurrentFrame->mvpMapPoints.end();it!=ite; ++it)
    {
        MapPoint* pMP = *it;
        if(pMP)
        {
            if(!pMP->isBad())
            {
                map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::iterator it=observations.begin(), itend=observations.end(); it!=itend; ++it)
                    ++frameCounter[it->first];
            }
            else
                (*it)=NULL;
        }
    }

    std::priority_queue<std::pair<int, KeyFrame*>, vector<std::pair<int, KeyFrame *> >, CompareKeyframe > swappedCounter;
    for(map<KeyFrame*,int>::iterator it=frameCounter.begin(), itEnd=frameCounter.end(); it!=itEnd; it++)
    {
        swappedCounter.push(std::make_pair(it->second, (KeyFrame*)(it->first)));
    }
    if(!swappedCounter.empty())
        mpReferenceKF=swappedCounter.top().second;
    mvpOldLocalKeyFrames=mvpLocalKeyFrames;
    for(vector<KeyFrame*>::iterator iter=mvpOldLocalKeyFrames.begin();iter!=mvpOldLocalKeyFrames.end();++iter)
        (*iter)->SetErase(DoubleWindowKF);
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(mnSpatialWinSize);

    for(deque<Frame*>::iterator qIt=mvpTemporalFrames.begin(), qEndIt= mvpTemporalFrames.end();
        qIt!=qEndIt; ++qIt)
    {
        if((*qIt)->isKeyFrame())
            ((KeyFrame*)(*qIt))->mnTrackReferenceForFrame= mpCurrentFrame->mnId;
        //note keyframes in temporal window are set not erase in their creation
    }
    // S keyframes that observe a map point are included in the local map.
    int numKeyFrames=0;
    while(!swappedCounter.empty()){
        std::pair<int, KeyFrame*> it= swappedCounter.top();
        swappedCounter.pop();
        if(it.second->mnTrackReferenceForFrame == mpCurrentFrame->mnId){//exclude keyframes in the temporal window
            continue;
        }
        else{
            KeyFrame* pKF = it.second;
            if(pKF->isBad())
                continue;
            mvpLocalKeyFrames.push_back(pKF);
            pKF->SetNotErase(DoubleWindowKF);
            pKF->mnTrackReferenceForFrame = mpCurrentFrame->mnId;
            ++numKeyFrames;
            if(numKeyFrames==mnSpatialWinSize)
                break;
        }
    }

    if(numKeyFrames<mnSpatialWinSize){
        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(4);
            for(vector<KeyFrame*>::iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
            {
                KeyFrame* pNeighKF = *itNeighKF;
                if(!pNeighKF->isBad() && (pNeighKF->mnTrackReferenceForFrame!=mpCurrentFrame->mnId))
                {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->SetNotErase(DoubleWindowKF);
                        pNeighKF->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                        ++numKeyFrames;
                        break;//Huai: at most one is chosen
                }
            }
            if(numKeyFrames== mnSpatialWinSize)
                break;
        }
    }
    assert(numKeyFrames<=mnSpatialWinSize);
    // given keyframes in temporal window, update map points which are to be searched for in the current frame
    mvpLocalMapPoints.clear();
    mvpLocalMapPoints.reserve(mnFeatures);
    for(deque<Frame*>::iterator itF=mvpTemporalFrames.begin(), itEndF=mvpTemporalFrames.end(); itF!=itEndF; ++itF)
    {
        if(!((*itF)->isKeyFrame())) continue;
        vector<MapPoint*> vpMPs = ((KeyFrame*)(*itF))->GetMapPointMatches();

        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; ++itMP)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mpCurrentFrame->mnId){
                ++pMP->mnObservationsInDoubleWindow;
                continue;
            }
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                pMP->mnObservationsInDoubleWindow=1;
            }
        }
    }
    // given keyframes in spatial window, update map points which are to be searched for in the current frame
    for(vector<KeyFrame*>::iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
        for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mpCurrentFrame->mnId){
                ++pMP->mnObservationsInDoubleWindow;
                continue;
            }
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                pMP->mnObservationsInDoubleWindow=1;
            }
        }
    }

    // FEJ technique subjects to a constant condition: all MapPoints not observed by the double window are marked as having first estimate
    for(size_t jean=0; jean<mvpOldLocalKeyFrames.size(); ++jean){
        size_t kate=0;
        for(; kate<mvpLocalKeyFrames.size(); ++kate){
            if(mvpOldLocalKeyFrames[jean]== mvpLocalKeyFrames[kate])
                break;
        }
        if(kate==mvpLocalKeyFrames.size())//jean slips out of the spatial window
        {
            vector<MapPoint*> vpMPs = mvpOldLocalKeyFrames[jean]->GetMapPointMatches();

            for(vector<MapPoint*>::iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; ++itMP)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnTrackReferenceForFrame==mpCurrentFrame->mnId)
                    continue;
                if(!pMP->isBad() )
                {
                    pMP->SetFirstEstimate();
                }
            }
        }
    }
    if(mvpTemporalFrames.size()==1)//because at the beginning there are two keyframes of which only one in temporal window
        mvpLocalKeyFrames.clear();
}

bool Tracking::Relocalisation()
{
    // Compute Bag of Words Vector
    mpCurrentFrame->ComputeBoW();

    // Relocalisation is performed when tracking is lost and forced at some stages during loop closing
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs;

    if(!RelocalisationRequested()){
        for(deque<Frame*>::iterator qIt= mvpTemporalFrames.begin(); qIt!=mvpTemporalFrames.end(); ++qIt)
        {
            if((*qIt)->isKeyFrame()==false){
                delete (*qIt);
            }
            else
                  ((KeyFrame*)(*qIt))->SetErase(DoubleWindowKF);
        }
        mvpTemporalFrames.clear();
        vpCandidateKFs= mpKeyFrameDB->DetectRelocalisationCandidates(mpCurrentFrame);
    }
    else // Forced Relocalisation: Relocate against local window around last keyframe
    {
        boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
        mbForceRelocalisation = false;
        vpCandidateKFs.reserve(10);
        vpCandidateKFs = mpLastKeyFrame->GetBestCovisibilityKeyFrames(9);
        vpCandidateKFs.push_back(mpLastKeyFrame);
    }

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(size_t i=0; i<vpCandidateKFs.size(); i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,*mpCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(*mpCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(size_t i=0; i<vpCandidateKFs.size(); i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                mpCurrentFrame->SetPose(Converter::toSE3d(Tcw));

                set<MapPoint*> sFound;

                for(size_t j=0; j<vbInliers.size(); j++)
                {
                    if(vbInliers[j])
                    {
                        mpCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mpCurrentFrame->mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                if(nGood<10)
                    continue;

                for(size_t io =0, ioend=mpCurrentFrame->mvbOutlier.size(); io<ioend; io++)
                    if(mpCurrentFrame->mvbOutlier[io])
                        mpCurrentFrame->mvpMapPoints[io]=NULL;

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(*mpCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(size_t ip =0, ipend=mpCurrentFrame->mvpMapPoints.size(); ip<ipend; ip++)
                                if(mpCurrentFrame->mvpMapPoints[ip])
                                    sFound.insert(mpCurrentFrame->mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(*mpCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                                for(size_t io =0; io<mpCurrentFrame->mvbOutlier.size(); io++)
                                    if(mpCurrentFrame->mvbOutlier[io])
                                        mpCurrentFrame->mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }
                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=30)//was 50
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mpCurrentFrame->mnId;
        return true;
    }
}
// forced and/or requested relocalization is only done by loop closing
void Tracking::ForceRelocalisation(const g2o::Sim3 Sneww2oldw)
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    mbForceRelocalisation = true;  
    mnLastRelocFrameId= mpCurrentFrame->mnId;
    mSneww2oldw= Sneww2oldw;
}

bool Tracking::RelocalisationRequested()
{
    boost::mutex::scoped_lock lock(mMutexForceRelocalisation);
    return mbForceRelocalisation;
}
void Tracking::Reset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = false;
        mbReseting = true;
    }
#ifdef SLAM_USE_ROS
    // Wait until publishers are stopped
    ros::Rate r(500);
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped)
                break;
        }
        r.sleep();
    }
#endif
    for(deque<Frame*>::iterator qIt= mvpTemporalFrames.begin(); qIt!=mvpTemporalFrames.end(); ++qIt)
    {
        if((*qIt)->isKeyFrame()==false)
            delete (*qIt);
    }
    mvpTemporalFrames.clear();
    delete mpCurrentFrame;
    mpCurrentFrame=NULL;
#ifndef MONO
    delete mpLastFrame;
#endif
    mpLastFrame=NULL;

    // Reset Local Mapping
    mpLocalMapper->RequestReset();
    // Reset Loop Closing
    mpLoopClosing->RequestReset();
    // Clear BoW Database
    mpKeyFrameDB->clear();
    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextKeyId = 0;
    Frame::nNextId = 0;
    mState = NOT_INITIALIZED;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbReseting = false;
    }
}

void Tracking::CheckResetByPublishers()
{
    bool bReseting = false;

    {
        boost::mutex::scoped_lock lock(mMutexReset);
        bReseting = mbReseting;
    }

    if(bReseting)
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbPublisherStopped = true;
    }
#ifdef SLAM_USE_ROS
    // Hold until reset is finished
    ros::Rate r(500);
#endif
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(!mbReseting)
            {
                mbPublisherStopped=false;
                break;
            }
        }
#ifdef SLAM_USE_ROS
        r.sleep();
#else
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
#endif
    }
}
bool Tracking::isInTemporalWindow(const Frame* pFrame)const
{
    return (pFrame->mnId>=mvpTemporalFrames.front()->mnId&&
            pFrame->mnId<=mvpTemporalFrames.back()->mnId);
}
} //namespace ORB_SLAM
