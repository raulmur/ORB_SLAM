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

#include "global.h" //performance monitor

#include <opencv2/core/eigen.hpp>
#include<opencv2/opencv.hpp>

#ifdef SLAM_USE_ROS
#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
#endif

#include "vio/timegrabber.h"

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

extern string slamhome;

namespace ORB_SLAM
{
#ifdef SLAM_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

//remove matches that have u1c falls outside of [xl, xr)
std::vector<p_match> cropMatches(const std::vector<p_match> &pMatches, float xl, float xr)
{
    std::vector<p_match> resMatches;
    resMatches.reserve(pMatches.size());
    for(auto it= pMatches.begin(), ite= pMatches.end(); it!=ite; ++it)
    {
        if(it->u1c<xl || it->u1c>=xr)
            continue;
        else
            resMatches.push_back(*it);
    }
    return resMatches;
}

static bool to_bool(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::istringstream is(str);
    bool b;
    is >> std::boolalpha >> b;
    return b;
}

#ifdef SLAM_USE_ROS
Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher*pFramePublisher, MapPublisher *pMapPublisher, Map *pMap, string strSettingPath):
    mState(NO_IMAGES_YET),mpCurrentFrame(NULL),mpLastFrame(NULL), mpInitialFrame(NULL),
    initVwsBaBg(Eigen::Matrix<double, 9, 1>::Zero()),
    mpORBVocabulary(pVoc), mpInitializer(NULL),
    mnTemporalWinSize(Config::temporalWindowSize()),mnSpatialWinSize(Config::spatialWindowSize()),
    mpFramePublisher(pFramePublisher), mpMapPublisher(pMapPublisher), mpMap(pMap),
#else
Tracking::Tracking(ORBVocabulary* pVoc, FramePublisher*pFramePublisher, /*MapPublisher *pMapPublisher,*/ Map *pMap, string strSettingPath):
    mState(NO_IMAGES_YET),mpCurrentFrame(NULL),mpLastFrame(NULL), mpInitialFrame(NULL),
    initVwsBaBg(Eigen::Matrix<double, 9, 1>::Zero()),
    mpORBVocabulary(pVoc), mpInitializer(NULL),
    mnTemporalWinSize(Config::temporalWindowSize()),mnSpatialWinSize(Config::spatialWindowSize()),
    mpFramePublisher(pFramePublisher),/*mpMapPublisher(pMapPublisher),*/ mpMap(pMap),
#endif
    mStrSettingFile(strSettingPath),
    mfsSettings(strSettingPath, cv::FileStorage::READ), mpLastKeyFrame(NULL),
    mnLastRelocFrameId(0), mbPublisherStopped(false), mbReseting(false), mbForceRelocalisation(false),
    mVelocity(Sophus::SE3d()), mVelByStereoOdometry(Eigen::Vector3d::Zero()),
    mbUseIMUData(false), mnFrameIdOfSecondKF(0), mnFeatures(mfsSettings["ORBextractor.nFeatures"]),
    mMotionModel(Eigen::Vector3d(0,0,0),Eigen::Quaterniond(1,0,0,0)),
    mpImuProcessor(NULL),
    mfTrackedFeatureRatio(0.6), mnMinTrackedFeatures(200)
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
    string trace_dir = slamhome + (std::string)mfsSettings["trace_dir"];
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
    // assume the stereo images are synchronized and rectified
    cv::FileNode matTr2l = mfsSettings["Stereo.se3Right2Left"];
    cv::FileNode matTl2r = mfsSettings["Stereo.se3Left2Right"];
    cv::Mat T_l2r; //float type
    if(!matTl2r.empty()) {
        matTl2r>>T_l2r;
    }
    else
    {
        assert(!matTr2l.empty());
        matTr2l>>T_l2r;
        cv::Mat R_l2r = T_l2r.colRange(0,3).rowRange(0,3).t();
        T_l2r.col(3).rowRange(0,3)= - R_l2r*T_l2r.col(3).rowRange(0,3);
        T_l2r.colRange(0,3).rowRange(0,3)= R_l2r;
    }

    mTl2r= Converter::toSE3d(T_l2r);
    assert(distCoef.lpNorm<Eigen::Infinity>()< 1e-8);
    right_cam_ = new vk::PinholeCamera(width, height,
                                       fx, fy, cx, cy,
                                       distCoef(0), distCoef(1),
                                       distCoef(2),distCoef(3));

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

        imu_.q_n_aw_babw.head<3>()=q_noise_acc;
        imu_.q_n_aw_babw.segment<3>(3)=q_noise_gyr;
        imu_.q_n_aw_babw.segment<3>(6)=q_noise_accbias;
        imu_.q_n_aw_babw.tail<3>()=q_noise_gyrbias;

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
        imu_.gwomegaw.head<3>()=ginw;
        cv::cv2eigen(omegaew, tempVec3d);
        imu_.gwomegaw.tail<3>()=tempVec3d;
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
    if(mfsSettings["qcv_tracks"].isString() && mfsSettings["qcv_deltas"].isString())
        mStereoSFM.init(mfsSettings["qcv_tracks"], mfsSettings["qcv_deltas"]);

    // ORB parameters
    float fScaleFactor = mfsSettings["ORBextractor.scaleFactor"];
    int nLevels = mfsSettings["ORBextractor.nLevels"];
    int fastTh = mfsSettings["ORBextractor.fastTh"];
    int Score = mfsSettings["ORBextractor.nScoreType"];
    assert(Score==1 || Score==0);

    float sigmaLevel0 = 1.0f;
    if(mfsSettings["ORBextractor.sigmaLevel0"].isReal()) //"'ORBextractor.sigmaLevel0' parameter available in configuration file."
        sigmaLevel0 = mfsSettings["ORBextractor.sigmaLevel0"];

    mpORBextractor = new ORBextractor(mnFeatures,fScaleFactor,nLevels,Score,fastTh,sigmaLevel0);

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
    mpIniORBextractor = new ORBextractor(mnFeatures*2,1.2,8,Score,fastTh, sigmaLevel0);

    if(mfsSettings["Tracking.tracked_feature_ratio"].isReal())
        mfTrackedFeatureRatio = mfsSettings["Tracking.tracked_feature_ratio"];
    if(mfsSettings["Tracking.min_tracked_features"].isInt())
        mnMinTrackedFeatures = mfsSettings["Tracking.min_tracked_features"];

    string dataset=mfsSettings["dataset"];
    if (dataset.compare("KITTIOdoSeq")==0)
        experimDataset =KITTIOdoSeq;
    else if (dataset.compare("Tsukuba")==0)
        experimDataset=Tsukuba;
    else if(dataset.compare("MalagaUrbanExtract6")==0)
        experimDataset=MalagaUrbanExtract6;
    else if(dataset.compare("CrowdSourcedData")==0)
        experimDataset=CrowdSourcedData;
    else if(dataset.compare("DiLiLi")==0)
        experimDataset=DiLiLi;
    else{
        std::cerr<<"Unsupported dataset:"<<dataset<<endl;
    }

    PrepareImuProcessor();

    cv::FileNode T_Wc_W_ = mfsSettings["T_Wc_W"];
    if(T_Wc_W_.isSeq()) {
      Eigen::Matrix4d T_Wc_W_e;
      T_Wc_W_e << T_Wc_W_[0], T_Wc_W_[1], T_Wc_W_[2], T_Wc_W_[3],
                  T_Wc_W_[4], T_Wc_W_[5], T_Wc_W_[6], T_Wc_W_[7],
                  T_Wc_W_[8], T_Wc_W_[9], T_Wc_W_[10], T_Wc_W_[11],
                  T_Wc_W_[12], T_Wc_W_[13], T_Wc_W_[14], T_Wc_W_[15];

      mT_Wc_W = Sophus::SE3d(T_Wc_W_e);
      std::stringstream s;
      s << mT_Wc_W.matrix3x4();
      cout << "Custom World frame provided T_Wc_W=\n" << s.str() << endl;
    }

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
            Vector3d xn1((kp1.pt.x-mpLastKeyFrame->cam_.cx())/mpLastKeyFrame->cam_.fx(),
                         (kp1.pt.y-mpLastKeyFrame->cam_.cy())/mpLastKeyFrame->cam_.fy(), 1.0 ),
                    ray1(xn1);

            Vector3d xn3((kp3.pt.x-pCurrFrame->cam_.cx())/pCurrFrame->cam_.fx(),
                         (kp3.pt.y-pCurrFrame->cam_.cy())/pCurrFrame->cam_.fy(), 1.0 );

            Vector3d ray3 = Rw2c1*Rw2c2.transpose()*xn3;
            float cosParallaxRays = ray1.dot(ray3)/(ray1.norm()*ray3.norm());

            if((cosParallaxRays<0 || cosParallaxRays> Config::triangMaxCosRays())
                    && (kp1.pt.x -kp2.pt.x< Config::triangMinDisp()))//parallax
            {
                ++rejectHisto[1];
                    continue;
            }
            // Linear Triangulation Method
            Vector3d xn2((kp2.pt.x-mpLastKeyFrame->right_cam_.cx())/mpLastKeyFrame->right_cam_.fx(),
                         (kp2.pt.y-mpLastKeyFrame->right_cam_.cy())/mpLastKeyFrame->right_cam_.fy(), 1.0 );

            Vector3d xn4((kp4.pt.x-pCurrFrame->right_cam_.cx())/pCurrFrame->right_cam_.fx(),
                         (kp4.pt.y-pCurrFrame->right_cam_.cy())/pCurrFrame->right_cam_.fy(), 1.0 );

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
            float u1 = mpLastKeyFrame->cam_.fx()*x1*invz1+mpLastKeyFrame->cam_.cx();
            float v1 = mpLastKeyFrame->cam_.fy()*y1*invz1+mpLastKeyFrame->cam_.cy();
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
            float u2 = pCurrFrame->cam_.fx()*x2*invz2+pCurrFrame->cam_.cx();
            float v2 = pCurrFrame->cam_.fy()*y2*invz2+pCurrFrame->cam_.cy();
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
    SLAM_DEBUG_STREAM("created new points "<<numNewPoints<<" from quad matches "<<vQuadMatches.size());
    if(numNewPoints< 6)
        SLAM_DEBUG_STREAM("Rejected quad matches for fullCell, cosRay, SVD0, PDOP, negZ, reproj,"
                          " distZero, distRatio "<< rejectHisto.transpose());
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
    const float fx1 = pCurrentKeyFrame->cam_.fx();
    const float fy1 = pCurrentKeyFrame->cam_.fy();
    const float cx1 = pCurrentKeyFrame->cam_.cx();
    const float cy1 = pCurrentKeyFrame->cam_.cy();
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;
    // const float ratioFactor = 1.5f*pCurrentKeyFrame->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    // Check first that baseline is not too short
    // Small translation errors for short baseline keyframes make scale to diverge
    Eigen::Vector3d Ow2 = pKF2->GetCameraCenter();
    Eigen::Vector3d vBaseline = Ow2-Ow1;
    const float baseline = vBaseline.norm();
    const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);


    if(baseline/medianDepthKF2<0.002)//ratio between Baseline and Depth
    {
        SLAM_ERROR_STREAM("too small baseline between kfs "<<baseline<< " "<< medianDepthKF2);
    }

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

    const float fx2 = pKF2->cam_.fx();
    const float fy2 = pKF2->cam_.fy();
    const float cx2 = pKF2->cam_.cx();
    const float cy2 = pKF2->cam_.cy();
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
    SLAM_DEBUG_STREAM("created new points in cur KF "<<counter<<" out of matches:"<<vMatchedKeysUn1.size());
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
            if(mbUseIMUData){
#ifdef SLAM_DEBUG_OUTPUT
             SLAM_INFO_STREAM("Please define the distance between the first two keyframes which is to determine the proper scale");
             Vector3d initTrans(0.048, -0.057, 0.034);
             scale= initTrans.norm()*2;
#endif
            }
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
                SLAM_DEBUG_STREAM("last frame and curr frame index in image sequence:"<< mpLastFrame->mnId <<" "<<mpCurrentFrame->mnId);
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
                mpLastFrame->PartialRelease();
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
                             const Sophus::SE3d *pred_Tr_delta, Eigen::Matrix<double, 9,1> sb)
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

    double lastFrameTime = mpLastFrame==NULL? -1:mpLastFrame->mTimeStamp;
    //compute ORB descriptors of vStereoMatches
    mpCurrentFrame=new Frame(im, timeStampSec, mStereoSFM.getNumDenseFeatures(),
                                   right_img, mStereoSFM.getNumDenseFeatures(),
                                   vStereoMatches, mpORBextractor, mpORBVocabulary, cam_, right_cam_,
                                   mTl2r, ginc, sb);

    // also test whether a quad match satisfy stereo matches
    if(mpLastFrame!=NULL)
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
                 SLAM_DEBUG_STREAM("last frame and curr frame index in image sequence:"<< mpLastFrame->mnId <<" "<<mpCurrentFrame->mnId);
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
                if(mpLastFrame->mnId== mnFrameIdOfSecondKF){
                    mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
                    pLastFrame= mpLastKeyFrame;
                }
                else
                    pLastFrame= new Frame(*mpLastFrame);
            }
            else{
                assert(mpLastFrame->mnId> mnFrameIdOfSecondKF);//because we added second frame as keyframe
                mpLastFrame->PartialRelease();
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

        if(mpLastFrame->mnId!=mnFrameIdOfSecondKF) //delay deleting the second keyframe until sliding out of window
            delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }   
    // Update drawer
    mpFramePublisher->Update(this, im);

    if(mpCurrentFrame!=NULL){

        Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
#ifdef SLAM_USE_ROS
        Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
        tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                        Rwc(1,0),Rwc(1,1),Rwc(1,2),
                        Rwc(2,0),Rwc(2,1),Rwc(2,2));
        tf::Vector3 V(twc(0), twc(1), twc(2));
        tf::Transform tfTcw(M,V);
        mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World",
                                                 "ORBSLAM_DWO/Camera"));
#endif
        if(lastFrameTime == -1)
            mVelByStereoOdometry = Eigen::Vector3d::Zero();
        else
            mVelByStereoOdometry = mVelByStereoOdometry*0.4 + 0.6*Rwc*Tcp.translation()
                    /(timeStampSec - lastFrameTime);

    }
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
                             const Sophus::SE3d *pred_Tr_delta,Eigen::Matrix<double, 9,1> sb)
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
    //I believe the reason is prior motion is not necessary in feature matching. E.g.,qcv stereoSFM did not use such motion prior to aid feature matching
    // but when features are matched to points in local map, Stereo PTAM used a prior motion.
    // On the other hand, prior motion should be helpful in initializing pose optimization
    if (mVisoStereo.Tr_valid) mVisoStereo.matcher->matchFeatures(2,&mVisoStereo.Tr_delta);
    else          mVisoStereo.matcher->matchFeatures(2);

    vector<p_match> vQuadMatches = cropMatches( mVisoStereo.matcher->getMatches(), Config::cropROIXL(), Config::cropROIXR());
    libviso2::VisualOdometryStereo::parameters param=mVisoStereo.getParameters();
    mVisoStereo.matcher->bucketFeatures(param.bucket.max_features, param.bucket.bucket_width,param.bucket.bucket_height);
    vector<p_match> p_matched = cropMatches(mVisoStereo.matcher->getMatches(), Config::cropROIXL(), Config::cropROIXR());
    SLAM_STOP_TIMER("extract_quadmatches");

    SLAM_START_TIMER("track_previous_frame");
    vector<double> tr_delta = mVisoStereo.transformationMatrixToVector (Converter::toViso2Matrix(Tcp));
    const enum Optimizer {RANSAC_Geiger, RANSAC_5Point, ROBUST_Klein} approach=RANSAC_Geiger;
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
        if(mpCurrentFrame){// not the first frame

            // hack starts
            //            mState= LOST;
            if(vQuadMatches.size()< 9){
                mState= LOST;
                cout<<"libviso2 odometry failed for images at time:"<<timeStampSec
                <<" of quad matches:"<<vQuadMatches.size()<<endl;
            }
            else
            {
                mVisoStereo.Tr_valid = true;
                mVisoStereo.Tr_delta = libviso2::Matrix::eye(4);
            }
            //hack ends
        }
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

    double lastFrameTime = mpLastFrame==NULL? -1:mpLastFrame->mTimeStamp;
    //compute ORB descriptors of vStereoMatches
    mpCurrentFrame=new Frame(im, timeStampSec, mVisoStereo.matcher->getNumDenseFeatures(true),
                                   right_img, mVisoStereo.matcher->getNumDenseFeatures(false),
                                   vStereoMatches, mpORBextractor, mpORBVocabulary, cam_, right_cam_,
                                   mTl2r, ginc, sb);

    // also test whether a quad match satisfy stereo matches
    if(mpLastFrame!=NULL)
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
        if(vQuadMatches.size()<=10)
        {
            SLAM_WARN_STREAM("Restart initialing as the first two stereo frames have too few quad matches "<< vQuadMatches.size());
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
                if(mpLastFrame->mnId==mnFrameIdOfSecondKF){
                    mpLocalMapper->InsertKeyFrame(mpLastKeyFrame);
                    pLastFrame= mpLastKeyFrame;
                }
                else
                    pLastFrame= new Frame(*mpLastFrame);
            }
            else{
                assert(mpLastFrame->mnId>mnFrameIdOfSecondKF);//because we added second frame as keyframe
                mpLastFrame->PartialRelease();
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
            if(mpMap->KeyFramesInMap()<=10)
            {
                std::cout<<"Reset called once lost as mpMap does not have enough keyframes "<<
                           mpMap->KeyFramesInMap()<<std::endl;
                Reset();
                return;
            }
        }
        if(mpLastFrame->mnId!=mnFrameIdOfSecondKF) //delay deleting the second keyframe until sliding out of window
            delete mpLastFrame;
        mpLastFrame = mpCurrentFrame;
    }  
    // Update drawer
    mpFramePublisher->Update(this, im);
    if(mpCurrentFrame!=NULL){

        Eigen::Matrix3d Rwc = mpCurrentFrame->mTcw.rotationMatrix().transpose();
#ifdef SLAM_USE_ROS
        Eigen::Vector3d twc = -Rwc*mpCurrentFrame->mTcw.translation();
        tf::Matrix3x3 M(Rwc(0,0),Rwc(0,1),Rwc(0,2),
                        Rwc(1,0),Rwc(1,1),Rwc(1,2),
                        Rwc(2,0),Rwc(2,1),Rwc(2,2));
        tf::Vector3 V(twc(0), twc(1), twc(2));
        tf::Transform tfTcw(M,V);
        mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORBSLAM_DWO/World",
                                                 "ORBSLAM_DWO/Camera"));
#endif
        if(lastFrameTime == -1)
            mVelByStereoOdometry = Eigen::Vector3d::Zero();
        else
            mVelByStereoOdometry = mVelByStereoOdometry*0.4 + 0.6*Rwc*Tcp.translation()
                    /(timeStampSec - lastFrameTime);

    }


}

void Tracking::FirstInitialization()
{    
    //We ensure a minimum ORB features to continue, otherwise discard frame
    if(mpCurrentFrame->mvKeysUn.size()<=20)
    {
        SLAM_DEBUG_STREAM("Skip first frame has too few keypoints: "<<mpCurrentFrame->mvKeysUn.size());
    }
    else
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

        mPose=libviso2::Matrix::eye(4);
        mpInitializer =  new Initializer(*mpInitialFrame,1.0,200);
        mState = INITIALIZING;
        //SLAM_DEBUG_STREAM(" Added to map the very first keyframe with #kp "<<mpCurrentFrame->mvKeysUn.size());
        std::cout<<" Added to map the very first keyframe with #kp "<<mpCurrentFrame->mvKeysUn.size()<<std::endl;
    }
}

// only used to initialize monocular pipeline
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

    const float fx1 = pKFcur->cam_.fx();
    const float fy1 = pKFcur->cam_.fy();
    const float cx1 = pKFcur->cam_.cx();
    const float cy1 = pKFcur->cam_.cy();

    proxy= pPrevFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c2= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c2= Tw2c2.topLeftCorner<3,3>();
    Eigen::Vector3d twinc2=Tw2c2.col(3);

    const float fx2 = pPrevFrame->cam_.fx();
    const float fy2 = pPrevFrame->cam_.fy();
    const float cx2 = pPrevFrame->cam_.cx();
    const float cy2 = pPrevFrame->cam_.cy();

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
            Vector3d xn2((kp2.pt.x-pKFcur->right_cam_.cx())/pKFcur->right_cam_.fx(),
                         (kp2.pt.y-pKFcur->right_cam_.cy())/pKFcur->right_cam_.fy(), 1.0 );

            Vector3d xn4((kp4.pt.x-pPrevFrame->right_cam_.cx())/pPrevFrame->right_cam_.fx(),
                         (kp4.pt.y-pPrevFrame->right_cam_.cy())/pPrevFrame->right_cam_.fy(), 1.0 );

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

    //    Optimizer::GlobalBundleAdjustemnt(mpMap,20);
    float medianDepth = pKFcur->ComputeSceneMedianDepth(2);
    int trackedMPs = pKFcur->TrackedMapPoints();
    if(medianDepth<0 || trackedMPs<Config::initMinTracked())
    {
//        SLAM_DEBUG_STREAM("Wrong initialization for stereo images with median depth "<< medianDepth <<
//                          " and tracked mp in second kf "<<trackedMPs<< ". Reseting...");
        std::cerr<<"Wrong initialization for stereo images with median depth "<< medianDepth <<
                          " and tracked mp in second kf "<<trackedMPs<< ". Reseting..."<<std::endl;
        Reset();
        return;
    }else
    {
        //    SLAM_DEBUG_STREAM("created initial map of points from quad matches:"<<mpMap->MapPointsInMap()<<" "<<vQuadMatches.size());
          std::cout<< "created initial map of points from quad matches:"<<mpMap->MapPointsInMap()
                   <<" "<<vQuadMatches.size()<<std::endl;
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
    mnFrameIdOfSecondKF = mpLastFrame->mnId;
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
    mnFrameIdOfSecondKF = mpLastFrame->mnId;
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
        Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
    Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
    Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
    mnMatchesInliers = Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
//        mnMatchesInliers = Optimizer::PoseOptimization(mpCurrentFrame, mpMap);
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        mnMatchesInliers= nObs;
        SLAM_DEBUG_STREAM("Inliers after local search:"<< mnMatchesInliers);
    }
    else{
        int nBad = Optimizer::LocalOptimize(cam_, mpMap, mvpLocalKeyFrames,
                                            mvpLocalMapPoints, mvpTemporalFrames,
                                            mpCurrentFrame, mpLastFrame, mbUseIMUData?&imu_:NULL, right_cam_, &mTl2r);

        mnMatchesInliers= nObs-nBad;
        SLAM_DEBUG_STREAM("Inliers after DWO :"<< mnMatchesInliers<<" and bad obs "<<nBad);
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
    // Huai: is it possible to process new keyframes when loop closing is running? But it would be quite involved
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
    const float fPercent = 0.9;
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
    const float fPercent = mfTrackedFeatureRatio;
    const bool c2 = (mnMatchesInliers< nRefMatches*fPercent) && mnMatchesInliers>15 && mnMatchesInliers < mnMinTrackedFeatures;
    int num_matchless_cells= point_stats.numFeatureLessCorners3x3(4);
    // need KF if 4/9 of the image is void of matches as in ScaViSLAM
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

    std::vector<std::shared_ptr<PnPsolver> > vpPnPsolvers;
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
                vpPnPsolvers[i].reset(pSolver);
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

            std::shared_ptr<PnPsolver> pSolver = vpPnPsolvers[i];
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

                int nGood = Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
                        nGood = Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
                                nGood = Optimizer::PoseOptimization(mpCurrentFrame, mpMap);

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
    ros::Rate r(50);
    size_t ticks = 0;
    size_t maxWaitTicks = 50*0.1; //  how long do we wait for user input, 50x number of secs.
    while(1)
    {
        {
            boost::mutex::scoped_lock lock(mMutexReset);
            if(mbPublisherStopped || ticks> maxWaitTicks)
            {
                mbPublisherStopped = true;
                break;
            }
        }
        r.sleep();
        ++ticks;
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

    std::cout <<"system reset finishes "<< std::endl;
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

void Tracking::GetLastestPoseEstimate(TrackingResult & rhs) const{
    rhs.status_ = mState;
    if(mpLastFrame!=NULL)
        rhs.timestamp_ = mpLastFrame->mTimeStamp;
    else
        rhs.timestamp_ = -1;
    if(mState == WORKING){
        rhs.T_Wc_C_ = mT_Wc_W*mpLastFrame->GetPose().inverse();
        rhs.vwsBaBg_ = mpLastFrame->speed_bias;
        rhs.vwsBaBg_.head<3>() = mT_Wc_W.rotationMatrix()*rhs.vwsBaBg_.head<3>();
    }
}

void Tracking::GetViso2PoseEstimate(TrackingResult & rhs) const{

    if(mpLastFrame!=NULL)
        rhs.timestamp_ = mpLastFrame->mTimeStamp;
    else
        rhs.timestamp_ = -1;

    if(mVisoStereo.Tr_valid){
        rhs.status_ = WORKING;
        rhs.T_Wc_C_ = mT_Wc_W*Converter::toSE3d(mPose);        
    }else
        rhs.status_ = LOST;
    rhs.vwsBaBg_.setZero();
}

bool Tracking::ProcessAMonocularFrame(cv::Mat &left_img, double time_frame,
                                      const RawImuMeasurementVector & imuMeas){
    if(left_img.cols != cam_->width() || left_img.rows!= cam_->height())
    {
        cerr<<"Incompatible image size, check setting file Camera.width .height fields or the end of video!"<<endl;
        return false;
    }
    Sophus::SE3d predTcp; //predicted transformation from previous camera frame to current camera frame
    if(mbUseIMUData){
        if(!mpImuProcessor->bStatesInitialized){
            mpImuProcessor->initStates(initTws, initVwsBaBg, time_frame);
            ProcessFrameMono(left_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                                 NULL, initVwsBaBg);
        }
        else{
            predTcp=mpImuProcessor->propagate(time_frame, imuMeas);
            ProcessFrameMono(left_img, time_frame,
                             imuMeas, &predTcp, mpImuProcessor->speed_bias_1);

            if(mState == WORKING){
                mpImuProcessor->resetStates((imu_.T_imu_from_cam*mpLastFrame->mTcw).inverse(), mpLastFrame->speed_bias);
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
    return true;
}

bool Tracking::ProcessAStereoFrame(cv::Mat &left_img, cv::Mat &right_img, double time_frame,
                                   const RawImuMeasurementVector & imuMeas){
    if(left_img.cols != cam_->width() || left_img.rows!= cam_->height())
    {
        cerr<<"Incompatible image size, check setting file Camera.width .height fields!"<<endl;
        return false;
    }
    // either processframe or processframeQCV is supposed to work in this function
    Sophus::SE3d predTcp; //predicted transformation from previous camera frame to current camera frame
    if(mbUseIMUData){
        if(!mpImuProcessor->bStatesInitialized){
            mpImuProcessor->initStates(initTws, initVwsBaBg, time_frame);
            ProcessFrame(left_img, right_img, time_frame, std::vector<Eigen::Matrix<double, 7,1 > >(),
                             NULL, initVwsBaBg);
        }
        else{
            predTcp=mpImuProcessor->propagate(time_frame, imuMeas);

            Eigen::Matrix<double, 9, 1> velAndBiases = mpImuProcessor->speed_bias_1;
            if(experimDataset == DiLiLi)
                //Hack: it is observed that imu prediction may be worse than stereo pose differention
                velAndBiases.head<3>() = mVelByStereoOdometry;

            ProcessFrame(left_img, right_img, time_frame,
                         imuMeas, &predTcp, velAndBiases);
        }
        if(mState == WORKING){
            mpImuProcessor->resetStates((imu_.T_imu_from_cam*mpLastFrame->mTcw).inverse(), mpLastFrame->speed_bias);
        }else if(mState == NOT_INITIALIZED){ //this occurs when tracking is reset
            mpImuProcessor->initStates(initTws, initVwsBaBg, time_frame); //TODO: this assumes the velocity eqauls to the specified value, use an initialization procedure is preferred
            std::cout <<"imu proc reset at frame time "<< time_frame <<std::endl;
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
    return true;
}

void Tracking::PrepareImuProcessor(){   
    if(mbUseIMUData){
        mpImuProcessor=new vio::IMUProcessor(imu_);

        //initialize IMU states
        initTws=imu_.T_imu_from_cam.inverse();
        cv::Mat vs0inw;
        cv::FileNode vsinw = mfsSettings["vs0inw"];
        if(vsinw.isMap()){
            mfsSettings["vs0inw"]>>vs0inw;
            std::cout << "vs0inw "<<vs0inw.at<double>(0)<< " "<<vs0inw.at<double>(1)<<" "<<vs0inw.at<double>(2)<<std::endl;
        }
        else
            std::cerr<< "vs0inw(velocity of sensor in the world frame) is needed in the setting file"<< std::endl;

        initVwsBaBg[0]=vs0inw.at<double>(0);
        initVwsBaBg[1]=vs0inw.at<double>(1);
        initVwsBaBg[2]=vs0inw.at<double>(2);
    }
}

void Tracking::ResizeCameraModel(const int downscale){
    vk::PinholeCamera* oldCam = cam_;
    cam_ = new vk::PinholeCamera( oldCam->width()/downscale, oldCam->height()/downscale, oldCam->fx()/downscale,
                                  oldCam->fy()/downscale, oldCam->cx()/downscale, oldCam->cy()/downscale,
                                  oldCam->d0(), oldCam->d1(), oldCam->d2(), oldCam->d3());
    delete oldCam;
}

ostream& operator<<(ostream& os, const TrackingResult& tr)
{
    if(tr.status_ == TrackingState::WORKING){
        Eigen::Matrix<double,4,1> q = tr.T_Wc_C_.unit_quaternion().coeffs();
        Eigen::Vector3d t = tr.T_Wc_C_.translation();
        os << setprecision(6) << tr.timestamp_ << " " << t.transpose()<<
              " " << q.transpose() <<" "<< tr.vwsBaBg_.transpose();
    }
    return os;
}


} //namespace ORB_SLAM
