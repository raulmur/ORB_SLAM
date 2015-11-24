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

#include "LocalMapping.h"
#include "config.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <opencv2/core/eigen.hpp>
#ifdef SLAM_USE_ROS
#include <ros/ros.h>
#endif

namespace ORB_SLAM
{
LocalMapping::LocalMapping(Map *pMap):
    mbResetRequested(false), mpMap(pMap),  mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

// if loop closing is requesting local mapping to stop or the local mapping is suspended
// the trackiing thread will not insert new keyframes, but there may still be keyframes in
// the waitlist, i.e., mlNewKeyFrames, they will be deleted once loop closing release the hold on local mapping
void LocalMapping::Run()
{
#ifdef SLAM_USE_ROS
    ros::Rate r(300);
    while(ros::ok())
#else
    while(1)
#endif
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            SLAM_START_TIMER("local_mapper");
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints between neighboring keyframes and this new keyframe
            if(mpCurrentKeyFrame->mnFrameId>1){
#ifdef MONO
            CreateNewMapPoints();
#else
            CreateNewMapPointsStereo();            
#endif
            }
            // Find more matches in neighbor keyframes and fuse point duplications
            SearchInNeighbors();

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
            //    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA);
                // huai: local BA not necessary when double window optimization is done in the tracking thread

                // Check redundant local Keyframes
                KeyFrameCulling();

                mpMap->SetFlagAfterBA();

                // Tracking will see Local Mapping idle
                if(!CheckNewKeyFrames())
                    SetAcceptKeyFrames(true);
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            SLAM_STOP_TIMER("local_mapper");
        }

        // Safe area to stop: stop the local mapping thread when loop closing is going on, restart it when loop closing finishes
        if(stopRequested())
        {
            Stop();
#ifdef SLAM_USE_ROS
            ros::Rate r2(1000);
            while(isStopped() && ros::ok())
            {                
                r2.sleep();
            }
#else
            while(isStopped())
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(5));
            }
#endif
            SetAcceptKeyFrames(true);
        }

        ResetIfRequested();
#ifdef SLAM_USE_ROS
        r.sleep();
#else
        boost::this_thread::sleep(boost::posix_time::milliseconds(2));
#endif
    }
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
    SetAcceptKeyFrames(false);
}


bool LocalMapping::CheckNewKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        boost::mutex::scoped_lock lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    if(mpCurrentKeyFrame->mnFrameId==0)
        return;

    // Associate MapPoints to the new keyframe and update normal and descriptor
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    if(mpCurrentKeyFrame->mnFrameId>1) //This operations are already done in the tracking for the first two keyframes and their newly created points
    {
#ifndef MONO
        mpCurrentKeyFrame->ComputeBoW();
#endif
        for(size_t i=0; i<vpMapPointMatches.size(); ++i)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad()) //&& pMP->mnFirstKFid != mpCurrentKeyFrame->mnFrameId)
                {
//                    pMP->AddObservation(mpCurrentKeyFrame, i);
//                    pMP->AddObservation(mpCurrentKeyFrame, i, false);//Huai:done in tracking thread
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
            }
        }
        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }
    for(size_t i=0; i<vpMapPointMatches.size(); ++i)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP && pMP->mnFirstKFid == mpCurrentKeyFrame->mnFrameId)
        {
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnFrameId;
    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
#ifdef MONO
        else if((nCurrentKFid-pMP->mnFirstKFid)>=2 && pMP->Observations()<=2)
#else
        else if((nCurrentKFid-pMP->mnFirstKFid)>=2 && pMP->Observations()<=1)// Huai changed from 2 to 1
#endif
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((nCurrentKFid-pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Take neighbor keyframes in covisibility graph
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(20);

    ORBmatcher matcher(0.6,false);

    Eigen::Matrix3d Rcw1 = mpCurrentKeyFrame->GetRotation();
    Eigen::Matrix3d Rwc1 = Rcw1.transpose();
    Eigen::Vector3d tcw1 = mpCurrentKeyFrame->GetTranslation();
    Eigen::Matrix<double,3,4> Tcw1;
    Tcw1.topLeftCorner(3,3)=Rcw1;
    Tcw1.col(3)= tcw1;
    Eigen::Vector3d Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float fx1 = mpCurrentKeyFrame->cam_->fx();
    const float fy1 = mpCurrentKeyFrame->cam_->fy();
    const float cx1 = mpCurrentKeyFrame->cam_->cx();
    const float cy1 = mpCurrentKeyFrame->cam_->cy();
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpNeighKFs[i];
        if(pKF2->mnFrameId+1==mpCurrentKeyFrame->mnFrameId)
            continue;
        // Check first that baseline is not too short
        // Small translation errors for short baseline keyframes make scale to diverge
        Eigen::Vector3d Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3d vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();
        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        Eigen::Matrix3d F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fulfil epipolar constraint
        vector<cv::KeyPoint> vMatchedKeysUn1;
        vector<cv::KeyPoint> vMatchedKeysUn2;
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedKeysUn1,vMatchedKeysUn2,vMatchedIndices);

        Eigen::Matrix3d Rcw2 = pKF2->GetRotation();
        Eigen::Matrix3d Rwc2 = Rcw2.transpose();
        Eigen::Vector3d tcw2 = pKF2->GetTranslation();
        Eigen::Matrix<double,3,4> Tcw2;
        Tcw2.topLeftCorner(3,3)=Rcw2;
        Tcw2.col(3)=tcw2;

        const float fx2 = pKF2->cam_->fx();
        const float fy2 = pKF2->cam_->fy();
        const float cx2 = pKF2->cam_->cx();
        const float cy2 = pKF2->cam_->cy();
        const float invfx2 = 1.0f/fx2;
        const float invfy2 = 1.0f/fy2;

        // Triangulate each match
        for(size_t ikp=0, iendkp=vMatchedKeysUn1.size(); ikp<iendkp; ikp++)
        {
            const int idx1 = vMatchedIndices[ikp].first;
            const int idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = vMatchedKeysUn1[ikp];
            const cv::KeyPoint &kp2 = vMatchedKeysUn2[ikp];
            int posX, posY;
            if(!mpCurrentKeyFrame->mpFG->IsPointEligible(kp1, posX, posY)) continue;

            // Check parallax between rays
            Eigen::Vector3d xn1((kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0 );
            Eigen::Vector3d ray1 = Rwc1*xn1;
            Eigen::Vector3d xn2((kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0 );
            Eigen::Vector3d ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm()*ray2.norm());

            if(cosParallaxRays<0 || cosParallaxRays>Config::triangMaxCosRays())
                continue;

            // Linear Triangulation Method
            Eigen::Matrix<double, 4,4> A;
            A.row(0) = xn1(0)*Tcw1.row(2)-Tcw1.row(0);
            A.row(1) = xn1(1)*Tcw1.row(2)-Tcw1.row(1);
            A.row(2) = xn2(0)*Tcw2.row(2)-Tcw2.row(0);
            A.row(3) = xn2(1)*Tcw2.row(2)-Tcw2.row(1);

            cv::Mat Aprime, w,u,vt;
            cv::eigen2cv(A,Aprime);
            cv::SVD::compute(Aprime,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

            cv::Mat x3D = vt.row(3).t();

            if(x3D.at<double>(3)==0)
                continue;
            // Euclidean coordinates
            x3D = x3D.rowRange(0,3)/x3D.at<double>(3);
            Eigen::Vector3d x3Dt;
            cv::cv2eigen(x3D, x3Dt);

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2)*x3Dt+tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2)*x3Dt+tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            float sigmaSquare1 = mpCurrentKeyFrame->GetSigma2(kp1.octave);
            float x1 = Rcw1.row(0)*x3Dt+tcw1(0);
            float y1 = Rcw1.row(1)*x3Dt+tcw1(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1+cx1;
            float v1 = fy1*y1*invz1+cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>Config::reprojThresh2()*sigmaSquare1)
                continue;

            //Check reprojection error in second keyframe
            float sigmaSquare2 = pKF2->GetSigma2(kp2.octave);
            float x2 = Rcw2.row(0)*x3Dt+tcw2(0);
            float y2 = Rcw2.row(1)*x3Dt+tcw2(1);
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
            float ratioOctave = mpCurrentKeyFrame->GetScaleFactor(kp1.octave)/pKF2->GetScaleFactor(kp2.octave);
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3Dt,mpCurrentKeyFrame, idx1, mpMap);

            pMP->AddObservation(pKF2,idx2);
         
            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            mpCurrentKeyFrame->mpFG->AddMapPoint(posX, posY, idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
    delete mpCurrentKeyFrame->mpFG;
    mpCurrentKeyFrame->mpFG=NULL;
}
// for now, we only check left image to get matches
// a grid is used to control distribution of map points in current keyframe
void LocalMapping::CreateNewMapPointsStereo()
{
    // Take neighbor keyframes in covisibility graph
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(20);

    ORBmatcher matcher(0.6,false);
    Sophus::SE3d proxy= mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<double,3,4> Tw2c1= proxy.matrix3x4();
    Eigen::Matrix3d Rw2c1= Tw2c1.topLeftCorner<3,3>();
    Eigen::Vector3d twinc1= Tw2c1.col(3);

    Eigen::Matrix<double,3,4> Tw2c1r= mpCurrentKeyFrame->GetPose(false).matrix3x4();
    Eigen::Vector3d Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    std::vector< Eigen::Vector3d> obs(4);
    std::vector<Sophus::SE3d> frame_poses(4);
    frame_poses[0]= Sophus::SE3d(Tw2c1.topLeftCorner<3,3>(), Tw2c1.col(3));
    frame_poses[1]= Sophus::SE3d(Tw2c1r.topLeftCorner<3,3>(), Tw2c1r.col(3));

    const float fx1 = mpCurrentKeyFrame->cam_->fx();
    const float fy1 = mpCurrentKeyFrame->cam_->fy();
    const float cx1 = mpCurrentKeyFrame->cam_->cx();
    const float cy1 = mpCurrentKeyFrame->cam_->cy();
   
    const float invfx1 = 1.0f/fx1;
    const float invfy1 = 1.0f/fy1;
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->GetScaleFactor();

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpNeighKFs[i];
        if(pKF2->mnId+1==mpCurrentKeyFrame->mnId)//because we triangulated mappoints between
            //the current keyframe and its previous frame in Tracking thread
            continue;
        // Check first that baseline is not too short
        // Small translation errors for short baseline keyframes make scale to diverge
        Eigen::Vector3d Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3d vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();
        const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
        const float ratioBaselineDepth = baseline/medianDepthKF2;

        if(ratioBaselineDepth<0.01)
            continue;

        // Compute Fundamental Matrix
        Eigen::Matrix3d F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fulfil epipolar constraint
        vector<cv::KeyPoint> vMatchedKeysUn1;
        vector<cv::KeyPoint> vMatchedKeysUn2;
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedKeysUn1,vMatchedKeysUn2,vMatchedIndices);

        proxy= pKF2->GetPose();
        Eigen::Matrix<double,3,4> Tw2c2= proxy.matrix3x4();
        Eigen::Matrix3d Rw2c2= Tw2c2.topLeftCorner<3,3>();
        Eigen::Vector3d twinc2= Tw2c2.col(3);
        Eigen::Matrix<double,3,4> Tw2c2r= pKF2->GetPose(false).matrix3x4();

        frame_poses[2]= Sophus::SE3d(Tw2c2.topLeftCorner<3,3>(), Tw2c2.col(3));
        frame_poses[3]= Sophus::SE3d(Tw2c2r.topLeftCorner<3,3>(), Tw2c2r.col(3));

        const float fx2 = pKF2->cam_->fx();
        const float fy2 = pKF2->cam_->fy();
        const float cx2 = pKF2->cam_->cx();
        const float cy2 = pKF2->cam_->cy();
        const float invfx2 = 1.0f/fx2;
        const float invfy2 = 1.0f/fy2;

        // Triangulate each match based on stereo observations
        for(size_t ikp=0, iendkp=vMatchedKeysUn1.size(); ikp<iendkp; ikp++)
        {
            const int idx1 = vMatchedIndices[ikp].first; // indices of features in current keyframe
            const int idx2 = vMatchedIndices[ikp].second;// indices of features in the other keyframe
            const cv::KeyPoint &kp1 = vMatchedKeysUn1[ikp];//current keyframe
            const cv::KeyPoint &kp2 = mpCurrentKeyFrame->mvRightKeysUn[idx1];
            int posX, posY;
            if(!mpCurrentKeyFrame->mpFG->IsPointEligible(kp1, posX, posY)) continue;
            const cv::KeyPoint &kp3 = vMatchedKeysUn2[ikp];
            const cv::KeyPoint &kp4 = pKF2->mvRightKeysUn[idx2];
#if 1
            // Check parallax between left and right rays
            Eigen::Vector3d xn1((kp1.pt.x-cx1)*invfx1,
                                (kp1.pt.y-cy1)*invfy1, 1.0 ),
                    ray1(xn1);
            Eigen::Vector3d xn3((kp3.pt.x-cx2)*invfx2,
                                (kp3.pt.y-cy2)*invfy2, 1.0 );

            Eigen::Vector3d ray3 = Rw2c1*Rw2c2.transpose()*xn3;
            const float cosParallaxRays = ray1.dot(ray3)/(ray1.norm()*ray3.norm());

            if((cosParallaxRays<0 || cosParallaxRays>Config::triangMaxCosRays())
                    && (kp1.pt.x -kp2.pt.x< Config::triangMinDisp()))
                continue;
            // Linear Triangulation Method
            Eigen::Vector3d xn2((kp2.pt.x-mpCurrentKeyFrame->right_cam_->cx())/mpCurrentKeyFrame->right_cam_->fx(),
                                (kp2.pt.y-mpCurrentKeyFrame->right_cam_->cy())/mpCurrentKeyFrame->right_cam_->fy(), 1.0 );

            Eigen::Vector3d xn4((kp4.pt.x-pKF2->right_cam_->cx())/pKF2->right_cam_->fx(),
                                (kp4.pt.y-pKF2->right_cam_->cy())/pKF2->right_cam_->fy(), 1.0 );

            Eigen::Matrix<double, 8,4> A;
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
            cv::cv2eigen(x3D, x3Dt);

 /*           obs[0]= xn1; obs[1]= xn2; obs[2]= xn3; obs[3]= xn4;
            double pdop=0;
            MapPoint::optimize(obs,frame_poses,  x3Dt, pdop, mpCurrentKeyFrame->cam_, 5);
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
            float sigmaSquare1 = mpCurrentKeyFrame->GetSigma2(kp1.octave);
            float x1 = Rw2c1.row(0)*x3Dt+twinc1(0);
            float y1 = Rw2c1.row(1)*x3Dt+twinc1(1);
            float invz1 = 1.0/z1;
            float u1 = fx1*x1*invz1 + cx1;
            float v1 = fy1*y1*invz1 + cy1;
            float errX1 = u1 - kp1.pt.x;
            float errY1 = v1 - kp1.pt.y;
            if((errX1*errX1+errY1*errY1)>Config::reprojThresh2()*sigmaSquare1)
                continue;

            //Check reprojection error in second frame
            float sigmaSquare2 = pKF2->GetSigma2(kp3.octave);
            float x2 = Rw2c2.row(0)*x3Dt+twinc2(0);
            float y2 = Rw2c2.row(1)*x3Dt+twinc2(1);
            float invz2 = 1.0/z2;
            float u2 = fx2*x2*invz2 + cx2;
            float v2 = fy2*y2*invz2 + cy2;
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
            if(kp1.pt.x -kp2.pt.x< 3 || kp3.pt.x- kp4.pt.x<3)//parallax
                continue;

            float base= -mpCurrentKeyFrame->mTl2r.translation()[0];
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
            if(abs(x3D1(2)- x3D2(2))>0.2)
                continue;
            Eigen::Vector3d x3Dt= (x3D1+ x3D2)/2;
#endif
            // Triangulation is succesful
            MapPoint* pMP = new MapPoint(x3Dt,mpCurrentKeyFrame, idx1, mpMap);

            pMP->AddObservation(pKF2, idx2);       
            pMP->AddObservation(pKF2,idx2, false);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            mpCurrentKeyFrame->mpFG->AddMapPoint(posX, posY, idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
    delete mpCurrentKeyFrame->mpFG;
    mpCurrentKeyFrame->mpFG=NULL;
}
// detect and fuse duplicate map points
void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(20);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnFrameId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnFrameId;

        // Extend to some second neighbors
        vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnFrameId || pKFi2->mnFrameId==mpCurrentKeyFrame->mnFrameId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher(0.6);
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnFrameId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnFrameId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

// stop is only requested by loop closing thread
void LocalMapping::RequestStop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopRequested = true;
    boost::mutex::scoped_lock lock2(mMutexNewKFs);
    mbAbortBA = true;
}

void LocalMapping::Stop()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isStopped()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    return mbStopped;
}
// stop is only requested by loop closing thread
bool LocalMapping::stopRequested()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    boost::mutex::scoped_lock lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();
}

bool LocalMapping::AcceptKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    boost::mutex::scoped_lock lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnFrameId==0 || pKF->isBad())
            continue;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    if(pMP->Observations()>3)
                    {
                        int scaleLevel = pKF->GetKeyPointUn(i).octave;
                        map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {                            
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            int scaleLeveli = pKFi->GetKeyPointUn(mit->second).octave;
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=3)
                                    break;
                            }
                        }
                        if(nObs>=3)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs){
                pKF->SetBadFlag();
#ifdef SLAM_DEBUG_OUTPUT
                pKF->N+=100000;
#endif
        }
    }
}



void LocalMapping::RequestReset()
{
    {
        boost::mutex::scoped_lock lock(mMutexReset);
        mbResetRequested = true;
    }
#ifdef SLAM_USE_ROS
    ros::Rate r(500);
    while(ros::ok())
#else
    while(1)
#endif
    {
        {
        boost::mutex::scoped_lock lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
#ifdef SLAM_USE_ROS
        r.sleep();
#else
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
#endif
    }
}

void LocalMapping::ResetIfRequested()
{
    boost::mutex::scoped_lock lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

} //namespace ORB_SLAM
