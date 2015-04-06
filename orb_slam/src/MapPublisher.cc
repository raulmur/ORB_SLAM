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

#include "MapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM
{


MapPublisher::MapPublisher(Map* pMap):mpMap(pMap), mbCameraUpdated(false)
{
    const char* MAP_FRAME_ID = "/ORB_SLAM/World";
    const char* POINTS_NAMESPACE = "MapPoints";
    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
    const char* GRAPH_NAMESPACE = "Graph";
    const char* CAMERA_NAMESPACE = "Camera";

    //Configure MapPoints
    fPointSize=0.01;
    mPoints.header.frame_id = MAP_FRAME_ID;
    mPoints.ns = POINTS_NAMESPACE;
    mPoints.id=0;
    mPoints.type = visualization_msgs::Marker::POINTS;
    mPoints.scale.x=fPointSize;
    mPoints.scale.y=fPointSize;
    mPoints.pose.orientation.w=1.0;
    mPoints.action=visualization_msgs::Marker::ADD;
    mPoints.color.a = 1.0;

    //Configure KeyFrames
    fCameraSize=0.04;
    mKeyFrames.header.frame_id = MAP_FRAME_ID;
    mKeyFrames.ns = KEYFRAMES_NAMESPACE;
    mKeyFrames.id=1;
    mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
    mKeyFrames.scale.x=0.005;
    mKeyFrames.pose.orientation.w=1.0;
    mKeyFrames.action=visualization_msgs::Marker::ADD;

    mKeyFrames.color.b=1.0f;
    mKeyFrames.color.a = 1.0;

    //Configure Covisibility Graph
    mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
    mCovisibilityGraph.ns = GRAPH_NAMESPACE;
    mCovisibilityGraph.id=2;
    mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
    mCovisibilityGraph.scale.x=0.002;
    mCovisibilityGraph.pose.orientation.w=1.0;
    mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
    mCovisibilityGraph.color.b=0.7f;
    mCovisibilityGraph.color.g=0.7f;
    mCovisibilityGraph.color.a = 0.3;

    //Configure KeyFrames Spanning Tree
    mMST.header.frame_id = MAP_FRAME_ID;
    mMST.ns = GRAPH_NAMESPACE;
    mMST.id=3;
    mMST.type = visualization_msgs::Marker::LINE_LIST;
    mMST.scale.x=0.005;
    mMST.pose.orientation.w=1.0;
    mMST.action=visualization_msgs::Marker::ADD;
    mMST.color.b=0.0f;
    mMST.color.g=1.0f;
    mMST.color.a = 1.0;

    //Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.01;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

    //Configure Reference MapPoints
    mReferencePoints.header.frame_id = MAP_FRAME_ID;
    mReferencePoints.ns = POINTS_NAMESPACE;
    mReferencePoints.id=6;
    mReferencePoints.type = visualization_msgs::Marker::POINTS;
    mReferencePoints.scale.x=fPointSize;
    mReferencePoints.scale.y=fPointSize;
    mReferencePoints.pose.orientation.w=1.0;
    mReferencePoints.action=visualization_msgs::Marker::ADD;
    mReferencePoints.color.r =1.0f;
    mReferencePoints.color.a = 1.0;

    //Configure Publisher
    publisher = nh.advertise<visualization_msgs::Marker>("ORB_SLAM/Map", 10);

    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mKeyFrames);
    publisher.publish(mCurrentCamera);
}

void MapPublisher::Refresh()
{
    if(isCamUpdated())
    {
       cv::Mat Tcw = GetCurrentCameraPose();
       PublishCurrentCamera(Tcw);
       ResetCamFlag();
    }
    if(mpMap->isMapUpdated())
    {
        vector<KeyFrame*> vKeyFrames = mpMap->GetAllKeyFrames();
        vector<MapPoint*> vMapPoints = mpMap->GetAllMapPoints();
        vector<MapPoint*> vRefMapPoints = mpMap->GetReferenceMapPoints();

        PublishMapPoints(vMapPoints, vRefMapPoints);   
        PublishKeyFrames(vKeyFrames);

        mpMap->ResetUpdated();
    }    
}

void MapPublisher::PublishMapPoints(const vector<MapPoint*> &vpMPs, const vector<MapPoint*> &vpRefMPs)
{
    mPoints.points.clear();
    mReferencePoints.points.clear();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mPoints.points.push_back(p);
    }

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        geometry_msgs::Point p;
        cv::Mat pos = (*sit)->GetWorldPos();
        p.x=pos.at<float>(0);
        p.y=pos.at<float>(1);
        p.z=pos.at<float>(2);

        mReferencePoints.points.push_back(p);
    }

    mPoints.header.stamp = ros::Time::now();
    mReferencePoints.header.stamp = ros::Time::now();
    publisher.publish(mPoints);
    publisher.publish(mReferencePoints);
}

void MapPublisher::PublishKeyFrames(const vector<KeyFrame*> &vpKFs)
{
    mKeyFrames.points.clear();
    mCovisibilityGraph.points.clear();
    mMST.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    for(size_t i=0, iend=vpKFs.size() ;i<iend; i++)
    {
        cv::Mat Tcw = vpKFs[i]->GetPose();
        cv::Mat Twc = Tcw.inv();
        cv::Mat ow = vpKFs[i]->GetCameraCenter();
        cv::Mat p1w = Twc*p1;
        cv::Mat p2w = Twc*p2;
        cv::Mat p3w = Twc*p3;
        cv::Mat p4w = Twc*p4;

        geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
        msgs_o.x=ow.at<float>(0);
        msgs_o.y=ow.at<float>(1);
        msgs_o.z=ow.at<float>(2);
        msgs_p1.x=p1w.at<float>(0);
        msgs_p1.y=p1w.at<float>(1);
        msgs_p1.z=p1w.at<float>(2);
        msgs_p2.x=p2w.at<float>(0);
        msgs_p2.y=p2w.at<float>(1);
        msgs_p2.z=p2w.at<float>(2);
        msgs_p3.x=p3w.at<float>(0);
        msgs_p3.y=p3w.at<float>(1);
        msgs_p3.z=p3w.at<float>(2);
        msgs_p4.x=p4w.at<float>(0);
        msgs_p4.y=p4w.at<float>(1);
        msgs_p4.z=p4w.at<float>(2);

        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_o);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p2);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p3);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p4);
        mKeyFrames.points.push_back(msgs_p1);

        // Covisibility Graph
        vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                geometry_msgs::Point msgs_o2;
                msgs_o2.x=Ow2.at<float>(0);
                msgs_o2.y=Ow2.at<float>(1);
                msgs_o2.z=Ow2.at<float>(2);
                mCovisibilityGraph.points.push_back(msgs_o);
                mCovisibilityGraph.points.push_back(msgs_o2);
            }
        }

        // MST
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            geometry_msgs::Point msgs_op;
            msgs_op.x=Owp.at<float>(0);
            msgs_op.y=Owp.at<float>(1);
            msgs_op.z=Owp.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_op);
        }
        set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            geometry_msgs::Point msgs_ol;
            msgs_ol.x=Owl.at<float>(0);
            msgs_ol.y=Owl.at<float>(1);
            msgs_ol.z=Owl.at<float>(2);
            mMST.points.push_back(msgs_o);
            mMST.points.push_back(msgs_ol);
        }
    }

    mKeyFrames.header.stamp = ros::Time::now();
    mCovisibilityGraph.header.stamp = ros::Time::now();
    mMST.header.stamp = ros::Time::now();

    publisher.publish(mKeyFrames);
    publisher.publish(mCovisibilityGraph);
    publisher.publish(mMST);
}

void MapPublisher::PublishCurrentCamera(const cv::Mat &Tcw)
{
    mCurrentCamera.points.clear();

    float d = fCameraSize;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    cv::Mat Twc = Tcw.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = ros::Time::now();

    publisher.publish(mCurrentCamera);
}

void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbCameraUpdated = true;
}

cv::Mat MapPublisher::GetCurrentCameraPose()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    return mCameraPose.clone();
}

bool MapPublisher::isCamUpdated()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mbCameraUpdated = false;
}

} //namespace ORB_SLAM
