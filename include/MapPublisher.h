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

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"

namespace ORB_SLAM
{

class MapPublisher
{
public:
    MapPublisher(Map* pMap);

    Map* mpMap;

    void Refresh();
    void PublishMapPoints(const std::vector<MapPoint*> &vpMPs, const std::vector<MapPoint*> &vpRefMPs);
    void PublishKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void PublishCurrentCamera(const cv::Mat &Tcw);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

private:

    cv::Mat GetCurrentCameraPose();
    bool isCamUpdated();
    void ResetCamFlag();

    ros::NodeHandle nh;
    ros::Publisher publisher;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;

    float fCameraSize;
    float fPointSize;

    cv::Mat mCameraPose;
    bool mbCameraUpdated;

    boost::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
