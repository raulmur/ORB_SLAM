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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include<set>

#include<boost/thread.hpp>



namespace ORB_SLAM
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetCurrentCameraPose(cv::Mat Tcw);
    void SetReferenceKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    cv::Mat GetCameraPose();
    std::vector<KeyFrame*> GetReferenceKeyFrames();
    std::vector<MapPoint*> GetReferenceMapPoints();

    int MapPointsInMap();
    int KeyFramesInMap();

    void SetFlagAfterBA();
    bool isMapUpdated();
    void ResetUpdated();

    unsigned int GetMaxKFid();

    void clear();

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    unsigned int mnMaxKFid;

    boost::mutex mMutexMap;
    bool mbMapUpdated;
};

} //namespace ORB_SLAM

#endif // MAP_H
