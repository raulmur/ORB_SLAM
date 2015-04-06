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

#include "Map.h"

namespace ORB_SLAM
{

Map::Map()
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mbMapUpdated=true;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.insert(pMP);
    mbMapUpdated=true;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mbMapUpdated=true;
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mbMapUpdated=true;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
    mbMapUpdated=true;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mvpReferenceMapPoints;
}

bool Map::isMapUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

} //namespace ORB_SLAM
