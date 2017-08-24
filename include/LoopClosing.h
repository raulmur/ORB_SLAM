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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "sophus/sim3.hpp"
#include <boost/thread.hpp>

#include "KeyFrameDatabase.h"

//#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
    typedef map<KeyFrame*,Sophus::SE3d,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, Sophus::SE3d> > > KeyFrameAndSE3Pose;
public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    Sophus::Sim3d GetSnew2old();
    Sophus::SE3d GetTnew2old();


public:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(KeyFrameAndPose &CorrectedPosesMap);
    void SearchAndFuse(KeyFrameAndSE3Pose &CorrectedPosesMap);
    void CorrectLoop();
    void CorrectLoopSE3();
    void ResetIfRequested();
    bool mbResetRequested;
    boost::mutex mMutexReset;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    boost::mutex mMutexLoopQueue;

    std::vector<float> mvfLevelSigmaSquare;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints; 
    g2o::Sim3 mg2oScw; //transform from new world frame to current keyframe
    double mScale_cw;

    long unsigned int mLastLoopKFid;

};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
