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

#include "MapPoint.h"
#include "ORBmatcher.h"
//#include "vikit/math_utils.h"
#include "global.h" //EPS
#include "config.h"
//#include "ros/ros.h"

namespace ORB_SLAM
{

long unsigned int MapPoint::nNextId=0;


MapPoint::MapPoint(const Eigen::Vector3d &Pos, KeyFrame *pRefKF,const int nIDInKF, Map* pMap):
    mnId(nNextId++),mnFirstKFid(pRefKF->mnFrameId), mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0),
    mnLoopPointForKF(0), mnCorrectedByKF(0),mnCorrectedReference(0),mbFixedLinearizationPoint(false),
    mpRefKF(pRefKF), mnVisible(1), mnFound(1),v_pt_(NULL),mpMap(pMap),
    mWorldPos(Pos), mNormalVector(0,0,0),mDescriptor(pRefKF->GetDescriptor(nIDInKF)),
    mbBad(false), mfMinDistance(0), mfMaxDistance(0)
{
    mObservations[pRefKF]= nIDInKF;
#ifndef MONO
    mRightObservations[pRefKF]= nIDInKF;
#endif
}


void MapPoint::SetWorldPos(const Eigen::Vector3d &Pos)
{
    boost::mutex::scoped_lock lock(mMutexPos);
    mWorldPos=Pos;
}

Eigen::Vector3d MapPoint::GetWorldPos()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mWorldPos;
}

Eigen::Vector3d MapPoint::GetNormal()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mNormalVector;
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     boost::mutex::scoped_lock lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pF, size_t idx, bool left)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    if(left)
        mObservations[pF]=idx;
    else
        mRightObservations[pF]=idx;
}
// local mapping and loop closing do not call this function directly
void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            mObservations.erase(pKF);
            if(mpRefKF==pKF)            
    			mpRefKF=mObservations.begin()->first;
#ifdef MONO
            // If only 1 or 2 observations or less, discard point
            if(mObservations.size()<=2)
                bBad=true;
#else
            if(mObservations.size()<=1) //set 1 to avoid that points used in tracking thread are marked bad,
                // because if a point is used in tracking, it must be observed by at least two KEYframes in the double window
                // and these frames are not allowed to be culled and cannot be this keyframe
                bBad=true;
#endif
        }
        mRightObservations.erase(pKF);
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations(bool left)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    if(left)
        return mObservations;
    else
        return mRightObservations;
}

int MapPoint::Observations()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mObservations.size();
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
        mbBad=true;
    
        obs = mObservations;
        mObservations.clear();
        mRightObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }
    mpMap->EraseMapPoint(this);
	Release();
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;
    assert(pMP->mpRefKF->isBad()==false);

    map<KeyFrame*,size_t> obs;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
		obs=mObservations;
        mbBad=true;
       //note this point may be observed in current frame and used in localoptimize,
        // but we still delete its observations, so some isolated point may exist in localoptimize
        mObservations.clear();
        mRightObservations.clear();
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in frame
        KeyFrame* pKF = mit->first;
   
        int nMPId=pMP->IdInKeyFrame(pKF);
        if(nMPId==-1)
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
#ifndef MONO
            pMP->AddObservation(pKF,mit->second, false);
#endif
        }
        else
        {
            assert(nMPId!= (int) mit->second);
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->ComputeDistinctiveDescriptors();
    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    boost::mutex::scoped_lock lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnVisible++;
}

void MapPoint::IncreaseFound()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mnFound++;
}

float MapPoint::GetFoundRatio()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}
// choose the distinctive descriptor from those of this MapPoint's observations
void MapPoint::ComputeDistinctiveDescriptors()
{
    assert(mpRefKF->isBad()==false);
    assert(!mbBad);

    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations, right_observations;
	{
        boost::mutex::scoped_lock lock1(mMutexFeatures);
    
        observations=mObservations;
		right_observations=mRightObservations;
    }
   
    vDescriptors.reserve(observations.size()+right_observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
//        assert(pF->mvpMapPoints[mit->second]);
        if(!pKF->isBad())
            vDescriptors.push_back(pKF->GetDescriptor(mit->second));
    }
    for(map<KeyFrame*,size_t>::iterator mit=right_observations.begin(), mend=right_observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->GetDescriptor(mit->second, false));
    }
  	assert(!vDescriptors.empty());
    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        boost::mutex::scoped_lock lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();       
    }
}
cv::Mat MapPoint::GetDescriptor()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mDescriptor.clone();
}
void MapPoint::SetDescriptor(const cv::Mat & descrip)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    mDescriptor=descrip;
}
int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF, bool left)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    if(left){
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
    }
    else{
        if(mRightObservations.count(pKF))
            return mRightObservations[pKF];
        else
            return -1;        
    }
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

int MapPoint::IdInKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    map<KeyFrame*, size_t>::const_iterator it= mObservations.find(pKF);
    if(it==mObservations.end())
        return -1;
    else
        return it->second;
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
   
    KeyFrame* pRefKF;
    Eigen::Vector3d Pos;
    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        boost::mutex::scoped_lock lock2(mMutexPos);
   		if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        assert(pRefKF->isBad()==false);
        Pos = mWorldPos;
    }

    Eigen::Vector3d normal(0,0,0);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(),
        mend=observations.end(); mit!=mend; ++mit)
    {
        KeyFrame* pKF = mit->first;
        if(pKF->isBad())
            continue;
        Eigen::Vector3d Owi = pKF->GetCameraCenter();
        Eigen::Vector3d normali = Pos - Owi;
        normal = normal + normali/normali.norm();
        ++n;
    }
	assert(n);
    Eigen::Vector3d PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();
    const int nLevels = pRefKF->GetScaleLevels();
    if(nLevels==1)
    {
        boost::mutex::scoped_lock lock3(mMutexPos);
        mfMinDistance = dist/4.0f;
        mfMaxDistance = 4.0f*dist;
        mNormalVector = normal/n;
        return;
    }

    const int level = pRefKF->GetKeyPointScaleLevel(observations[pRefKF]);
    const float levelScaleFactor =  pRefKF->GetScaleFactor(level);

    {
        boost::mutex::scoped_lock lock3(mMutexPos);     
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->GetScaleFactor(nLevels-1);
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    boost::mutex::scoped_lock lock(mMutexPos);
    return mfMaxDistance;
}
void MapPoint::SetFirstEstimate()
{
    if(!mbFixedLinearizationPoint){
        mWorldPos_first_estimate=mWorldPos;
        mbFixedLinearizationPoint=true;
    }
}
void MapPoint::Release()
{
    mObservations.clear();
    mRightObservations.clear();
//    mNormalVector.release();
//    mDescriptor.release();
}

// 4 obs, each is observations in image plane z=1, (\bar{x}, \bar{y}, 1), each of 4 frame_poses is Tw2c(i)
// old_point stores initial position and optimized position, pdop is position dilution of precision

//void MapPoint::optimize(const std::vector< Eigen::Vector3d> & obs,
//                        const std::vector<Sophus::SE3d> & frame_poses,
//                        Eigen::Vector3d& old_point, double & pdop,
//                        vk::PinholeCamera* cam, size_t n_iter)
//{
//  Eigen::Vector3d pos = old_point;
//  double chi2 = 0.0;
//  Eigen::Matrix3d A;
//  Eigen::Vector3d b;
//    const double pixel_variance=1.0;
//  for(size_t i=0; i<n_iter; ++i)
//  {
//    A.setZero();
//    b.setZero();
//    double new_chi2 = 0.0;

//    // compute residuals
//    std::vector<Sophus::SE3d>::const_iterator it_poses= frame_poses.begin();
//    for(std::vector< Eigen::Vector3d>::const_iterator it=obs.begin();
//        it!=obs.end(); ++it, ++it_poses)
//    {
//      Matrix23d J;
//      const Eigen::Vector3d p_in_f( (*it_poses) * pos);
//      MapPoint::jacobian_xyz2uv(p_in_f, it_poses->rotationMatrix(), J);
//      const Eigen::Vector2d e(vk::project2d(*it) - vk::project2d(p_in_f));
//      new_chi2 += e.squaredNorm();
//      A.noalias() += J.transpose() * J;
//      b.noalias() -= J.transpose() * e;
//    }
//    pdop= pixel_variance*sqrt(A.inverse().trace())/cam->errorMultiplier2();
//    if(pdop> Config::PDOPThresh()) break;
//    // solve linear system
//    const Eigen::Vector3d dp(A.ldlt().solve(b));

//    // check if error increased
//    if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0]))
//    {
//#ifdef POINT_OPTIMIZER_DEBUG
//      cout << "it " << i
//           << "\t FAILURE \t new_chi2 = " << new_chi2 << endl;
//#endif
//      pos = old_point; // roll-back
//      break;
//    }

//    // update the model
//    Eigen::Vector3d new_point = pos + dp;
//    old_point = pos;
//    pos = new_point;
//    chi2 = new_chi2;
//#ifdef POINT_OPTIMIZER_DEBUG
//    cout << "it " << i
//         << "\t Success \t new_chi2 = " << new_chi2
//         << "\t norm(b) = " << vk::norm_max(b)
//         << endl;
//#endif

//    // stop when converged
//    if(vk::norm_max(dp) <= ORB_SLAM::EPS)
//      break;
//  }
//  old_point =pos;
//#ifdef POINT_OPTIMIZER_DEBUG
//  cout << endl;
//#endif
//}

} //namespace ORB_SLAM
