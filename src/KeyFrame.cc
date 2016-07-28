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

#include "KeyFrame.h"
#include "Converter.h"
#include "g2o_types/eigen_utils.h"
#include "global.h" //for debugging output
#include <vikit/math_utils.h>
//#include <ros/ros.h>

namespace ORB_SLAM
{

long unsigned int KeyFrame::nNextKeyId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):Frame(F),  mnFrameId(nNextKeyId++),
    mnTrackReferenceForFrame(0),mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnRelocQuery(0),mpFG(NULL),    mpKeyFrameDB(pKFDB),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(0), mbToBeErased(false),
    mpMap(pMap)
{
    // mGrids is taken care in copying base Frame
    /*mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }*/
//    SetPose(F.mTcw);
}
void KeyFrame::Release()
{    
    // the following commented member variables may be referred to by another thread when this keyframe is setbad
//    mTcw;
//    mOw;  

//    imu_observ.clear();//TODO: somehow a keyframe in the temporal window may get released
    mvKeys.clear();
//    mvKeysUn.clear();
    mvRightKeys.clear();
    mvRightKeysUn.clear();
    mBowVec.clear();
    mFeatVec.clear();
//    mDescriptors.release();
    mRightDescriptors.release();

//    mvpMapPoints.clear();
    mvbOutlier.clear();
//    for(int i=0;i<FRAME_GRID_COLS;i++)
//        for(int j=0; j<FRAME_GRID_ROWS; j++)
//            mGrid[i][j].clear();

    viso2LeftId2StereoId.clear();
    viso2RightId2StereoId.clear();

    mConnectedKeyFrameWeights.clear();
    mvpOrderedConnectedKeyFrames.clear();
    mvOrderedWeights.clear();
    mspChildrens.clear();
    mspLoopEdges.clear();
    if(mpFG)
        delete mpFG;
    mpFG=NULL;
}


void KeyFrame::SetPose(const Eigen::Matrix3d &Rcw,const Eigen::Vector3d &tcw)
{
    boost::mutex::scoped_lock lock(mMutexPose);
    mTcw=Sophus::SE3d(Rcw, tcw);
    mOw=-Rcw.transpose()*tcw;
}

void KeyFrame::SetPose(const Sophus::SE3d &Tcw_)
{
    boost::mutex::scoped_lock lock(mMutexPose);
    mTcw=Tcw_;
    mOw = mTcw.inverse().translation();
}

Sophus::SE3d KeyFrame::GetPose(bool left)
{
    if(left){
    boost::mutex::scoped_lock lock(mMutexPose);
    return mTcw;
    }
    else
        return (mTl2r*mTcw);

}

Sophus::SE3d KeyFrame::GetPoseInverse()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return mTcw.inverse();
}

Eigen::Vector3d KeyFrame::GetCameraCenter()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return mOw;
}

Eigen::Matrix3d KeyFrame::GetRotation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return mTcw.rotationMatrix();
}

Eigen::Vector3d KeyFrame::GetTranslation()
{
    boost::mutex::scoped_lock lock(mMutexPose);
    return mTcw.translation();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    boost::mutex::scoped_lock lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    assert((pMP && mvpMapPoints[idx]==NULL));
  
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    assert(mvpMapPoints[idx]);
    mvpMapPoints[idx]=NULL;
 
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
   	assert(idx>=0 && mvpMapPoints[idx]);
    mvpMapPoints[idx]=NULL;

}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}
// how many map points are found in this keyframe
int KeyFrame::TrackedMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);

    int nPoints=0;

    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; ++i)
    {
        if(mvpMapPoints[i])
            ++nPoints;
    }
    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints;
}


MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mvpMapPoints[idx];
}


int KeyFrame::GetKeyPointScaleLevel(const size_t &idx) const
{
    assert(idx< mvKeysUn.size());
    return mvKeysUn[idx].octave;

}

cv::Mat KeyFrame::GetDescriptors(bool left)
{
    if(left)
    return mDescriptors.clone();
    else
        return mRightDescriptors.clone();
}

vector<cv::KeyPoint> KeyFrame::GetKeyPointsUn() const
{
    return mvKeysUn;
}

Eigen::Matrix3d KeyFrame::GetCalibrationMatrix() const
{
    return cam_.K();
}

DBoW2::FeatureVector KeyFrame::GetFeatureVector()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mFeatVec;
}

DBoW2::BowVector KeyFrame::GetBowVector()
{
    boost::mutex::scoped_lock lock(mMutexFeatures);
    return mBowVec;
}


void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        boost::mutex::scoped_lock lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if((mit->first)->mnFrameId==mnFrameId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        boost::mutex::scoped_lock lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
//huai: FAQ: do we need to update mvpOrderedConnectedKeyFrames of connected keyframes?
        if(mbFirstConnection && mnFrameId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }
    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}


void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    mbNotErase |= LoopCandidateKF;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    boost::mutex::scoped_lock lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase(uchar enableWhichProtection)
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    mbNotErase |=enableWhichProtection;
}
uchar KeyFrame::GetNotErase()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mbNotErase;
}
void KeyFrame::SetErase(uchar disableWhichProtection)
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if( disableWhichProtection == DoubleWindowKF)
            mbNotErase &= (~DoubleWindowKF);
        else{
            if(mspLoopEdges.empty())
            {
                mbNotErase &= (~LoopCandidateKF);
            }
        }
    }

    if(mbNotErase==0 && mbToBeErased)
    {// set bad flag must have been called once for *this
        SetBadFlag();
#ifdef SLAM_DEBUG_OUTPUT
        N+=1000000;
#endif
    }
}

void KeyFrame::SetBadFlag()
{
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mnFrameId==0 || mbBad)
            return;
        if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    {
        boost::mutex::scoped_lock lock1(mMutexFeatures);
        for(vector<MapPoint*>::const_iterator it=mvpMapPoints.begin(); it!=mvpMapPoints.end(); ++it){
            if((*it) && (!(*it)->isBad()))
                (*it)->EraseObservation(this);
        }
    }

    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
            mit->first->EraseConnection(this);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mbBad = true;
    }

    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
    Release();//Huai
}

bool KeyFrame::isBad()
{
    boost::mutex::scoped_lock lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        boost::mutex::scoped_lock lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvKeysUn.size());

    int nMinCellX = floor((x-mnMinX-r)*mfGridElementWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*mfGridElementWidthInv);
    nMaxCellX = min(FRAME_GRID_COLS-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*mfGridElementHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*mfGridElementHeightInv);
    nMaxCellY = min(FRAME_GRID_ROWS-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t>& vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(abs(kpUn.pt.x-x)<=r && abs(kpUn.pt.y-y)<=r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(int q)
{
    vector<MapPoint*> vpMapPoints;
    Sophus::SE3d Tcw_;
    {
    boost::mutex::scoped_lock lock(mMutexFeatures);
    boost::mutex::scoped_lock lock2(mMutexPose);
    vpMapPoints = mvpMapPoints;
    Tcw_ = mTcw;
    }

    vector<float> vDepths;
    vDepths.reserve(mvpMapPoints.size());
    Eigen::Matrix<double,1,3> Rcw2 = Tcw_.rotationMatrix().row(2);
    float zcw = Tcw_.translation()[2];
    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            Eigen::Vector3d x3Dw = pMP->GetWorldPos();
            float z = Rcw2*x3Dw+zcw;
            vDepths.push_back(z);
        }
    }
    if(vDepths.size()==0)
        return 2.f;
    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];    
}
void KeyFrame::setExistingFeatures(FeatureGrid &fg) // this frame is not processed by local mapping thread yet
{
 // put map points in grid
    int jim=0;
    for(vector<MapPoint*>::iterator vit=mvpMapPoints.begin(), vend=mvpMapPoints.end();
        vit!=vend; ++vit, ++jim)
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
                cv::KeyPoint kpUn=mvKeysUn[jim];
                int posX = round((kpUn.pt.x-mnMinX)*fg.mfGridElementWidthInv);
                int posY = round((kpUn.pt.y-mnMinY)*fg.mfGridElementHeightInv);
                //cout<< "posX,Y "<< kpUn.pt.x<<" "<<kpUn.pt.y<<endl;
                assert(!(posX<0 || posX>=fg.mnGridCols || posY<0 || posY>=fg.mnGridRows));
                fg.AddMapPoint(posX,posY,jim);
            }
        }
    }
}
Eigen::Matrix3d ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    Sophus::SE3d T12= pKF1->GetPose()*(pKF2->GetPose().inverse());
    Eigen::Matrix3d R12 = T12.rotationMatrix();
    Eigen::Vector3d t12 = T12.translation();
    Eigen::Matrix3d t12x = skew3d(t12);
    return (pKF1->cam_.K_inv().transpose())*t12x*R12*(pKF2->cam_.K_inv());
}


} //namespace ORB_SLAM
