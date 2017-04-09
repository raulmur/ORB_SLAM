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

#include "Frame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "ORBmatcher.h" //stereo matching
#include "vio/eigen_utils.h" //skew3d
#include "global.h"

#include <vikit/vision.h>
#include <vikit/math_utils.h> //getMedian

//#include <ros/ros.h>


namespace ORB_SLAM
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::snMinX, Frame::snMinY, Frame::snMaxX, Frame::snMaxY;
float Frame::mfGridElementWidthInv;
float Frame::mfGridElementHeightInv;

Frame& Frame::operator =(const Frame& rv)
{
    if(this!=&rv)
    {
        this->Frame::~Frame();
        new(this) Frame(rv);
    }
    return *this;
}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor),
     mTimeStamp(frame.mTimeStamp),mTcw(frame.mTcw),mOw(frame.mOw),
     prev_frame(frame.prev_frame), next_frame(frame.next_frame), speed_bias(frame.speed_bias), imu_observ(frame.imu_observ),
     mbFixedLinearizationPoint(frame.mbFixedLinearizationPoint), speed_bias_first_estimate(frame.speed_bias_first_estimate),
     mTcw_first_estimate(frame.mTcw_first_estimate),
     cam_(frame.cam_), right_cam_(frame.right_cam_),mTl2r(frame.mTl2r),    
     // we do not clone pyramid because frame will be deleted and these pyramid will be kept by the new frame
     N(frame.N), mvKeys(frame.mvKeys), mvKeysUn(frame.mvKeysUn),
     mvRightKeys(frame.mvRightKeys), mvRightKeysUn(frame.mvRightKeysUn),
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), mDescriptors(frame.mDescriptors.clone()),
     mRightDescriptors(frame.mRightDescriptors.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier),
     mnId(frame.mnId), mnScaleLevels(frame.mnScaleLevels), mfScaleFactor(frame.mfScaleFactor),
     mfLogScaleFactor(frame.mfLogScaleFactor), mvScaleFactors(frame.mvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mnMinX(frame.mnMinX),mnMaxX(frame.mnMaxX),mnMinY(frame.mnMinY), mnMaxY(frame.mnMaxY),
     viso2LeftId2StereoId(frame.viso2LeftId2StereoId),viso2RightId2StereoId(frame.viso2RightId2StereoId),
     mbBad(frame.mbBad), v_kf_(frame.v_kf_), v_sb_(frame.v_sb_)
{ 
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];
}


Frame::Frame(cv::Mat &im_, const double &timeStamp, ORBextractor* extractor,
             ORBVocabulary* voc, vk::PinholeCamera* cam,
             const Eigen::Vector3d ginc, const Eigen::Matrix<double, 9,1> sb)
    :mpORBvocabulary(voc),mpORBextractor(extractor), mTimeStamp(timeStamp),
      prev_frame(NULL), next_frame(NULL), speed_bias(sb), mbFixedLinearizationPoint(false), cam_(*cam),
      right_cam_(*cam), mnId(nNextId++), mbBad(false),v_kf_(NULL), v_sb_(NULL)
{

    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();

    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    cv::Mat fK=Converter::toCvMat(cam_.K());
    cv::Mat fDistCoef=(cv::Mat_<float>(4,1)<< cam_.d0(),cam_.d1(), cam_.d2(), cam_.d3());

    //compute orientation either with gravity or illumination
    if(ginc.norm()<1e-6) //assigned proper value
    {
        (*mpORBextractor)(im_, cv::Mat(), mvKeys, mDescriptors);
        UndistortKeyPoints( mvKeys, mvKeysUn, fK, fDistCoef);
    }else{
        (*mpORBextractor)( im_, cv::Mat(), mvKeys, mDescriptors,
          mvKeysUn, fK, fDistCoef, ginc);
    }
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    // This is done for the first created Frame
    if(mbInitialComputations)
    {
        ComputeImageBounds();
        mnMinX = snMinX;
        mnMaxX = snMaxX;
        mnMinY = snMinY;
        mnMaxY = snMaxY;

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        mbInitialComputations=false;
    }
    else
    {
        mnMinX = snMinX;
        mnMaxX = snMaxX;
        mnMinY = snMinY;
        mnMaxY = snMaxY;
    }

    // Assign Features to Grid Cells
    int nReserve = 0.5*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);


    for(size_t i=0;i<mvKeysUn.size();i++)
    {
        cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
    mvbOutlier = vector<bool>(N,false);    
}

Frame::Frame(cv::Mat &im_ , const double & timeStamp, const int num_features_left, cv::Mat &right_img, const int num_features_right,
                      const std::vector<p_match> & vStereoMatches, ORBextractor* extractor, ORBVocabulary* voc,
            vk::PinholeCamera * cam, vk::PinholeCamera * right_cam, const Sophus::SE3d& Tl2r,
             const Eigen::Vector3d & ginc, const Eigen::Matrix<double, 9,1> sb)
    :mpORBvocabulary(voc),mpORBextractor(extractor), mTimeStamp(timeStamp),
      prev_frame(NULL), next_frame(NULL), speed_bias(sb), mbFixedLinearizationPoint(false),
      cam_(*cam), right_cam_(*right_cam),
        mTl2r(Tl2r), mnId(nNextId++), mbBad(false),v_kf_(NULL), v_sb_(NULL)
{
    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();

    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    N=vStereoMatches.size();
    if(N==0)
        return;
    // Exctract ORB for left image and right image
    mvKeys.resize(N);
    mvRightKeys.resize(N);
    viso2LeftId2StereoId.resize(num_features_left, -1);
    viso2RightId2StereoId.resize(num_features_right, -1);

    for (int jack=0; jack<N; ++jack)
    {
        const p_match & pMatch=vStereoMatches[jack];
        viso2LeftId2StereoId[pMatch.i1c]= jack;
        viso2RightId2StereoId[pMatch.i2c]= jack;

        mvKeys[jack]=cv::KeyPoint(pMatch.u1c, pMatch.v1c, 11);//11 according to stereoscan article
        mvRightKeys[jack]=cv::KeyPoint(pMatch.u2c, pMatch.v2c, 11);
    }
    cv::Mat fK=Converter::toCvMat(cam_.K());
    cv::Mat fDistCoef=(cv::Mat_<float>(4,1)<< cam_.d0(),cam_.d1(), cam_.d2(), cam_.d3());

    cv::Mat fRightK=Converter::toCvMat(right_cam_.K());
    cv::Mat fRightDistCoef=(cv::Mat_<float>(4,1)<< right_cam_.d0(),right_cam_.d1(), right_cam_.d2(), right_cam_.d3());

    UndistortKeyPoints( mvKeys, mvKeysUn, fK, fDistCoef);
    UndistortKeyPoints( mvRightKeys, mvRightKeysUn, fRightK, fRightDistCoef);
    //compute orientation either with gravity or illumination
    if(ginc.norm()<1e-6) //assigned proper value
    {
        (*mpORBextractor)(im_, mvKeys, mDescriptors);
        (*mpORBextractor)(right_img, mvRightKeys, mRightDescriptors);
    }else
    {
        computeKeyPointGAO(mvKeys, mvKeysUn, fK, fDistCoef, ginc);
        (*mpORBextractor)(im_, mvKeys, mDescriptors, true);

        Eigen::Vector3d ginr=mTl2r*ginc;

        computeKeyPointGAO(mvRightKeys, mvRightKeysUn, fRightK, fRightDistCoef, ginr);
        (*mpORBextractor)(right_img, mvRightKeys, mRightDescriptors, true);
    }       

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    // This is done for the first created Frame
    if(mbInitialComputations)
    {
        ComputeImageBounds();
        mnMinX = snMinX;
        mnMaxX = snMaxX;
        mnMinY = snMinY;
        mnMaxY = snMaxY;
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        mbInitialComputations=false;
    }
    else
    {
        mnMinX = snMinX;
        mnMaxX = snMaxX;
        mnMinY = snMinY;
        mnMaxY = snMaxY;
    }

    // Assign Features to Grid Cells
    int nReserve = 0.5*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);//N.B. mGrid[i][j] can grow over nReserve

    for(size_t i=0;i<mvKeysUn.size();i++)
    {
        cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
    mvbOutlier = vector<bool>(N,false);

}
//Deallocate memory occupied by this Frame
void Frame::Release()
{
    imu_observ.clear();  
    mvKeys.clear();
    mvKeysUn.clear();
    mvRightKeys.clear();
    mvRightKeysUn.clear();
    mBowVec.clear();
    mFeatVec.clear();

    mDescriptors.release();
    mRightDescriptors.release();
    mvpMapPoints.clear();
   
    mvbOutlier.clear();
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j].clear();
    viso2LeftId2StereoId.clear();
    viso2RightId2StereoId.clear();

}

// all the vectors that are not used in keyframe operations will be released before creating a keyframe from this frame
void Frame::PartialRelease()
{
//    imu_observ.clear();
    mvKeys.clear();
    mvRightKeys.clear();
    mvbOutlier.clear();

// mGrid is used for detecting matches between keyframes
//    for(int i=0;i<FRAME_GRID_COLS;i++)
//        for(int j=0; j<FRAME_GRID_ROWS; j++)
//            mGrid[i][j].clear();

    viso2LeftId2StereoId.clear();
    viso2RightId2StereoId.clear();
}

Frame::~Frame()
{
    Release();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rotationMatrix();
    mtcw = mTcw.translation();
    mOw = -mRcw.transpose()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    Eigen::Vector3d P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3d Pc = mRcw*P+mtcw;
    const float PcX = Pc(0);
    const float PcY= Pc(1);
    const float PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0/PcZ;
    const float u=cam_.fx()*PcX*invz+cam_.cx();
    const float v=cam_.fy()*PcY*invz+cam_.cy();

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3d PO = P-mOw;
    const float dist = PO.norm();

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    Eigen::Vector3d Pn = pMP->GetNormal();

    float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale level acording to the distance
    float ratio = dist/minDistance;

    vector<float>::iterator it = lower_bound(mpORBextractor->mvScaleFactor.begin(), mpORBextractor->mvScaleFactor.end(), ratio);
    int nPredictedLevel = it-mpORBextractor->mvScaleFactor.begin();

    if(nPredictedLevel>=GetScaleLevels())
        nPredictedLevel=GetScaleLevels()-1;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}
//a map point has to be seen in both left and right image
bool Frame::isInFrustumStereo(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    Eigen::Vector3d P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3d Pc = mRcw*P+mtcw;
    const float PcX = Pc(0);
    const float PcY= Pc(1);
    const float PcZ = Pc(2);

    Eigen::Vector3d PrXYZ= mTl2r*Pc;
    // Check positive depth
    if(PcZ<0.0f || PrXYZ[2]<0.0)
        return false;

    // Project in left and right image and check it is not outside
    const float invz = 1.0/PcZ;
    const float u=cam_.fx()*PcX*invz+cam_.cx();
    const float v=cam_.fy()*PcY*invz+cam_.cy();
    const double invzr= 1.0/ PrXYZ[2];
    Eigen::Matrix3d eigRightK= right_cam_.K();
    double ur= eigRightK(0,0)*PrXYZ[0]*invzr + eigRightK(0,2);
    double vr= eigRightK(1,1)*PrXYZ[1]*invzr + eigRightK(1,2);
    if(u<mnMinX || u>mnMaxX || ur<mnMinX || ur>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY|| vr<mnMinY || vr>mnMaxY )
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3d PO = P-mOw;
    const float dist = PO.norm();

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    Eigen::Vector3d Pn = pMP->GetNormal();

    float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale level acording to the distance
    float ratio = dist/minDistance;

    vector<float>::iterator it = lower_bound(mpORBextractor->mvScaleFactor.begin(), mpORBextractor->mvScaleFactor.end(), ratio);
    int nPredictedLevel = it-mpORBextractor->mvScaleFactor.begin();

    if(nPredictedLevel>=GetScaleLevels())
        nPredictedLevel=GetScaleLevels()-1;

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, int minLevel, int maxLevel) const
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

    bool bCheckLevels=true;
    bool bSameLevel=false;
    if(minLevel==-1 && maxLevel==-1)
        bCheckLevels=false;
    else
        if(minLevel==maxLevel)
            bSameLevel=true;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> & vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels && !bSameLevel)
                {
                    if(kpUn.octave<minLevel || kpUn.octave>maxLevel)
                        continue;
                }
                else if(bSameLevel)
                {
                    if(kpUn.octave!=minLevel)
                        continue;
                }

                if(abs(kpUn.pt.x-x)>r || abs(kpUn.pt.y-y)>r)
                    continue;

                vIndices.push_back(vCell[j]);
            }
        }
    }
    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        std::vector<cv::Mat> vDesc;
        vDesc.reserve(mDescriptors.rows);
        for (int j=0;j<mDescriptors.rows;j++){       
            vDesc.push_back(mDescriptors.row(j));
        }
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vDesc,mBowVec,mFeatVec,4);
    }
}
//void CreatePMatch(const Frame &F1, const Frame &F2, const vector<int>& vMatches, vector<p_match>& p_matched)
//{
//    p_matched.reserve(vMatches.size());
//    int counter=0;
//    for(std::vector<int>::const_iterator it=vMatches.begin(),
//        itEnd= vMatches.end(); it!=itEnd; ++it, ++counter){
//        if(*it>=0)
//        {
//            int i1p=counter;
//            float u1p= F1.mvKeysUn[counter].pt.x;
//            float v1p= F1.mvKeysUn[counter].pt.y;
//            float u2p= F1.mvRightKeysUn[counter].pt.x;
//            float v2p= F1.mvRightKeysUn[counter].pt.y;

//            int i1c= *it;
//            float u1c= F2.mvKeysUn[i1c].pt.x;
//            float v1c= F2.mvKeysUn[i1c].pt.y;
//            float u2c= F2.mvRightKeysUn[i1c].pt.x;
//            float v2c= F2.mvRightKeysUn[i1c].pt.y;
//            p_matched.push_back(p_match(u1p,v1p,i1p,u2p,v2p,i1p,
//                                        u1c, v1c, i1c, u2c, v2c, i1c));
//        }
//    }
//}

// Input: vQuadMatches are quad matches obtained by viso2, with indices of viso2 features
// this function maps viso2 feature ids to ids in a frame such that a match has the same id in the left and right image
// output: vQuadMatches with indices of features in a frame
// also test whether a quad match satisfy stereo matches
void remapQuadMatches(std::vector<p_match>& vQuadMatches,
                 const std::vector<int> & currLeftId2StereoId, const std::vector<int> & currRightId2StereoId,
                      const std::vector<int> & prevLeftId2StereoId, const std::vector<int> & prevRightId2StereoId)
{    
    int numOusted=0;
    for(vector<p_match>::iterator vIt=vQuadMatches.begin(); vIt!= vQuadMatches.end(); )
    {
        p_match& match= *vIt;
        if(match.i1c==-1){ // possibly reason: outlier or mixed match
            vIt=vQuadMatches.erase(vIt);
            ++numOusted;
        }
        else if(currLeftId2StereoId[match.i1c] == currRightId2StereoId[match.i2c] && currLeftId2StereoId[match.i1c]!=-1 &&
                prevLeftId2StereoId[match.i1p] == prevRightId2StereoId[match.i2p] && prevLeftId2StereoId[match.i1p]!=-1 )
        {
            match.i1c=currLeftId2StereoId[match.i1c];
            match.i2c= match.i1c;
            match.i1p= prevLeftId2StereoId[match.i1p];
            match.i2p= match.i1p;
            ++vIt;
        }
        else
        {
            vIt=vQuadMatches.erase(vIt);
            ++numOusted;
        }
    }
 //   SLAM_DEBUG_STREAM("numOusted quad matches and rest quad matches:"<<numOusted<<" "<< vQuadMatches.size());
#if 0
    cerr<<"Mismatch between stereo and quad match:"<<numOusted<<endl;
    if(vQuadMatches.size()==0)
        return;
    vector<int> vi1c;
    vi1c.resize(vQuadMatches.size());
    for(size_t i = 0; i < vi1c.size(); i++)
        vi1c[i]=vQuadMatches[i].i1c;

    std::sort(vi1c.begin(), vi1c.end());
    for(size_t i = 0; i < vi1c.size()-1; i++) {
        if (vi1c[i] == vi1c[i + 1]) {
            assert(false);
        }
    }
    for(size_t i = 0; i < vi1c.size(); i++)
        vi1c[i]=vQuadMatches[i].i1p;

    std::sort(vi1c.begin(), vi1c.end());
    for(size_t i = 0; i < vi1c.size()-1; i++) {
        if (vi1c[i] == vi1c[i + 1]) {
            assert(false);
        }
    }
#endif

}
void Frame::ComputeImageBounds()
{
    cv::Mat fK=Converter::toCvMat(cam_.K());
    cv::Mat fDistCoef=(cv::Mat_<float>(4,1)<< cam_.d0(),cam_.d1(), cam_.d2(), cam_.d3());

    if(cam_.d0()!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=cam_.width(); mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=cam_.height();
        mat.at<float>(3,0)=cam_.width(); mat.at<float>(3,1)=cam_.height();

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,fK,fDistCoef,cv::Mat(),fK);
        mat=mat.reshape(1);

        snMinX = min(floor(mat.at<float>(0,0)),floor(mat.at<float>(2,0)));
        snMaxX = max(ceil(mat.at<float>(1,0)),ceil(mat.at<float>(3,0)));
        snMinY = min(floor(mat.at<float>(0,1)),floor(mat.at<float>(1,1)));
        snMaxY = max(ceil(mat.at<float>(2,1)),ceil(mat.at<float>(3,1)));

    }
    else
    {
        snMinX = 0;
        snMaxX = cam_.width();
        snMinY = 0;
        snMaxY = cam_.height();
    }
}

void Frame::SetFirstEstimate()
{
    assert( mbFixedLinearizationPoint==false);
    mbFixedLinearizationPoint=true;
    mTcw_first_estimate= mTcw;
    speed_bias_first_estimate= speed_bias;
}
vector<MapPoint*> Frame::GetMapPointMatches()
{
    return mvpMapPoints;
}


void Frame::SetIMUObservations(const std::vector<Eigen::Matrix<double, 7, 1> >& imu_meas)
{
    imu_observ=imu_meas;
}
// last_frame is the frame connected to this frame by IMU readings
void Frame::SetPrevNextFrame(Frame* last_frame)
{
    prev_frame=last_frame;
    last_frame->next_frame= this;
}
void Frame::SetPose(const Sophus::SE3d &Tcw_)
{
    mTcw=Tcw_;
    mOw = mTcw.inverse().translation();
}

void Frame::SetPose(const Eigen::Matrix3d &Rcw,const Eigen::Vector3d &tcw)
{
    mTcw= Sophus::SE3d(Rcw, tcw);
    mOw=-Rcw.transpose()*tcw;
}
Sophus::SE3d Frame::GetPose(bool left)
{
    if(left)
        return mTcw;
    else
        return (mTl2r*mTcw);

}
Eigen::Matrix3d Frame::GetRotation()
{
    return mTcw.rotationMatrix();
}
void Frame::EraseMapPointMatch(const size_t &idx)
{
    assert(mvpMapPoints[idx]);
    mvpMapPoints[idx]=NULL;
  
}

Eigen::Vector3d Frame::GetCameraCenter()
{
    return mOw;
}
void Frame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    assert(pMP);
    mvpMapPoints[idx]=pMP;
   
}
cv::Mat Frame::GetDescriptor(const size_t &idx, bool left)
{
    if(left)
        return mDescriptors.row(idx).clone();
    else
        return mRightDescriptors.row(idx).clone();
}

void Frame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    assert(pMP);
    mvpMapPoints[idx]=pMP;
   
}
cv::KeyPoint Frame::GetKeyPointUn(const size_t &idx, bool left) const
{
    if(left)
    return mvKeysUn[idx];
    else
    return mvRightKeysUn[idx];
}

MapPoint* Frame::GetMapPoint(const size_t &idx)
{
    return mvpMapPoints[idx];
}

Eigen::Matrix3d Frame::ComputeFlr(Frame* pRightF, const Sophus::SE3d & Tl2r)
{// 1 is left, 2 is right
    Sophus::SE3d Tr2l= Tl2r.inverse();
    Eigen::Matrix3d R12 = Tr2l.rotationMatrix();
    Eigen::Vector3d t12 = Tr2l.translation();
    Eigen::Matrix3d t12x = vio::skew3d(t12);
    return (cam_.K_inv().transpose())*t12x*R12*(pRightF->cam_.K_inv());
}

float Frame::ComputeSceneMedianDepth(int q)
{
    vector<MapPoint*> vpMapPoints;
    Sophus::SE3d Tcw_;
    {
     vpMapPoints = mvpMapPoints;
    Tcw_ = mTcw;
    }

    vector<float> vDepths;
    vDepths.reserve(mvpMapPoints.size());
    Eigen::Matrix<double, 1,3> Rcw2 = Tcw_.rotationMatrix().row(2);
    float zcw = Tcw_.translation()[2];
    for(size_t i=0; i<mvpMapPoints.size(); ++i)
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
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}

bool getSceneDepth(Frame& frame, double& depth_mean, double& depth_min)
{
  vector<double> depth_vec;
  depth_vec.reserve(frame.mvpMapPoints.size());
  depth_min = std::numeric_limits<double>::max();
  for(auto it=frame.mvpMapPoints.begin(), ite=frame.mvpMapPoints.end(); it!=ite; ++it)
  {
    if((*it)!= NULL)
    {
        Eigen::Vector3d v3Temp=frame.GetPose()*((*it)->GetWorldPos());
      const double z = v3Temp[2];
      depth_vec.push_back(z);
      depth_min = fmin(z, depth_min);
    }
  }
  if(depth_vec.empty())
  {
    SLAM_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }
  depth_mean = vk::getMedian(depth_vec);
  return true;
}


} //namespace ORB_SLAM
