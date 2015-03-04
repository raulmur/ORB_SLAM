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


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include<boost/thread.hpp>


namespace ORB_SLAM
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class DatabaseResult;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Rcw,const cv::Mat &tcw);
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Calibration
    cv::Mat GetProjectionMatrix();
    cv::Mat GetCalibrationMatrix() const;

    // Bag of Words Representation
    void ComputeBoW();
    DBoW2::FeatureVector GetFeatureVector();
    DBoW2::BowVector GetBowVector();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints();
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    cv::KeyPoint GetKeyPointUn(const size_t &idx) const;
    cv::Mat GetDescriptor(const size_t &idx);
    int GetKeyPointScaleLevel(const size_t &idx) const;
    std::vector<cv::KeyPoint> GetKeyPoints() const;
    std::vector<cv::KeyPoint> GetKeyPointsUn() const;
    cv::Mat GetDescriptors();
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

    // Image
    cv::Mat GetImage();
    bool IsInImage(const float &x, const float &y) const;

    // Activate/deactivate erasable flags
    void SetNotErase();
    void SetErase();

    // Set/check erased
    void SetBadFlag();
    bool isBad();

    // Scale functions
    float inline GetScaleFactor(int nLevel=1) const{
        return mvScaleFactors[nLevel];}
    std::vector<float> inline GetScaleFactors() const{
        return mvScaleFactors;}
    std::vector<float> inline GetVectorScaleSigma2() const{
        return mvLevelSigma2;}
    float inline GetSigma2(int nLevel=1) const{
        return mvLevelSigma2[nLevel];}
    float inline GetInvSigma2(int nLevel=1) const{
        return mvInvLevelSigma2[nLevel];}
    int inline GetScaleLevels() const{
        return mnScaleLevels;}

    // Median MapPoint depth
    float ComputeSceneMedianDepth(int q = 2);

public:
    static long unsigned int nNextId;
    long unsigned int mnId;
    long unsigned int mnFrameId;

    double mTimeStamp;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    float mfGridElementWidthInv;
    float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Calibration parameters
    float fx, fy, cx, cy;

    //BoW
    DBoW2::BowVector mBowVec;

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Ow;

    // Original image, undistorted image bounds, and calibration matrix
    cv::Mat im;
    int mnMinX;
    int mnMinY;
    int mnMaxX;
    int mnMaxY;
    cv::Mat mK;

    // KeyPoints, Descriptors, MapPoints vectors (all associated by an index)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    cv::Mat mDescriptors;
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;
    DBoW2::FeatureVector mFeatVec;


    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Erase flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    // Scale
    int mnScaleLevels;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    Map* mpMap;

    boost::mutex mMutexPose;
    boost::mutex mMutexConnections;
    boost::mutex mMutexFeatures;
    boost::mutex mMutexImage;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
