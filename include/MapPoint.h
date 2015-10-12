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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"Frame.h"
#include"Map.h"
#include "g2o/types/sba/types_sba.h"
#include<opencv2/core/core.hpp>
#include<boost/thread.hpp>

namespace ScaViSLAM
{
class G2oVertexPointXYZ;
}
namespace ORB_SLAM
{

class ImageFeature;
class KeyFrame;
class Map;
typedef Eigen::Matrix<double, 2, 3> Matrix23d;

class MapPoint
{

public:
 
    MapPoint(const Eigen::Vector3d &Pos, KeyFrame* pRefKF, const int nIDInKF, Map* pMap);
    void SetWorldPos(const Eigen::Vector3d &Pos);
    Eigen::Vector3d GetWorldPos();

    Eigen::Vector3d GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations(bool left=true);
    int Observations();

    /// Add a reference to a frame.
    void AddObservation(KeyFrame* pKF,size_t idx, bool left=true);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF, bool left=true);
    bool IsInKeyFrame(KeyFrame* pF);
    int IdInKeyFrame(KeyFrame *pF);
    void SetBadFlag();
   
    bool isBad();

    void Replace(MapPoint* pMP);

    void IncreaseVisible();
    void IncreaseFound();
    float GetFoundRatio();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();
    void SetDescriptor(const cv::Mat &);
    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    void SetFirstEstimate();
    void Release();


    /// Remove reference to a frame.
    bool deleteFrameRef(KeyFrame* frame);



    /// Get number of observations.
    inline size_t nRefs() const { return mObservations.size(); }

    /// Optimize point position through minimizing the reprojection error.
    static void optimize(const std::vector< Eigen::Vector3d> & obs,
                            const std::vector<Sophus::SE3d> & frame_poses,
                            Eigen::Vector3d& old_point, double & pdop,
                            vk::PinholeCamera* cam, size_t n_iter=5);

    /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
    inline static void jacobian_xyz2uv(
        const Eigen::Vector3d& p_in_f,
        const Eigen::Matrix3d& R_f_w,
        Matrix23d& point_jac)
    {
      const double z_inv = 1.0/p_in_f[2];
      const double z_inv_sq = z_inv*z_inv;
      point_jac(0, 0) = z_inv;
      point_jac(0, 1) = 0.0;
      point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
      point_jac(1, 0) = 0.0;
      point_jac(1, 1) = z_inv;
      point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
      point_jac = - point_jac * R_f_w;
    }

public:

    long unsigned int mnId;
    static long unsigned int nNextId;
    long unsigned int mnFirstKFid; // id of the first kf that observes this point, remain constant once initialized

    // Variables used by TrackLocalMap()
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView; //do we need to track this mappoint in the current frame
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;
    unsigned int mnObservationsInDoubleWindow;
    // how many observations of this map point exist in KEYframes of the double window
    // it is to be used in the local optimizer of the tracking thread. N.B. (1) the double window
    // does not include the current frame for current implementation, (2) TODO: a map point's observation in a frame
    // is selectively used for optimization based on a quad tree to ensure uniform distribution

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    bool mbFixedLinearizationPoint; //for first estimate Jacobians

    Eigen::Vector3d mWorldPos_first_estimate;
    // Reference KeyFrame
    KeyFrame* mpRefKF;  
    int                         last_structure_optim_;    //!< Timestamp of last point optimization
    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;
    std::map<KeyFrame*,size_t> mRightObservations;

    // Tracking counters
    int mnVisible;
    int mnFound;
    ScaViSLAM::G2oVertexPointXYZ *                   v_pt_;                    //!< Temporary pointer to the point-vertex in g2o during bundle adjustment.
    Map* mpMap;

protected:    
     // Position in absolute coordinates
     Eigen::Vector3d mWorldPos;

     //because we match a map point to left frame, to right frame, and match left frame to right frame,
     // mObservations.size()==mRightObservations.size()

     // Mean viewing direction
     Eigen::Vector3d mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;


     boost::mutex mMutexPos;
     boost::mutex mMutexFeatures;

private:
     MapPoint & operator=(const MapPoint&);
     MapPoint(const MapPoint&);

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
