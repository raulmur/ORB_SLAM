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

#ifndef FRAME_H
#define FRAME_H
#include "vio/PointStatistics.h"
#include "vio_g2o/anchored_points.h"
#include <Eigen/Dense>
#include <viso2/p_match.h> //for matches adopted from libviso2

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "sophus/se3.hpp"
#include "boost/shared_ptr.hpp"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;


typedef std::vector<cv::Mat> ImgPyr;

class Frame
{
public:
    Frame(const Frame &frame);

    //monocular
    Frame(cv::Mat &im, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc,
          vk::PinholeCamera* cam,  const Eigen::Vector3d ginc=Eigen::Vector3d::Zero(),
          const Eigen::Matrix<double, 9,1> sb=Eigen::Matrix<double, 9,1>::Zero());
    // stereo and viso2 stereo matches
    Frame(cv::Mat &im , const double &timeStamp, const int num_features_left,
          cv::Mat &right_img, const int num_features_right,
          const std::vector<p_match> & vStereoMatches, ORBextractor* extractor, ORBVocabulary* voc,
          vk::PinholeCamera* cam, vk::PinholeCamera* right_cam,
          const Sophus::SE3d& Tl2r, const Eigen::Vector3d &ginc, const Eigen::Matrix<double, 9,1> sb);

    // Release part of the memory occupied by frame before copying into the base of a new keyframe
    void PartialRelease();

    virtual ~Frame();
    virtual bool isKeyFrame() const {return false;}
    virtual void Release();
    virtual vector<MapPoint*> GetMapPointMatches();

    virtual void SetPose(const Sophus::SE3d &Tcw_);
    virtual void SetPose(const Eigen::Matrix3d &Rcw,const Eigen::Vector3d &tcw);
    virtual Sophus::SE3d GetPose(bool left=true);
    virtual Eigen::Matrix3d GetRotation();
    virtual void EraseMapPointMatch(const size_t &idx);
    virtual Eigen::Vector3d GetCameraCenter();
    virtual void AddMapPoint(MapPoint* pMP, const size_t &idx);
  
    virtual MapPoint* GetMapPoint(const size_t &idx);
 
    virtual float ComputeSceneMedianDepth(int q = 2);
    /// Projects Point from unit sphere (f) in camera pixels (c).
    inline Eigen::Vector2d f2c(const Eigen::Vector3d& f) const { return cam_.world2cam( f ); }
    /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
    inline Eigen::Vector2d w2c(const Eigen::Vector3d& xyz_w) const { return cam_.world2cam( mTcw * xyz_w ); }

    /// Frame jacobian for projection of 3D point in (f)rame coordinate to
    /// unit plane coordinates uv (focal length = 1).
    /// $\frac{\partial (z-exp(\epsilon)\mathbf{X})}{\partial \epsilon(\upsilon, \omega)}$
    inline static void jacobian_xyz2uv(
        const Eigen::Vector3d& xyz_in_f,
        Eigen::Matrix<double,2,6>& J)
    {
      const double x = xyz_in_f[0];
      const double y = xyz_in_f[1];
      const double z_inv = 1./xyz_in_f[2];
      const double z_inv_2 = z_inv*z_inv;

      J(0,0) = -z_inv;              // -1/z
      J(0,1) = 0.0;                 // 0
      J(0,2) = x*z_inv_2;           // x/z^2
      J(0,3) = y*J(0,2);            // x*y/z^2
      J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
      J(0,5) = y*z_inv;             // y/z

      J(1,0) = 0.0;                 // 0
      J(1,1) = -z_inv;              // -1/z
      J(1,2) = y*z_inv_2;           // y/z^2
      J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
      J(1,4) = -J(0,3);             // -x*y/z^2
      J(1,5) = -x*z_inv;            // -x/z
    }

inline void updatePointStatistics(PointStatistics* stats){
      stats->num_points_grid2x2.setZero();
      stats->num_points_grid3x3.setZero();
      int half_width = Frame::mnMaxX*0.5;
      int half_height = Frame::mnMaxY*0.5;

      float third = 1./3.;
      int third_width = cam_.width()*third;
      int third_height = cam_.height()*third;
      int twothird_width = cam_.width()*2*third;
      int twothird_height = cam_.height()*2*third;
      size_t jack=0;
      Frame * frame= this;
      for (auto it=frame->mvpMapPoints.begin(), ite= frame->mvpMapPoints.end();
           it!=ite; ++it, ++jack)
      {
          if((*it)==NULL) continue;
          const cv::Point2f & uv = frame->mvKeysUn[jack].pt;
          int i = 1;
          int j = 1;
          if (uv.x<half_width)
              i = 0;
          if (uv.y<half_height)
              j = 0;
          ++(stats->num_points_grid2x2(i,j));

          i = 2;
          j = 2;
          if (uv.x<third_width)
              i = 0;
          else if (uv.x<twothird_width)
              i = 1;
          if (uv.y<third_height)
              j = 0;
          else if (uv.y<twothird_height)
              j = 1;
          ++(stats->num_points_grid3x3(i,j));
      }
  }

    Eigen::Matrix3d ComputeFlr(Frame* pRightF, const Sophus::SE3d & Tl2r);
    void SetIMUObservations(const std::vector<Eigen::Matrix<double, 7, 1> >&  );
    void SetPrevNextFrame(Frame* last_frame);

    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    cv::KeyPoint GetKeyPointUn(const size_t &idx, bool left=true) const;
    cv::Mat GetDescriptor(const size_t &idx, bool left=true);

    // Scale functions
    float inline GetSigma2(int nLevel=1) const{
        if( nLevel==1 && mpORBextractor->nlevels ==1)
            return 1.44f;
        return mvLevelSigma2[nLevel];
    }

    float inline GetScaleFactor(int nLevel=1) const{
        if( nLevel==1 && mpORBextractor->nlevels ==1)
            return 1.2f;
        return mvScaleFactors[nLevel];
    }
    std::vector<float> inline GetScaleFactors() const{        
        return mvScaleFactors;
    }
    std::vector<float> inline GetVectorScaleSigma2() const{
        return mvLevelSigma2;
    }

    float inline GetInvSigma2(int nLevel) const{
        return mvInvLevelSigma2[nLevel];
    }
    int inline GetScaleLevels() const{
        return mnScaleLevels;
    }

    void ComputeBoW();
    void UpdatePoseMatrices();

    // Check if a MapPoint is in the frustum of the camera and also fills variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    bool isInFrustumStereo(MapPoint* pMP, float viewingCosLimit);
    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r,
                                     const int minLevel=-1, const int maxLevel=-1) const;
    void SetFirstEstimate();

    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;

    // Frame timestamp
    double mTimeStamp;
    // Camera Pose
    Sophus::SE3d mTcw;
    Eigen::Vector3d mOw;
    Frame* prev_frame;// the previous frame (k-1) linked to this frame (k) by imu measurements, both frame having same cam_id_
    Frame* next_frame; //next frame (k+1) connected to this frame (k) by imu measurements, both frame having same cam_id_
    Eigen::Matrix<double, 9,1> speed_bias; //IMU states, vel_imu in world frame, accelerometer bias and gyro bias, at the epoch of this frame( of index k)
    std::vector<Eigen::Matrix<double, 7,1> > imu_observ;// IMU readings from t(p(k-1)-1) to t(p(k)-1) where k is this image index in stream
    // t(p(k)-1)<=t(k) and t(p(k)+1) must>t(k)
    // members for first estimate Jacoabians
    bool mbFixedLinearizationPoint;
    Eigen::Matrix<double, 9,1> speed_bias_first_estimate;
    Sophus::SE3d mTcw_first_estimate;

    // Calibration Matrix and k1,k2,p1,p2 Distortion Parameters
    vk::PinholeCamera           cam_;                   //!< Camera model.
    vk::PinholeCamera           right_cam_;             //!< Camera model.
    Sophus::SE3d mTl2r; // transform from left to right camera

    // Number of KeyPoints
    int N;
    // huai: mvKeys, mvKeysUn,mvRightKeys, mvRightKeysUn, mDescriptors,mRightDescriptors, mvpMapPoints, mvbOutliers
    // have the same length N, because we only create keys from stereo matches detected by libviso2
    // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;

    std::vector<cv::KeyPoint> mvRightKeys;
    std::vector<cv::KeyPoint> mvRightKeysUn;

    // Bag of Words Vector structures
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint
    cv::Mat mDescriptors;
    cv::Mat mRightDescriptors;

    // MapPoints(not candidate or deleted) associated to keypoints, NULL pointer if not associated
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]; //used by GetFeaturesInArea()

    // Current and Next Frame id
    static long unsigned int nNextId;
    long unsigned int mnId; //mnId has the same meaning in derived KeyFrame class and in base Frame class, it is supposed to be continuous for frames

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once)
    float mnMinX;
    float mnMaxX;
    float mnMinY;
    float mnMaxY;

    static float snMinX;
    static float snMaxX;
    static float snMinY;
    static float snMaxY;

    static bool mbInitialComputations;
    vector<int> viso2LeftId2StereoId;
    vector<int> viso2RightId2StereoId; // this two members convert the id of a feature indexed by viso2 to that used in ORB_SLAM
    bool mbBad;
    // Call UpdatePoseMatrices(), before using the following
    Eigen::Matrix3d mRcw;
    Eigen::Vector3d mtcw;

    vio::G2oVertexSE3*                   v_kf_;                  //!< Temporary pointer to the g2o node object of the keyframe.
    vio::G2oVertexSpeedBias*        v_sb_; //!< temporary pointer to g2o speed bias vertex
private:
    void ComputeImageBounds();
    Frame& operator= (const Frame&);
};
// given matched features between F1 and F2, put them into p_match structure
void CreatePMatch(const Frame &F1, const Frame &F2, const vector<int>& vMatches, vector<p_match>& p_matched);

void remapQuadMatches(std::vector<p_match>& vQuadMatches, const std::vector<int> & currLeftId2StereoId, const std::vector<int> & currRightId2StereoId,
                      const std::vector<int> & prevLeftId2StereoId, const std::vector<int> & prevRightId2StereoId);
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);
bool getSceneDepth(Frame& frame, double& depth_mean, double& depth_min);
typedef boost::shared_ptr<Frame> FramePtr;

}// namespace ORB_SLAM

#endif // FRAME_H
