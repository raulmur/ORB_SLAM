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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

//#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace ORB_SLAM
{

class LoopClosing;

class Optimizer
{
public:
    // used by tracking thread
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP, int nIterations = 5, bool *pbStopFlag=NULL);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL); // optimize initial map
    int static PoseOptimization(Frame* pFrame, Map* pMap=NULL); // optimize w.r.t tracked mappoints, used in track last frame and relocalization

    // functions used by LocalOptimizer in tracking thread
    void static setupG2o(ScaViSLAM::G2oCameraParameters * g2o_cam,
                  ScaViSLAM::G2oCameraParameters * g2o_cam_right,
               ScaViSLAM::G2oIMUParameters * g2o_imu,
               g2o::SparseOptimizer * optimizer);

    static ScaViSLAM::G2oVertexSE3* addPoseToG2o(const Sophus::SE3d & T_me_from_w,
                   int pose_id,
                   bool fixed,
                   g2o::SparseOptimizer * optimizer,
                      const Sophus::SE3d* first_estimate=NULL);
    static ScaViSLAM::G2oVertexSpeedBias* addSpeedBiasToG2o(const Eigen::Matrix<double, 9,1> & vinw_bias,
                   int sb_id,
                   bool fixed,
                   g2o::SparseOptimizer * optimizer,
                           const Eigen::Matrix<double, 9,1> * first_estimate= NULL);
    static ScaViSLAM::G2oVertexPointXYZ* addPointToG2o( MapPoint* pPoint,
                int g2o_point_id, bool fixed,
                g2o::SparseOptimizer * optimizer);

    static ScaViSLAM::G2oEdgeProjectXYZ2UV* addObsToG2o(const Eigen::Vector2d & obs, const Eigen::Matrix2d & Lambda,
              ScaViSLAM::G2oVertexPointXYZ*, ScaViSLAM::G2oVertexSE3*, bool robustify,  double huber_kernel_width,
              g2o::SparseOptimizer * optimizer, Sophus::SE3d * pTs2c=NULL);

    static size_t copyAllPosesToG2o(g2o::SparseOptimizer * optimizer, const std::vector<KeyFrame*> vpLocalKeyFrames,
                                       std::deque<Frame *> vpTemporalFrames, Frame * pCurrentFrame, Frame * pLastFrame=NULL,
                                       bool bUseIMUData= false);

    /// Temporary container to hold the g2o edge with reference to frame and point.
    struct EdgeContainerSE3d
    {
        ScaViSLAM::G2oEdgeProjectXYZ2UV*     edge;
        Frame*          frame;
        size_t id_; // id in frame feature vector
        bool            is_deleted;
        EdgeContainerSE3d(ScaViSLAM::G2oEdgeProjectXYZ2UV* e, Frame* frame, size_t id) :
            edge(e), frame(frame),id_(id),is_deleted(false)
        {}
    };
    static const int MAGIC2=2;         //we use 2*i to identify pose vertices and 2*i+1 for speeb bias vertices in g2o optimizer
    static int  LocalOptimize(vk::PinholeCamera * cam,
                              Map* pMap, const std::vector<KeyFrame*>& vpLocalKeyFrames,
                              std::vector<MapPoint*>& vpLocalMapPoints,
                              const std::deque<Frame*>& vpTemporalFrames, Frame* pCurrentFrame,
                              Frame *pLastFrame =NULL, ScaViSLAM::G2oIMUParameters* imu =NULL,
                              vk::PinholeCamera * right_cam =NULL, Sophus::SE3d * pTl2r= NULL);

    // used by local mapping thread
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag=NULL);

    // used by loopclosing thread
    // pLoopKF, pointer of loop keyframe, pCurKF, pointer of current keyframe, Scurw, similarity transform from world frame to current keyframe
    // NonCorrectedSim3, mpCurrentConnectedKFs and their poses T_w^i, CorrectedSim3, mpCurrentConnectedKFs and their poses S_w^i,
    // mpCurrentConnectedKFs are keyframes connected to current keyframe after loop detection, including the current keyframe
    // for current keyframe S_w^{cur} = S_{loop}^{cur}S_w^{loop}, for other keyframes, S_w^i= S(T_{cur}^i)S_w^{cur}
    // before calling this functions, positions of mappoints observed in mpCurrentConnectedKFs and their poses are corrected using CorrectedSim3
    //
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       std::map<KeyFrame*, set<KeyFrame*> > &LoopConnections);

    void static OptimizeEssentialGraphSE3(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, Sophus::SE3d &Tcurw,
                                           LoopClosing::KeyFrameAndSE3Pose &NonCorrectedSE3,
                                           LoopClosing::KeyFrameAndSE3Pose &CorrectedSE3,
                                           map<KeyFrame *, set<KeyFrame *> > &LoopConnections);

    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2 = 10);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
