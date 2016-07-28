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

#include "Optimizer.h"
#include "g2o_types/IMU_constraint.h"
#include "g2o_types/scale_solver.h"

#include "global.h" //SLAM_DEBUG_STREAM
//#include "Thirdparty/g2o/g2o/core/block_solver.h"
//#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
//#include "Thirdparty/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
//#include "Thirdparty/g2o/g2o/types/slam3d/edge_se3.h"
//#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
//#include "Thirdparty/g2o/g2o/solvers/dense/linear_solver_dense.h"
//#include "Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#include<Eigen/StdVector>

#include "Converter.h"

namespace ORB_SLAM
{
using namespace ScaViSLAM;
using namespace Sophus;
using namespace Eigen;

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag);
}

//TODO: adapt this function with local g2o types
void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP, int nIterations, bool* pbStopFlag)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    Map *pMap = vpKFs[0]->GetMap();
    pMap->mPointPoseConsistencyMutex.lock();
    // SET KEYFRAME VERTICES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnFrameId);
        vSE3->setFixed(pKF->mnFrameId==0);
        optimizer.addVertex(vSE3);
        if(pKF->mnFrameId>maxKFid)
            maxKFid=pKF->mnFrameId;
    }

    const float thHuber = sqrt(5.991);

    // SET MAP POINT VERTICES
    for(size_t i=0, iend=vpMP.size(); i<iend;i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos());
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //SET EDGES
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad())
                continue;
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKF->GetKeyPointUn(mit->second);
            obs << kpUn.pt.x, kpUn.pt.y;

            ScaViSLAM::EdgeSE3ProjectXYZ* e = new ScaViSLAM::EdgeSE3ProjectXYZ();

            e->setVertex(0, optimizer.vertex(id));
            e->setVertex(1, optimizer.vertex(pKF->mnFrameId));
            e->setMeasurement(obs);
            float invSigma2 = pKF->GetInvSigma2(kpUn.octave);
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber);

            e->fx = pKF->cam_.fx();
            e->fy = pKF->cam_.fy();
            e->cx = pKF->cam_.cx();
            e->cy = pKF->cam_.cy();

            optimizer.addEdge(e);
        }
    }

    pMap->mPointPoseConsistencyMutex.unlock();
    // Optimize!

    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data
    pMap->mPointPoseConsistencyMutex.lock();
    //Keyframes
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnFrameId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toSE3d(SE3quat));
    }

    //Points
    for(size_t i=0, iend=vpMP.size(); i<iend;i++)
    {
        MapPoint* pMP = vpMP[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate());
        pMP->UpdateNormalAndDepth();
    }
    pMap->mPointPoseConsistencyMutex.unlock();
}

int Optimizer::PoseOptimization(Frame *pFrame, Map * pMap)
{    
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    int nInitialCorrespondences=0;

    // SET FRAME VERTEX
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // SET MAP POINT VERTICES
//    pMap->mPointPoseConsistencyMutex.lock();
//    if(pMap->mbFinishedLoopClosing)
//        pMap->mbFinishedLoopClosing =false;

    vector<ScaViSLAM::EdgeSE3ProjectXYZ*> vpEdges;
    vector<g2o::VertexSBAPointXYZ*> vVertices;
    vector<float> vInvSigmas2;
    vector<size_t> vnIndexEdge;

    const int N = pFrame->mvpMapPoints.size();
    vpEdges.reserve(N);
    vVertices.reserve(N);
    vInvSigmas2.reserve(N);
    vnIndexEdge.reserve(N);

    const float delta = sqrt(5.991);

    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos());
            vPoint->setId(i+1);
            vPoint->setFixed(true);
            optimizer.addVertex(vPoint);
            vVertices.push_back(vPoint);

            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            //SET left EDGE
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            ScaViSLAM::EdgeSE3ProjectXYZ* e = new ScaViSLAM::EdgeSE3ProjectXYZ();

            e->setVertex(0, optimizer.vertex(i+1));
            e->setVertex(1, optimizer.vertex(0));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->GetInvSigma2(kpUn.octave);
            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(delta);

            e->fx = pFrame->cam_.fx();
            e->fy = pFrame->cam_.fy();
            e->cx = pFrame->cam_.cx();
            e->cy = pFrame->cam_.cy();
            e->setLevel(0);
            optimizer.addEdge(e);

            vpEdges.push_back(e);
            vInvSigmas2.push_back(invSigma2);
            vnIndexEdge.push_back(i);

        }
    }
//    pMap->mPointPoseConsistencyMutex.unlock();
    // We perform 4 optimizations, decreasing the inlier region
    // From second to final optimization we include only inliers in the optimization
    // At the end of each optimization we check which points are inliers
    const float chi2[4]={9.210,7.378,5.991,5.991};
    const int its[4]={10,10,7,5};

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization();
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
        {
            ScaViSLAM::EdgeSE3ProjectXYZ* e = vpEdges[i];

            const size_t idx = vnIndexEdge[i];

            if(pFrame->mvbOutlier[idx])

                e->computeError();


            if(e->chi2()>chi2[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else if(e->chi2()<=chi2[it])
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }
        }

        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();  
    pFrame->SetPose(Sophus::SE3d(SE3quat_recov.rotation(), SE3quat_recov.translation()));

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag)
{    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnFrameId;

    vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnFrameId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnFrameId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnFrameId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnFrameId && pKFi->mnBAFixedForKF!=pKF->mnFrameId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnFrameId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    Map * pMap = pKF->GetMap();
    pMap->mPointPoseConsistencyMutex.lock();
    if(pMap->mbFinishedLoopClosing)
        pMap->mbFinishedLoopClosing = false;
    // SET LOCAL KEYFRAME VERTICES
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnFrameId);
        vSE3->setFixed(pKFi->mnFrameId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnFrameId>maxKFid)
            maxKFid=pKFi->mnFrameId;
    }

    // SET FIXED KEYFRAME VERTICES
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnFrameId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnFrameId>maxKFid)
            maxKFid=pKFi->mnFrameId;
    }

    // SET MAP POINT VERTICES
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<ScaViSLAM::EdgeSE3ProjectXYZ*> vpEdges;
    vpEdges.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKF;
    vpEdgeKF.reserve(nExpectedSize);

    vector<float> vSigmas2;
    vSigmas2.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdge;
    vpMapPointEdge.reserve(nExpectedSize);

    const float thHuber = sqrt(5.991);

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos());
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //SET EDGES
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                Eigen::Matrix<double,2,1> obs;
                cv::KeyPoint kpUn = pKFi->GetKeyPointUn(mit->second);
                obs << kpUn.pt.x, kpUn.pt.y;

                ScaViSLAM::EdgeSE3ProjectXYZ* e = new ScaViSLAM::EdgeSE3ProjectXYZ();

                e->setVertex(0, optimizer.vertex(id));
                e->setVertex(1, optimizer.vertex(pKFi->mnFrameId));
                e->setMeasurement(obs);
                float sigma2 = pKFi->GetSigma2(kpUn.octave);
                float invSigma2 = pKFi->GetInvSigma2(kpUn.octave);
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber);

                e->fx = pKFi->cam_.fx();
                e->fy = pKFi->cam_.fy();
                e->cx = pKFi->cam_.cx();
                e->cy = pKFi->cam_.cy();

                optimizer.addEdge(e);
                vpEdges.push_back(e);
                vpEdgeKF.push_back(pKFi);
                vSigmas2.push_back(sigma2);
                vpMapPointEdge.push_back(pMP);
            }
        }
    }
    pMap->mPointPoseConsistencyMutex.unlock();

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inlier observations
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        ScaViSLAM::EdgeSE3ProjectXYZ* e = vpEdges[i];
        MapPoint* pMP = vpMapPointEdge[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKF[i];
            pKFi->EraseMapPointMatch(pMP);
            pMP->EraseObservation(pKFi);

            optimizer.removeEdge(e);
            vpEdges[i]=NULL;
        }
    }

    // Recover optimized data
    pMap->mPointPoseConsistencyMutex.lock();
    if(pMap->mbFinishedLoopClosing){
        pMap->mbFinishedLoopClosing =false;
        pMap->mPointPoseConsistencyMutex.unlock();
        return;
    }
    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnFrameId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toSE3d(SE3quat));
    }
    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate());
        pMP->UpdateNormalAndDepth();
    }
    pMap->mPointPoseConsistencyMutex.unlock();
    // Optimize again without the outliers

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Check inlier observations
    for(size_t i=0, iend=vpEdges.size(); i<iend;i++)
    {
        ScaViSLAM::EdgeSE3ProjectXYZ* e = vpEdges[i];

        if(!e)
            continue;

        MapPoint* pMP = vpMapPointEdge[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKF = vpEdgeKF[i];
            pKF->EraseMapPointMatch(pMP->GetIndexInKeyFrame(pKF));
            pMP->EraseObservation(pKF);
        }
    }

    // Recover optimized data
    pMap->mPointPoseConsistencyMutex.lock();
    if(pMap->mbFinishedLoopClosing){
        pMap->mbFinishedLoopClosing =false;
        pMap->mPointPoseConsistencyMutex.unlock();
        return;
    }
    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnFrameId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toSE3d(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate());
        pMP->UpdateNormalAndDepth();
    }
    pMap->mPointPoseConsistencyMutex.unlock();
}
struct scalei2j{
    int i; int j; double si2j;
    scalei2j(int _i=-1, int _j=-1, double _si2j=1.0):
        i(_i), j(_j), si2j(_si2j)
    {}
};
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, g2o::Sim3 &Scurw,
                                       LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       map<KeyFrame *, set<KeyFrame *> > &LoopConnections)
{
#define OptSim3 1
    // Setup optimizer
#if OptSim3
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverCholmod<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<ScaViSLAM::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // SET KEYFRAME VERTICES
    pMap->mPointPoseConsistencyMutex.lock();
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        ScaViSLAM::VertexSim3Expmap* VSim3 = new ScaViSLAM::VertexSim3Expmap();

        int nIDi = pKF->mnFrameId;

        if(CorrectedSim3.count(pKF))
        {
            vScw[nIDi] = CorrectedSim3[pKF];
            VSim3->setEstimate(CorrectedSim3[pKF]);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = pKF->GetRotation();
            Eigen::Matrix<double,3,1> tcw = pKF->GetTranslation();
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }
    pMap->mPointPoseConsistencyMutex.unlock();

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // SET LOOP EDGES
    for(map<KeyFrame *, set<KeyFrame *> >::iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnFrameId;
        set<KeyFrame*> &spConnections = mit->second;
        g2o::Sim3 Siw = vScw[nIDi];
        g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnFrameId;
            if((nIDi!=pCurKF->mnFrameId || nIDj!=pLoopKF->mnFrameId) && pKF->GetWeight(*sit)<minFeat)
                continue;
            g2o::Sim3 Sjw = vScw[nIDj];
            g2o::Sim3 Sji = Sjw * Swi;

            ScaViSLAM::EdgeSim3* e = new ScaViSLAM::EdgeSim3();
            e->setVertex(1, optimizer.vertex(nIDj));
            e->setVertex(0, optimizer.vertex(nIDi));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // SET NORMAL EDGES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnFrameId;

        g2o::Sim3 Swi;
        if(NonCorrectedSim3.count(pKF))
            Swi = NonCorrectedSim3[pKF].inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnFrameId;

            g2o::Sim3 Sjw;

            if(NonCorrectedSim3.count(pParentKF))
                Sjw = NonCorrectedSim3[pParentKF];
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            ScaViSLAM::EdgeSim3* e = new ScaViSLAM::EdgeSim3();
            e->setVertex(1, optimizer.vertex(nIDj));
            e->setVertex(0, optimizer.vertex(nIDi));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnFrameId<pKF->mnFrameId)
            {
                g2o::Sim3 Slw;
                if(NonCorrectedSim3.count(pLKF))
                    Slw = NonCorrectedSim3[pLKF];
                else
                    Slw = vScw[pLKF->mnFrameId];

                g2o::Sim3 Sli = Slw * Swi;
                ScaViSLAM::EdgeSim3* el = new ScaViSLAM::EdgeSim3();
                el->setVertex(1, optimizer.vertex(pLKF->mnFrameId));
                el->setVertex(0, optimizer.vertex(nIDi));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnFrameId<pKF->mnFrameId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnFrameId,pKFn->mnFrameId),max(pKF->mnFrameId,pKFn->mnFrameId))))
                        continue;

                    g2o::Sim3 Snw;
                    if(NonCorrectedSim3.count(pKFn))
                        Snw = NonCorrectedSim3[pKFn];
                    else
                        Snw = vScw[pKFn->mnFrameId];

                    g2o::Sim3 Sni = Snw * Swi;

                    ScaViSLAM::EdgeSim3* en = new ScaViSLAM::EdgeSim3();
                    en->setVertex(1, optimizer.vertex(pKFn->mnFrameId));
                    en->setVertex(0, optimizer.vertex(nIDi));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // OPTIMIZE
    ROS_INFO("opt20 in %s", __func__);
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    pMap->mPointPoseConsistencyMutex.lock();
    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnFrameId;

        ScaViSLAM::VertexSim3Expmap* VSim3 = static_cast<ScaViSLAM::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();

        //[R t/s;0 1]
        Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation()/CorrectedSiw.scale());

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnFrameId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnFrameId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        pMP->SetWorldPos(eigCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
    pMap->mbFinishedLoopClosing =true;
    pMap->mPointPoseConsistencyMutex.unlock();

#else
    // this method works a little worse and slower
    // first svd solve for scale, second optimize scale + trans, optionally third optimize scale +trans +rot
    // Setup three optimizers
    const int num_optimizer = 2;
    g2o::SparseOptimizer optimizer[num_optimizer];
    for (int jack =0; jack< num_optimizer; ++jack){
        g2o::BlockSolverX::LinearSolverType * linearSolver;
        linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer[jack].setAlgorithm(solver);
    }
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
    unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<int> kfid2varid(nMaxKFid+1, -1); // map from keyframe id to scale variable id in scale SVD solver
    const int minFeat = 100;

    // SET KEYFRAME VERTICES
    pMap->mPointPoseConsistencyMutex.lock();
    for(size_t i=0, iend=vpKFs.size(); i<iend;++i)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        ScaViSLAM::G2oVertexScaleTrans* vST = new ScaViSLAM::G2oVertexScaleTrans();
        ScaViSLAM::VertexSim3Expmap* vSim3 = NULL;
        if(num_optimizer ==3)
            vSim3 = new ScaViSLAM::VertexSim3Expmap();

        int nIDi = pKF->mnFrameId;
        kfid2varid[nIDi]= i;
        if(CorrectedSim3.count(pKF))
        {
            vScw[nIDi] = CorrectedSim3[pKF];            
            vST->setEstimate(Converter::toScaleTrans(CorrectedSim3[pKF]));
            vST->Rw2i= SO3d(CorrectedSim3[pKF].rotation());
            if(num_optimizer ==3)
                vSim3->setEstimate(CorrectedSim3[pKF]);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = pKF->GetRotation();
            Eigen::Matrix<double,3,1> tcw = pKF->GetTranslation();
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;           
            vST->setEstimate(Converter::toScaleTrans(Siw));
            vST->Rw2i= SO3d(Rcw);
            if(num_optimizer ==3)
                vSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF){          
            vST->setFixed(true);
            if(num_optimizer==3)
                vSim3->setFixed(true);
        }

        vST->setId(nIDi);
        vST->setMarginalized(false);

        optimizer[1].addVertex(vST);
        if(num_optimizer ==3){
            vSim3->setId(nIDi);
            vSim3->setMarginalized(false);
            optimizer[2].addVertex(vSim3);
        }
    }
    pMap->mPointPoseConsistencyMutex.unlock();

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    Eigen::Matrix<double,4,4> matLambdast = Eigen::Matrix<double,4,4>::Identity();
    Eigen::Matrix<double,7,7> matLambdasim = Eigen::Matrix<double,7,7>::Identity();

    std::vector<scalei2j> scale_edges;
    scale_edges.reserve(LoopConnections.size()+ vpKFs.size()*2);
    // SET LOOP EDGES   
    for(map<KeyFrame *, set<KeyFrame *> >::iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnFrameId;
        set<KeyFrame*> &spConnections = mit->second;
        g2o::Sim3 Siw = vScw[nIDi];
        g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnFrameId;
            if((nIDi!=pCurKF->mnFrameId || nIDj!=pLoopKF->mnFrameId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            g2o::Sim3 Sjw = vScw[nIDj];
            g2o::Sim3 Sji = Sjw * Swi;

            scale_edges.push_back(scalei2j(nIDi, nIDj, Sji.scale()));

            ScaViSLAM::G2oEdgeScaleTrans* est = new ScaViSLAM::G2oEdgeScaleTrans();
            est->setVertex(1, optimizer[1].vertex(nIDj));
            est->setVertex(0, optimizer[1].vertex(nIDi));
            est->setMeasurement(Converter::toScaleTrans(Sji));
            est->information() = matLambdast;
            optimizer[1].addEdge(est);

            if(num_optimizer==3){
            ScaViSLAM::EdgeSim3* esim = new ScaViSLAM::EdgeSim3();
            esim->setVertex(1, optimizer[2].vertex(nIDj));
            esim->setVertex(0, optimizer[2].vertex(nIDi));
            esim->setMeasurement(Sji);
            esim->information() = matLambdasim;
            optimizer[2].addEdge(esim);
            }
            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // SET NORMAL EDGES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnFrameId;

        g2o::Sim3 Swi;
        if(NonCorrectedSim3.count(pKF))
            Swi = NonCorrectedSim3[pKF].inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnFrameId;

            g2o::Sim3 Sjw;

            if(NonCorrectedSim3.count(pParentKF))
                Sjw = NonCorrectedSim3[pParentKF];
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            scale_edges.push_back(scalei2j(nIDi, nIDj, Sji.scale()));

            ScaViSLAM::G2oEdgeScaleTrans* est = new ScaViSLAM::G2oEdgeScaleTrans();
            est->setVertex(1, optimizer[1].vertex(nIDj));
            est->setVertex(0, optimizer[1].vertex(nIDi));
            est->setMeasurement(Converter::toScaleTrans(Sji));
            est->information() = matLambdast;
            optimizer[1].addEdge(est);
            if(num_optimizer==3){
            ScaViSLAM::EdgeSim3* e = new ScaViSLAM::EdgeSim3();
            e->setVertex(1, optimizer[2].vertex(nIDj));
            e->setVertex(0, optimizer[2].vertex(nIDi));
            e->setMeasurement(Sji);
            e->information() = matLambdasim;
            optimizer[2].addEdge(e);
            }
        }

        // Loop edges
        set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnFrameId<pKF->mnFrameId)
            {
                g2o::Sim3 Slw;
                if(NonCorrectedSim3.count(pLKF))
                    Slw = NonCorrectedSim3[pLKF];
                else
                    Slw = vScw[pLKF->mnFrameId];

                g2o::Sim3 Sli = Slw * Swi;

                scale_edges.push_back(scalei2j(nIDi, pLKF->mnFrameId, Sli.scale()));

                ScaViSLAM::G2oEdgeScaleTrans* elst = new ScaViSLAM::G2oEdgeScaleTrans();
                elst->setVertex(1, optimizer[1].vertex(pLKF->mnFrameId));
                elst->setVertex(0, optimizer[1].vertex(nIDi));
                elst->setMeasurement(Converter::toScaleTrans(Sli));
                elst->information() = matLambdast;
                optimizer[1].addEdge(elst);

                if(num_optimizer ==3){
                ScaViSLAM::EdgeSim3* el = new ScaViSLAM::EdgeSim3();
                el->setVertex(1, optimizer[2].vertex(pLKF->mnFrameId));
                el->setVertex(0, optimizer[2].vertex(nIDi));
                el->setMeasurement(Sli);
                el->information() = matLambdasim;
                optimizer[2].addEdge(el);
                }
            }
        }

        // Covisibility graph edges
        vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnFrameId<pKF->mnFrameId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnFrameId,pKFn->mnFrameId),max(pKF->mnFrameId,pKFn->mnFrameId))))
                        continue;

                    g2o::Sim3 Snw;
                    if(NonCorrectedSim3.count(pKFn))
                        Snw = NonCorrectedSim3[pKFn];
                    else
                        Snw = vScw[pKFn->mnFrameId];

                    g2o::Sim3 Sni = Snw * Swi;

                    scale_edges.push_back(scalei2j(nIDi, pKFn->mnFrameId, Sni.scale()));

                    ScaViSLAM::G2oEdgeScaleTrans* enst = new ScaViSLAM::G2oEdgeScaleTrans();
                    enst->setVertex(1, optimizer[1].vertex(pKFn->mnFrameId));
                    enst->setVertex(0, optimizer[1].vertex(nIDi));
                    enst->setMeasurement(Converter::toScaleTrans(Sni));
                    enst->information() = matLambdast;
                    optimizer[1].addEdge(enst);
                    if(num_optimizer==3){
                    ScaViSLAM::EdgeSim3* en = new ScaViSLAM::EdgeSim3();
                    en->setVertex(1, optimizer[2].vertex(pKFn->mnFrameId));
                    en->setVertex(0, optimizer[2].vertex(nIDi));
                    en->setMeasurement(Sni);
                    en->information() = matLambdasim;
                    optimizer[2].addEdge(en);
                    }
                }
            }
        }
    }

    // OPTIMIZE
    ROS_INFO("opt20 in %s", __func__);
    /// optimize scales
    // (1) SVD method to solve for scales, Ax=0; x= [s_0^0, s_0^1; s_0^2; ..., s_0^{N-1}], where s_0^j is the scale of frame j,
    // the solution is the last column of V of SVD of A
    size_t num_constraints = scale_edges.size();
    int num_epoch = vpKFs.size();
    Eigen::MatrixXd sm1( num_constraints, num_epoch);
    sm1.setZero();
    for(size_t jack=0; jack< num_constraints; ++jack)
    {
        sm1(jack, kfid2varid[scale_edges[jack].i])= scale_edges[jack].si2j;
        sm1(jack, kfid2varid[scale_edges[jack].j])= -1;
    }

    JacobiSVD<Eigen::MatrixXd > svd(sm1, ComputeThinV);
//    Eigen::Matrix<double, Eigen::Dynamic, 1> S=svd.singularValues();
//    cout<<"singular values "<<S.transpose() <<endl;
//    if(S.coeff(0)*5e-4 > S.coeff(num_epoch-1))
//        cout<<"Warning possible unsable result by SVD"<<endl;

    Eigen::Matrix<double, Eigen::Dynamic, 1> allScales(num_epoch, 1);
    assert(svd.matrixV().rows() == num_epoch && svd.matrixV().cols() == num_epoch);
    allScales= svd.matrixV().block(0, num_epoch-1, num_epoch, 1);
    allScales= allScales/allScales[0];

    // update scale estimates due to SVD
    for(size_t i=0;i<vpKFs.size();++i)
    {
        KeyFrame* pKFi = vpKFs[i];
        const int nIDi = pKFi->mnFrameId;

//        ScaViSLAM::G2oVertexScale* vS = static_cast<ScaViSLAM::G2oVertexScale*>(optimizer[0].vertex(nIDi));
//        assert( kfid2varid[nIDi]==i);
//        vS->setEstimate( allScales[i]);

        ScaViSLAM::G2oVertexScaleTrans* vST = static_cast<ScaViSLAM::G2oVertexScaleTrans*>(optimizer[1].vertex(nIDi));
        Eigen::Vector4d stw2i =  vST->estimate();
        stw2i[0]= allScales[i];
        vST->setEstimate(stw2i);
    }

    // optimize scale with g2o, not necessary
/*    optimizer[0].initializeOptimization();
    optimizer[0].optimize(20);
    for(size_t i=0;i<vpKFs.size();++i)
    {
        KeyFrame* pKFi = vpKFs[i];
        const int nIDi = pKFi->mnFrameId;
        ScaViSLAM::G2oVertexScale* vS = static_cast<ScaViSLAM::G2oVertexScale*>(optimizer[0].vertex(nIDi));
        double sw2i =  vS->estimate();
        ScaViSLAM::G2oVertexScaleTrans* vST = static_cast<ScaViSLAM::G2oVertexScaleTrans*>(optimizer[1].vertex(nIDi));
        Eigen::Vector4d v4 = vST->estimate();
        v4[0]= sw2i;
        vST->setEstimate(v4);
    }*/

    optimizer[1].initializeOptimization();
    optimizer[1].optimize(10);

    if(num_optimizer ==3){
    // update scale and translation estimates
        for(size_t i=0;i<vpKFs.size();++i)
        {
            KeyFrame* pKFi = vpKFs[i];
            const int nIDi = pKFi->mnFrameId;

            ScaViSLAM::G2oVertexScaleTrans* vST = static_cast<ScaViSLAM::G2oVertexScaleTrans*>(optimizer[1].vertex(nIDi));
            Eigen::Vector4d stw2i =  vST->estimate();
            g2o::Sim3 CorrectedSiw(vST->Rw2i.unit_quaternion(), stw2i.tail<3>(), stw2i[0]);

            ScaViSLAM::VertexSim3Expmap* vSim3 = static_cast<ScaViSLAM::VertexSim3Expmap*>(optimizer[2].vertex(nIDi));
            vSim3->setEstimate(CorrectedSiw);
        }
        optimizer[2].initializeOptimization();
        optimizer[2].optimize(10);
    }

    pMap->mPointPoseConsistencyMutex.lock();
    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        const int nIDi = pKFi->mnFrameId;
        g2o::Sim3 CorrectedSiw;
        if(num_optimizer ==3){
            ScaViSLAM::VertexSim3Expmap* vSim3 = static_cast<ScaViSLAM::VertexSim3Expmap*>(optimizer[2].vertex(nIDi));
            CorrectedSiw=vSim3->estimate();
        }else{
            ScaViSLAM::G2oVertexScaleTrans* vST = static_cast<ScaViSLAM::G2oVertexScaleTrans*>(optimizer[1].vertex(nIDi));
            Eigen::Vector4d stw2i =  vST->estimate();
            CorrectedSiw= g2o::Sim3(vST->Rw2i.unit_quaternion(), stw2i.tail<3>(), stw2i[0]);
        }
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        //[R t/s;0 1]
        Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation()/CorrectedSiw.scale());
        pKFi->SetPose(Tiw);
    }
    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];
        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnFrameId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnFrameId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        pMP->SetWorldPos(eigCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
    pMap->mbFinishedLoopClosing =true;
    pMap->mPointPoseConsistencyMutex.unlock();
#endif
}
void Optimizer::OptimizeEssentialGraphSE3(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF, Sophus::SE3d &Tcurw,
                                       LoopClosing::KeyFrameAndSE3Pose &NonCorrectedSE3,
                                       LoopClosing::KeyFrameAndSE3Pose &CorrectedSE3,
                                       map<KeyFrame *, set<KeyFrame *> > &LoopConnections)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr= new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    unsigned int nMaxKFid = pMap->GetMaxKFid();


    vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d> > vTcw(nMaxKFid+1);
    vector<Sophus::SE3d,Eigen::aligned_allocator<Sophus::SE3d> > vCorrectedTwc(nMaxKFid+1);
    vector<ScaViSLAM::G2oVertexSE3*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;

    // SET KEYFRAME VERTICES
    pMap->mPointPoseConsistencyMutex.lock();
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        ScaViSLAM::G2oVertexSE3* VSE3 = new ScaViSLAM::G2oVertexSE3();

        int nIDi = pKF->mnFrameId;

        if(CorrectedSE3.count(pKF))
        {
            vTcw[nIDi] = CorrectedSE3[pKF];
            VSE3->setEstimate(CorrectedSE3[pKF]);
        }
        else
        {
            Sophus::SE3d Tiw=pKF->GetPose();
            vTcw[nIDi] = Tiw;
            VSE3->setEstimate(Tiw);
        }

        if(pKF==pLoopKF)
            VSE3->setFixed(true);

        VSE3->setId(nIDi);
        VSE3->setMarginalized(false);

        optimizer.addVertex(VSE3);

        vpVertices[nIDi]=VSE3;
    }
    pMap->mPointPoseConsistencyMutex.unlock();

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();

    // SET LOOP EDGES
    for(map<KeyFrame *, set<KeyFrame *> >::iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnFrameId;
        set<KeyFrame*> &spConnections = mit->second;
        Sophus::SE3d Tiw = vTcw[nIDi];
        Sophus::SE3d Twi = Tiw.inverse();

        for(set<KeyFrame*>::iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnFrameId;
            if((nIDi!=pCurKF->mnFrameId || nIDj!=pLoopKF->mnFrameId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            Sophus::SE3d Tjw = vTcw[nIDj];
            Sophus::SE3d Tji = Tjw * Twi;

            ScaViSLAM::G2oEdgeSE3* e = new ScaViSLAM::G2oEdgeSE3();
            e->setVertex(1, optimizer.vertex(nIDj));
            e->setVertex(0, optimizer.vertex(nIDi));
            e->setMeasurement(Tji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // SET NORMAL EDGES
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnFrameId;

        Sophus::SE3d Twi;
        if(NonCorrectedSE3.count(pKF))
            Twi = NonCorrectedSE3[pKF].inverse();
        else
            Twi = vTcw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnFrameId;

            Sophus::SE3d Tjw;

            if(NonCorrectedSE3.count(pParentKF))
                Tjw = NonCorrectedSE3[pParentKF];
            else
                Tjw = vTcw[nIDj];

            Sophus::SE3d Tji = Tjw * Twi;

            ScaViSLAM::G2oEdgeSE3* e = new ScaViSLAM::G2oEdgeSE3();
            e->setVertex(1, optimizer.vertex(nIDj));
            e->setVertex(0, optimizer.vertex(nIDi));
            e->setMeasurement(Tji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnFrameId<pKF->mnFrameId)
            {
                Sophus::SE3d Tlw;
                if(NonCorrectedSE3.count(pLKF))
                    Tlw = NonCorrectedSE3[pLKF];
                else
                    Tlw = vTcw[pLKF->mnFrameId];

                Sophus::SE3d Tli = Tlw * Twi;
                ScaViSLAM::G2oEdgeSE3* el = new ScaViSLAM::G2oEdgeSE3();
                el->setVertex(1, optimizer.vertex(pLKF->mnFrameId));
                el->setVertex(0, optimizer.vertex(nIDi));
                el->setMeasurement(Tli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnFrameId<pKF->mnFrameId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnFrameId,pKFn->mnFrameId),max(pKF->mnFrameId,pKFn->mnFrameId))))
                        continue;

                    Sophus::SE3d Tnw;
                    if(NonCorrectedSE3.count(pKFn))
                        Tnw = NonCorrectedSE3[pKFn];
                    else
                        Tnw = vTcw[pKFn->mnFrameId];

                    Sophus::SE3d Tni = Tnw * Twi;

                    ScaViSLAM::G2oEdgeSE3* en = new ScaViSLAM::G2oEdgeSE3();
                    en->setVertex(1, optimizer.vertex(pKFn->mnFrameId));
                    en->setVertex(0, optimizer.vertex(nIDi));
                    en->setMeasurement(Tni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // OPTIMIZE

    optimizer.initializeOptimization();
    optimizer.optimize(20);

    pMap->mPointPoseConsistencyMutex.lock();
    // SE3 Pose Recovering
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnFrameId;

        ScaViSLAM::G2oVertexSE3* VSE3 = static_cast<ScaViSLAM::G2oVertexSE3*>(optimizer.vertex(nIDi));
        Sophus::SE3d CorrectedTiw =  VSE3->estimate();
        vCorrectedTwc[nIDi]=CorrectedTiw.inverse();

        pKFi->SetPose(CorrectedTiw);
    }
    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnFrameId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnFrameId;
        }


        Sophus::SE3d Trw = vTcw[nIDr];
        Sophus::SE3d correctedTwr = vCorrectedTwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedTwr*Trw*eigP3Dw;

        pMP->SetWorldPos(eigCorrectedP3Dw);

        pMP->UpdateNormalAndDepth();
    }
    pMap->mbFinishedLoopClosing =true;
    pMap->mPointPoseConsistencyMutex.unlock();
}
int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, float th2)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    Eigen::Matrix3d K1 = pKF1->GetCalibrationMatrix();
    Eigen::Matrix3d K2 = pKF2->GetCalibrationMatrix();

    // Camera poses
    Eigen::Matrix3d R1w = pKF1->GetRotation();
    Eigen::Vector3d t1w = pKF1->GetTranslation();
    Eigen::Matrix3d R2w = pKF2->GetRotation();
    Eigen::Vector3d t2w = pKF2->GetTranslation();

    // SET SIMILARITY VERTEX
    ScaViSLAM::VertexSim3Expmap * vSim3 = new ScaViSLAM::VertexSim3Expmap();
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1(0,2);
    vSim3->_principle_point1[1] = K1(1,2);
    vSim3->_focal_length1[0] = K1(0,0);
    vSim3->_focal_length1[1] = K1(1,1);
    vSim3->_principle_point2[0] = K2(0,2);
    vSim3->_principle_point2[1] = K2(1,2);
    vSim3->_focal_length2[0] = K2(0,0);
    vSim3->_focal_length2[1] = K2(1,1);
    optimizer.addVertex(vSim3);

    // SET MAP POINT VERTICES
    const int N = vpMatches1.size();
    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<ScaViSLAM::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<ScaViSLAM::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<float> vSigmas12, vSigmas21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    Map* pMap = pKF1->GetMap();
    pMap->mPointPoseConsistencyMutex.lock();
    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        int id1 = 2*i+1;
        int id2 = 2*(i+1);

        int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3d P3D1w = pMP1->GetWorldPos();
                Eigen::Vector3d P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(P3D1c);
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3d P3D2w = pMP2->GetWorldPos();
                Eigen::Vector3d P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(P3D2c);
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // SET EDGE x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        cv::KeyPoint kpUn1 = pKF1->GetKeyPointUn(i);
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        ScaViSLAM::EdgeSim3ProjectXYZ* e12 = new ScaViSLAM::EdgeSim3ProjectXYZ();
        e12->setVertex(0, optimizer.vertex(id2));
        e12->setVertex(1, optimizer.vertex(0));
        e12->setMeasurement(obs1);
        float invSigmaSquare1 = pKF1->GetInvSigma2(kpUn1.octave);
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // SET EDGE x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        cv::KeyPoint kpUn2 = pKF2->GetKeyPointUn(i2);
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        ScaViSLAM::EdgeInverseSim3ProjectXYZ* e21 = new ScaViSLAM::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, optimizer.vertex(id1));
        e21->setVertex(1, optimizer.vertex(0));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->GetSigma2(kpUn2.octave);
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }
    pMap->mPointPoseConsistencyMutex.unlock();
    // Optimize

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        ScaViSLAM::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ScaViSLAM::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=NULL;
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=NULL;
            vpEdges21[i]=NULL;
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        ScaViSLAM::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ScaViSLAM::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=NULL;
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    ScaViSLAM::VertexSim3Expmap* vSim3_recov = static_cast<ScaViSLAM::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}
void Optimizer::setupG2o(ScaViSLAM::G2oCameraParameters * g2o_cam,
                        G2oCameraParameters*g2o_cam_right,
                        G2oIMUParameters * g2o_imu,
                        g2o::SparseOptimizer * optimizer)
{
    bool bUseIMUData= g2o_imu!=NULL;
    if(bUseIMUData)
    {   //block solver_6_3 has errors in eigen in optimizing stereo+IMU observations,
        // About choosing block solvers, g2o has some examples
        //in g2o/examples/ba_anchored_inverse_depth/ba_anchored_inverse_depth_demo.cpp,
        // schur trick case: BlockSolver_6_3 and marginalizing 3D point vertices
        // otherwise: BlockSolverX and no marginalization
        // in g2o/examples/ba/ba_demo.cpp, BlockSolver_6_3 and marginalizing 3D point vertices
        //in g2o/examples/icp/gicp_sba_demo.cpp, BlockSolverX are used together with marginalizing 3D point vertices
        // in g2o/examples/bal/bal_example.cpp, g2o::BlockSolver< g2o::BlockSolverTraits<9, 3> > is used with marginalization
        // About linear solvers, g2o provides LinearSolverDense, LinearSolverCSparse, LinearSolverCholmod, and LinearSolverPCG

        optimizer->setVerbose(false);
        g2o::BlockSolverX::LinearSolverType * linearSolver=
                new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * block_solver =
                new g2o::BlockSolverX(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* lm =
                new g2o::OptimizationAlgorithmLevenberg(block_solver);

        lm->setMaxTrialsAfterFailure(5);
        optimizer->setAlgorithm(lm);
    }else{
        optimizer->setVerbose(false);
        typename g2o::BlockSolver_6_3::LinearSolverType * linearSolver
                = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * block_solver
                = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg * lm
                = new g2o::OptimizationAlgorithmLevenberg(block_solver);
        lm->setMaxTrialsAfterFailure(5);
        optimizer->setAlgorithm(lm);
    }
    if (!optimizer->addParameter(g2o_cam))
    {
        assert(false);
    }
#ifndef MONO
    assert(g2o_cam_right!=NULL);

    if (!optimizer->addParameter(g2o_cam_right))
    {
        assert(false);
    }
#endif
    if(bUseIMUData){
        if (!optimizer->addParameter(g2o_imu))
        {
            assert(false);
        }
    }
}

G2oVertexSE3* Optimizer::addPoseToG2o(const SE3d & T_me_from_w,
                                     int pose_id,
                                     bool fixed,
                                     g2o::SparseOptimizer * optimizer,
                                     const Sophus::SE3d* first_estimate)
{
    G2oVertexSE3 * v_se3 = new G2oVertexSE3();
    v_se3->setId(pose_id);
    v_se3->setEstimate(T_me_from_w);
    v_se3->setFixed(fixed);
    if(first_estimate!=NULL)
        v_se3->setFirstEstimate(*first_estimate);
    optimizer->addVertex(v_se3);
    return v_se3;
}
G2oVertexSpeedBias* Optimizer::addSpeedBiasToG2o(const Matrix<double, 9,1> & vinw_bias,
                                                int sb_id,
                                                bool fixed,
                                                g2o::SparseOptimizer * optimizer,
                                                const Eigen::Matrix<double, 9,1> * first_estimate)
{
    G2oVertexSpeedBias * v_sb = new G2oVertexSpeedBias();
    v_sb->setId(sb_id);
    v_sb->setEstimate(vinw_bias);
    v_sb->setFixed(fixed);
    // v_sb->setMarginalized(true); // this line causes error
    if(first_estimate!=NULL)
        v_sb->setFirstEstimate(*first_estimate);
    optimizer->addVertex(v_sb);
    return v_sb;
}

G2oVertexPointXYZ* Optimizer::addPointToG2o( MapPoint* pPoint,
                                                   int g2o_point_id, bool fixed,
                                                   g2o::SparseOptimizer * optimizer)
{
    G2oVertexPointXYZ * v_point = new G2oVertexPointXYZ;
    v_point->setId(g2o_point_id);
    v_point->setFixed(fixed);
    v_point->setEstimate(pPoint->GetWorldPos());
    v_point->setMarginalized(true);//if true, Schur trick is used to marginalize this vertex during optimization
    if(pPoint->mbFixedLinearizationPoint)
        v_point->setFirstEstimate(pPoint->mWorldPos_first_estimate);
    optimizer->addVertex(v_point);
    return v_point;
}

G2oEdgeProjectXYZ2UV* Optimizer::addObsToG2o(const Vector2d & obs,
                                            const Matrix2d & Lambda,
                                            ScaViSLAM::G2oVertexPointXYZ* point_vertex,
                                            ScaViSLAM::G2oVertexSE3* pose_vertex,
                                            bool robustify,
                                            double huber_kernel_width,
                                            g2o::SparseOptimizer * optimizer, SE3d * pTs2c)
{
    G2oEdgeProjectXYZ2UV * e=NULL;
    if(pTs2c)
        e = new G2oExEdgeProjectXYZ2UV(*pTs2c);
    else
        e = new G2oEdgeProjectXYZ2UV();
    e->resize(2);

    assert(point_vertex->dimension()==3);
    e->vertices()[0] = point_vertex;

    assert(pose_vertex->dimension()==6);
    e->vertices()[1] = pose_vertex;

    e->setMeasurement(obs);
    e->information() = Lambda;

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(huber_kernel_width);

    e->resizeParameters(1);
    bool param_status = e->setParameterId(0, pTs2c==NULL ? 0:1);
    assert(param_status);
    optimizer->addEdge(e);
    return e;
}
//N.B. g2o refuses negative indices
// copy frames in temporal and spatial window and the previous and current frame to g2o
// return next unused possible vertex id
// i found that fix the first frame does not have much effect
size_t Optimizer::copyAllPosesToG2o(g2o::SparseOptimizer * optimizer, const std::vector<KeyFrame*> vpLocalKeyFrames,
                                   std::deque<Frame *> vpTemporalFrames, Frame * pCurrentFrame, Frame * pLastFrame,
                                   bool bUseIMUData)
{
    size_t max_vertex_id=0;
    size_t vertexid= 0;
    for (vector <KeyFrame*>::const_iterator  it_win = vpLocalKeyFrames.begin(), it_end_win=vpLocalKeyFrames.end();
         it_win!=it_end_win; ++it_win)
    {
        SE3d Tw2cref= (*it_win)->GetPose(); //transform from world to reference camera (i.e. left camera) frame
        vertexid= (*it_win)->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;
        if((*it_win)->mbFixedLinearizationPoint){
            SE3d Tw2cref_fe=(*it_win)->mTcw_first_estimate;
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false, optimizer, &Tw2cref_fe);
        }
        else
        {
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false, optimizer);
        }
    }
    for (deque<Frame*>::const_iterator  it_win = vpTemporalFrames.begin(), it_end_win=vpTemporalFrames.end();
         it_win!=it_end_win; ++it_win)
    {
        SE3d Tw2cref= (*it_win)->GetPose();  //transform from world to reference camera (i.e. left camera) frame
        vertexid= (*it_win)->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;

        if((*it_win)->mbFixedLinearizationPoint){
            SE3d Tw2cref_fe=(*it_win)->mTcw_first_estimate;
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid, false,  optimizer, &Tw2cref_fe);
            if(bUseIMUData)
            {
                (*it_win)->v_sb_=addSpeedBiasToG2o((*it_win)->speed_bias, vertexid+1, false,
                                                   optimizer, &(*it_win)->speed_bias_first_estimate);
            }
        }
        else
        {
            (*it_win)->v_kf_ = addPoseToG2o(Tw2cref, vertexid,  false, optimizer);
            if(bUseIMUData)
            {
                (*it_win)->v_sb_=addSpeedBiasToG2o((*it_win)->speed_bias, vertexid+1, false,
                                                   optimizer);
            }
        }
    }
    SE3d Tw2cref;
    // previous frame
#ifndef MONO
        assert( vpTemporalFrames.size()==0 || vpTemporalFrames.back()->mnId+1== pLastFrame->mnId);
        Tw2cref= pLastFrame->mTcw; //transform from world to reference camera (i.e. left camera) frame
        vertexid= pLastFrame->mnId*MAGIC2;
        if(max_vertex_id< vertexid)
            max_vertex_id=vertexid;
        assert(!pLastFrame->mbFixedLinearizationPoint);
        pLastFrame->v_kf_=addPoseToG2o(Tw2cref, vertexid, false, optimizer);
        if(bUseIMUData)
        {
            pLastFrame->v_sb_=addSpeedBiasToG2o(pLastFrame->speed_bias, vertexid+1, false,
                                                 optimizer);
        }
#endif
    //current frame
    Tw2cref= pCurrentFrame->mTcw; //transform from world to reference camera (i.e. left camera) frame
    vertexid= pCurrentFrame->mnId*MAGIC2;
    if(max_vertex_id< vertexid)
        max_vertex_id=vertexid;
    pCurrentFrame->v_kf_=addPoseToG2o(Tw2cref, vertexid, false, optimizer);
    if(bUseIMUData)
    {
        pCurrentFrame->v_sb_=addSpeedBiasToG2o(pCurrentFrame->speed_bias, vertexid+1, false,
                                                optimizer);
    }
    return max_vertex_id+2;
}

// For all map points in the local map, add them into the optimizer if they are observed by at least 2 keyframes
// return the number of outlier/bad observations made by the current frame
// we do not need to check pKF->isBad, because localMapper will not setbadflag for keyframes in double window
int Optimizer::LocalOptimize(vk::PinholeCamera * cam,
                            Map* pMap, const std::vector<KeyFrame*>& vpLocalKeyFrames,
                            std::vector<MapPoint*>& vpLocalMapPoints,
                            const std::deque<Frame*>& vpTemporalFrames, Frame* pCurrentFrame,
                            Frame *pLastFrame, ScaViSLAM::G2oIMUParameters* imu,
                             vk::PinholeCamera * right_cam, Sophus::SE3d * pTl2r)
{
    bool bUseIMUData = imu!=NULL;
    g2o::SparseOptimizer optimizer;
    ScaViSLAM::G2oCameraParameters  * g2o_cam
            = new G2oCameraParameters(Vector2d(cam->cx(), cam->cy()),
                                      Vector2d(cam->fx(), cam->fy()));
    g2o_cam->setId(0);
    ScaViSLAM::G2oCameraParameters  * g2o_cam_right=NULL;
#ifndef MONO
    g2o_cam_right
            = new G2oCameraParameters(Vector2d(right_cam->cx(), right_cam->cy()),
                                      Vector2d(right_cam->fx(), right_cam->fy()));
    g2o_cam_right->setId(1);
#endif
    G2oIMUParameters  * g2o_imu  = NULL;
    if(bUseIMUData)
    {
        g2o_imu  = new G2oIMUParameters(*imu);
        g2o_imu->setId(2);
    }
    setupG2o(g2o_cam, g2o_cam_right, g2o_imu, &optimizer);

    pMap->mPointPoseConsistencyMutex.lock();
    if(pMap->mbFinishedLoopClosing){
        pMap->mbFinishedLoopClosing = false;
        pMap->mPointPoseConsistencyMutex.unlock();
        return 0; // skip this optimization and do next optimization after states in the double window are updated
    }
    size_t nOffset=copyAllPosesToG2o(&optimizer, vpLocalKeyFrames, vpTemporalFrames, pCurrentFrame, pLastFrame, bUseIMUData);

    // Now go throug all the points and add observations. Add a fixed neighbour
    // Keyframe if it is not in the set of core kfs
    const double delta2 = 5.991;
    const double delta = sqrt(delta2);
    int nInitialCorrespondences=0; //how many map points are tracked in current frame
    list<EdgeContainerSE3d> edges;
    size_t n_mps = 0;
    size_t n_fix_kfs=0;

    list<KeyFrame*> neib_kfs;
    for(auto it_pt = vpLocalMapPoints.begin(), ite=vpLocalMapPoints.end(); it_pt!=ite; ++it_pt)
    {
        // Create point vertex
        MapPoint* pMP = *it_pt;
        if(pMP->isBad()) { (*it_pt)=NULL; continue;}
        std::map<KeyFrame*, size_t> obs_copy= pMP->GetObservations();
        if(obs_copy.size()<2) { (*it_pt)=NULL; continue;}
        G2oVertexPointXYZ*v_pt= addPointToG2o(pMP, pMP->mnId + nOffset, false, &optimizer);
        pMP->v_pt_= v_pt;
        ++n_mps;
        // Add edges
        std::map<KeyFrame*, size_t>::const_iterator mit_obs=obs_copy.begin();
        while(mit_obs!=obs_copy.end())
        {
            KeyFrame * pKF= mit_obs->first;
            if(pKF->v_kf_ == NULL)
            {
                // frame does not have a vertex yet -> it belongs to the neib kfs and
                // is fixed. create one:
                ++n_fix_kfs;
                SE3d Tw2cref= pKF->GetPose(); //transform from world to reference camera (i.e. left camera) frame
                size_t vertexid= pKF->mnId*MAGIC2;

                if( pKF->mbFixedLinearizationPoint){
                    SE3d Tw2cref_fe= pKF->mTcw_first_estimate;
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer, &Tw2cref_fe);
                }
                else
                {
                    pKF->v_kf_=addPoseToG2o(Tw2cref, vertexid, true, &optimizer);
                }
                neib_kfs.push_back(pKF);
            }
            // create edge
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pKF->mvKeysUn[mit_obs->second];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pKF->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                     v_pt, pKF->v_kf_, true, delta, &optimizer);
            edges.push_back(EdgeContainerSE3d(porter, pKF, mit_obs->second));

#ifndef MONO
            // set right edge
            kpUn = pKF->mvRightKeysUn[mit_obs->second];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, v_pt, pKF->v_kf_, true, delta, &optimizer, pTl2r );
            edges.push_back(EdgeContainerSE3d(porter, pKF,  mit_obs->second));
#endif
            ++mit_obs;
        }
    }
    pMap->mPointPoseConsistencyMutex.unlock();

    // add constraint by IMU observations for frames in the temporal window and the current frame
    if(bUseIMUData){
        for (std::deque<Frame*>::const_iterator  it_win = vpTemporalFrames.begin(), it_end_win= vpTemporalFrames.end();
             it_win!=it_end_win; ++it_win)
        {
            G2oEdgeIMUConstraint * e = new G2oEdgeIMUConstraint();
            e->setParameterId(0,2);
            e->resize(4);
            if((*it_win)->next_frame ==NULL)//must be the last frame in the temporal window
            {
                if(it_win != vpTemporalFrames.end()-1)// some consecutive frames may have no inertial observations in between
                {
                    delete e;
                    continue;
                }
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
#ifdef MONO
                e->vertices()[2]
                        = pCurrentFrame->v_kf_;
                e->vertices()[3]
                        = pCurrentFrame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= pCurrentFrame->mTimeStamp;
                assert(pCurrentFrame->imu_observ.size());
                e->setMeasurement(pCurrentFrame->imu_observ);
#else
                e->vertices()[2]
                        = pLastFrame->v_kf_;
                e->vertices()[3]
                        = pLastFrame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= pLastFrame->mTimeStamp;
                assert(pLastFrame->imu_observ.size());
                e->setMeasurement(pLastFrame->imu_observ);
#endif

            }
            else{
                e->vertices()[0]
                        = (*it_win)->v_kf_;
                e->vertices()[1]
                        = (*it_win)->v_sb_;
                e->vertices()[2]
                        = (*it_win)->next_frame->v_kf_;
                e->vertices()[3]
                        = (*it_win)->next_frame->v_sb_;
                e->time_frames[0]= (*it_win)->mTimeStamp;
                e->time_frames[1]= (*it_win)->next_frame->mTimeStamp;
#ifdef SLAM_DEBUG_OUTPUT
                if((*it_win)->next_frame->isKeyFrame())
                {
                    KeyFrame * larry= (KeyFrame*)((*it_win)->next_frame);
                    if(larry->isBad()){
                        SLAM_DEBUG_STREAM("Bad keyframe in temporal window mnId,N, mbNotErase, keys size:"
                                          <<larry->mnId<<" "<<larry->N<<" "<<larry->GetNotErase()<<" "<< larry->mvKeysUn.size());

//                        assert(false);
                    }
                }
#endif
                e->setMeasurement((*it_win)->next_frame->imu_observ);
            }
            e->calcAndSetInformation(*g2o_imu);
            optimizer.addEdge(e);
        }
#ifndef MONO
        // the previous frame and current frame
        G2oEdgeIMUConstraint * e = new G2oEdgeIMUConstraint();
        e->setParameterId(0,2);
        e->resize(4);
        e->vertices()[0]
                = pLastFrame->v_kf_;
        e->vertices()[1]
                = pLastFrame->v_sb_;
        e->vertices()[2]
                = pCurrentFrame->v_kf_;
        e->vertices()[3]
                = pCurrentFrame->v_sb_;
        e->time_frames[0]= pLastFrame->mTimeStamp;
        e->time_frames[1]= pCurrentFrame->mTimeStamp;
        e->setMeasurement(pCurrentFrame->imu_observ);
        e->calcAndSetInformation(*g2o_imu);
        optimizer.addEdge(e);
#endif
    }

    // add points' observations for each non keyframe in the temporal window and the previous and current frame
    // add observations of frames in the temporal window
    for(deque<Frame*>::const_iterator qIt= vpTemporalFrames.begin(), qItEnd= vpTemporalFrames.end();
        qIt!=qItEnd; ++qIt)
    {
        if((*qIt)->isKeyFrame())
        {
            continue; //keyframe observations are already added for mappoints
        }
        size_t jim=0;
        for(vector<MapPoint*>::const_iterator itMP=(*qIt)->mvpMapPoints.begin(), itEndMP=(*qIt)->mvpMapPoints.end();
            itMP!=itEndMP; ++itMP, ++jim)
        {
            MapPoint* pMP = *itMP;
            if(pMP && pMP->v_pt_!=NULL)    // if valid pointer, and has been added to optimizer
            {
                //SET left EDGE
                Eigen::Matrix<double,2,1> obs;
                cv::KeyPoint kpUn = (*qIt)->mvKeysUn[jim];
                obs << kpUn.pt.x, kpUn.pt.y;
                const float invSigma2 = (*qIt)->GetInvSigma2(kpUn.octave);
                Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
                G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                         pMP->v_pt_, (*qIt)->v_kf_, true, delta, &optimizer);

                edges.push_back(EdgeContainerSE3d(porter, *qIt, jim));
#ifndef MONO
                // set right edge
                kpUn = (*qIt)->mvRightKeysUn[jim];
                obs << kpUn.pt.x, kpUn.pt.y;
                assert(kpUn.octave==0);
                porter=addObsToG2o(obs, infoMat,
                                   pMP->v_pt_, (*qIt)->v_kf_, true, delta, &optimizer, pTl2r );
                edges.push_back(EdgeContainerSE3d(porter, *qIt, jim));
#endif
            }
        }
    }
    size_t jim=0;
#ifndef MONO
    for(vector<MapPoint*>::const_iterator itMP=pLastFrame->mvpMapPoints.begin(), itEndMP=pLastFrame->mvpMapPoints.end();
        itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && pMP->v_pt_!=NULL)
        {
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pLastFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pLastFrame->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;
            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat,
                                                     pMP->v_pt_, pLastFrame->v_kf_, true, delta, &optimizer);
            edges.push_back(EdgeContainerSE3d(porter, pLastFrame, jim));

            // set right edge
            kpUn = pLastFrame->mvRightKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, pMP->v_pt_, pLastFrame->v_kf_, true, delta, &optimizer, pTl2r );
            edges.push_back(EdgeContainerSE3d(porter, pLastFrame, jim));

        }
    }
#endif
    jim=0;
    for(vector<MapPoint*>::const_iterator itMP=pCurrentFrame->mvpMapPoints.begin(),
        itEndMP=pCurrentFrame->mvpMapPoints.end(); itMP!=itEndMP; ++itMP, ++jim)
    {
        MapPoint* pMP = *itMP;
        if(pMP && pMP->v_pt_!=NULL)
        {
            //SET left EDGE
            Eigen::Matrix<double,2,1> obs;
            cv::KeyPoint kpUn = pCurrentFrame->mvKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            const float invSigma2 = pCurrentFrame->GetInvSigma2(kpUn.octave);
            Eigen::Matrix2d infoMat=Eigen::Matrix2d::Identity()*invSigma2;

            G2oEdgeProjectXYZ2UV* porter=addObsToG2o(obs, infoMat, pMP->v_pt_,
                                                     pCurrentFrame->v_kf_, true, delta, &optimizer );
            edges.push_back(EdgeContainerSE3d(porter, pCurrentFrame, jim));

#ifndef MONO
            // set right edge
            kpUn = pCurrentFrame->mvRightKeysUn[jim];
            obs << kpUn.pt.x, kpUn.pt.y;
            assert(kpUn.octave==0);
            porter=addObsToG2o(obs, infoMat, pMP->v_pt_, pCurrentFrame->v_kf_, true, delta, &optimizer, pTl2r );
            edges.push_back(EdgeContainerSE3d(porter, pCurrentFrame, jim));
#endif
            pCurrentFrame->mvbOutlier[jim] = false;
            ++nInitialCorrespondences;
        }
    }
    if(edges.size()==0)
    {
        cout<<"Graph with no edges."<<endl;
        return 0;
    }
    SLAM_DEBUG_STREAM("opt5 structBA in "<< __func__);
    optimizer.initializeOptimization();

    //    cout << "Performing structure-only BA:" << endl;
    g2o::StructureOnlySolver<3> structure_only_ba;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
        g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
        if (v->dimension() == 3)
            points.push_back(v);
    }
    structure_only_ba.calc(points, 5);
#ifndef MONO
// remove the right observations, because the extrinsic calibration error often causes drift
    for(auto it_edge= edges.begin(), ite_edge= edges.end(); it_edge!=ite_edge;)
    {
        if(dynamic_cast<G2oExEdgeProjectXYZ2UV*>(it_edge->edge))
            it_edge= edges.erase(it_edge);
        else
            ++it_edge;
    }

    for(auto eg_it=optimizer.edges().begin(), eg_end= optimizer.edges().end(); eg_it!=eg_end; )
    {
        if(dynamic_cast<G2oExEdgeProjectXYZ2UV*>(*eg_it))
        {
            eg_it= optimizer.edges().erase(eg_it);
        }
        else
            ++eg_it;
    }
#endif
    SLAM_DEBUG_STREAM("opt5 joint BA in "<<__func__);
    optimizer.optimize(5);
    int nBad=0;
    /// Emprically the following outlier removal process does not contribute much
#if 0
    // Check inlier observations, draw inspiration from LocalBundleAdjustment and PoseOptimization
    for(list<EdgeContainerSE3d>::iterator it = edges.begin(); it != edges.end();)
    {
        G2oEdgeProjectXYZ2UV* e1 = it->edge;
//N.B. right observations are removed for stereo odometry
        if(e1->chi2()>delta2)
        {
            Frame* pFi = it->frame;
            MapPoint* pMP= pFi->GetMapPoint(it->id_);
            if(pMP==NULL || (pMP->isBad())){
                ++it;
                continue;
            }
            if(pFi->isKeyFrame())
            {
                pFi->EraseMapPointMatch(it->id_);
                pMP->EraseObservation((KeyFrame*)pFi);
            }
            else{
                pFi->EraseMapPointMatch(it->id_);
                if(pFi==pCurrentFrame){
                    pFi->mvbOutlier[it->id_]=true;
                    ++nBad;
                }
            }
            optimizer.removeEdge(e1);
            it =edges.erase(it);
        }
        else
            ++it;
    }

    // Optimize again without the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(3);
#endif
    // Check inlier observations, this section draws inspiration from LocalBundleAdjustment and PoseOptimization by Raul
    for(list<EdgeContainerSE3d>::iterator it = edges.begin(); it != edges.end(); )
    {
        G2oEdgeProjectXYZ2UV* e1 = it->edge;
#if 0 //right observations are removed
        ++it;
        G2oExEdgeProjectXYZ2UV* e2 = (G2oExEdgeProjectXYZ2UV*)it->edge;
        if(e1->chi2()>delta2 || e2->chi2()>delta2)// || !e->isDepthPositive())
#else
        if(e1->chi2()>delta2)
#endif
        {
            Frame* pFi = it->frame;
            MapPoint* pMP= pFi->GetMapPoint(it->id_);
            if(pMP==NULL || (pMP->isBad()))
            {
                ++it;
                continue;
            }
            if(pFi->isKeyFrame())
            {
                pFi->EraseMapPointMatch(it->id_);
                pMP->EraseObservation((KeyFrame*)pFi);
            }
            else{
                pFi->EraseMapPointMatch(it->id_);
                if(pFi==pCurrentFrame)
                    ++nBad;
            }
        }
        ++it;
    }
    //update current frame
    pCurrentFrame->SetPose(pCurrentFrame->v_kf_->estimate());
    pCurrentFrame->v_kf_ = NULL;
#ifndef MONO
    pLastFrame->SetPose(pLastFrame->v_kf_->estimate());
    pLastFrame->v_kf_ = NULL;
#endif
    if(bUseIMUData){
        pCurrentFrame->speed_bias = pCurrentFrame->v_sb_->estimate();
        pCurrentFrame->v_sb_=NULL;
#ifndef MONO
        pLastFrame->speed_bias = pLastFrame->v_sb_->estimate();
        pLastFrame->v_sb_=NULL;
#endif
    }

    pMap->mPointPoseConsistencyMutex.lock();
    if(pMap->mbFinishedLoopClosing){
        pMap->mbFinishedLoopClosing = false;
        // remove these links
        for(deque<Frame*>::const_iterator it = vpTemporalFrames.begin(); it != vpTemporalFrames.end(); ++it)
        {
            assert((*it)->v_kf_!=NULL);

            (*it)->v_kf_ = NULL;
            if(bUseIMUData){
            assert((*it)->v_sb_!=NULL);

            (*it)->v_sb_=NULL;
            }
        }
        for(vector<KeyFrame*>::const_iterator it = vpLocalKeyFrames.begin(); it != vpLocalKeyFrames.end(); ++it)
        {
            (*it)->v_kf_ = NULL;
            assert((*it)->v_sb_==NULL);
        }
        for(list<KeyFrame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
            (*it)->v_kf_ = NULL;
        // Update Mappoints
        for(vector<MapPoint*>::iterator it = vpLocalMapPoints.begin(); it != vpLocalMapPoints.end(); ++it)
        {
            if((*it)==NULL) continue;
            (*it)->v_pt_ = NULL;
        }
        pMap->mPointPoseConsistencyMutex.unlock();
        return nBad;
    }
    // Update Keyframes
    for(deque<Frame*>::const_iterator it = vpTemporalFrames.begin(); it != vpTemporalFrames.end(); ++it)
    {
        assert((*it)->v_kf_!=NULL);
        (*it)->SetPose((*it)->v_kf_->estimate());
        (*it)->v_kf_ = NULL;
        if(bUseIMUData){
        assert((*it)->v_sb_!=NULL);
        (*it)->speed_bias = (*it)->v_sb_->estimate();
        (*it)->v_sb_=NULL;
        }
    }
    for(vector<KeyFrame*>::const_iterator it = vpLocalKeyFrames.begin(); it != vpLocalKeyFrames.end(); ++it)
    {
        (*it)->SetPose((*it)->v_kf_->estimate());
        (*it)->v_kf_ = NULL;
        assert((*it)->v_sb_==NULL);
    }
    for(list<KeyFrame*>::iterator it = neib_kfs.begin(); it != neib_kfs.end(); ++it)
        (*it)->v_kf_ = NULL;
    // Update Mappoints
    for(vector<MapPoint*>::iterator it = vpLocalMapPoints.begin(); it != vpLocalMapPoints.end(); ++it)
    {
        if((*it)==NULL) continue;
        (*it)->SetWorldPos((*it)->v_pt_->estimate());
        (*it)->v_pt_ = NULL;
    }

    pMap->mPointPoseConsistencyMutex.unlock();

    return nBad;
    //N.B. g2o will delete _algorithm, _parameters, EdgeSet and VertexIDMap upon destruction of SparseOptimizer
}

} //namespace ORB_SLAM
