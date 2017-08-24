/**
* This file is part of ORB-SLAM.
* It is based on the file orb.cpp from the OpenCV library (see BSD license below)
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

#include "ORBmatcher.h"

#include<limits.h>

//#include<ros/ros.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include "ORBextractor.h" //for findMatchDirect()
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>


using namespace std;

namespace ORB_SLAM
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;


ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;    

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        vector<size_t> vNearIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.GetScaleFactor(nPredictedLevel),nPredictedLevel-1,nPredictedLevel);

        if(vNearIndices.empty())
            continue;

        cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=INT_MAX;
        int bestLevel= -1;
        int bestDist2=INT_MAX;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::iterator vit=vNearIndices.begin(), vend=vNearIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                continue;

            cv::Mat d=F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;
//            ++pMP->mnObservationsInDoubleWindow; //Huai: count in the observation of current frame
            nmatches++;
        }
    }

    return nmatches;
}
/// Find a match by finding feature matches in a small neighborhood of px_cur
/// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
bool ORBmatcher::findMatchDirect(const cv::KeyPoint & pt, const cv::Mat &descriptor,
        const ORBextractor* pORBextractor, cv::KeyPoint & pt_cur, cv::Mat & descriptor_kp)
{ 
    int search_level_=pt.octave;// TODO: extend to lower and upper one level

    cv::Point2f & px_cur=pt_cur.pt;
    cv::Point2f px_scaled(px_cur*(1/pORBextractor->GetScaleFactor(search_level_)));

    const int nBorder=15;
    int minX= (int)std::max(0.f, round(px_scaled.x)-nBorder);
    int maxX=(int)std::min(pORBextractor->mvImagePyramid[search_level_].cols, (int)(round(px_scaled.x)+nBorder+1));
    int minY= (int)std::max(0.f, round(px_scaled.y)-nBorder);
    int maxY=(int)std::min(pORBextractor->mvImagePyramid[search_level_].rows, (int)(round(px_scaled.y)+nBorder+1));
    cv::Mat cellImage = pORBextractor->mvImagePyramid[search_level_].rowRange(minY,maxY).colRange(minX,maxX);

    vector<cv::KeyPoint> vCellKeyPoints;
    vCellKeyPoints.reserve(10);
    cv::FAST(cellImage,vCellKeyPoints,7,true);
    if(vCellKeyPoints.size()==0)
        return false;
    for(vector<cv::KeyPoint>::iterator it=vCellKeyPoints.begin(),
        ite=vCellKeyPoints.end(); it!=ite; ++it)
    {
        it->pt.x+= minX;
        it->pt.y+= minY;
    }
    computeOrientation(pORBextractor->mvImagePyramid[search_level_],
                       vCellKeyPoints, pORBextractor->umax);
    const cv::Mat& workingMat = pORBextractor->mvBlurredImagePyramid[search_level_];
    cv::Mat descriptor_cur= cv::Mat(vCellKeyPoints.size(), 32, CV_8U);
    computeDescriptors(workingMat, vCellKeyPoints, descriptor_cur, pORBextractor->pattern);

    int bestDist = INT_MAX;
    int bestDist2 = INT_MAX;
    int bestIdx2 = -1;
    int i2=0;
    for(vector<cv::KeyPoint>::iterator vit=vCellKeyPoints.begin(),
        vend=vCellKeyPoints.end(); vit!=vend; ++vit, ++i2)
    {
        cv::Mat d2 = descriptor_cur.row(i2);
        int dist = DescriptorDistance(descriptor,d2);
        if(dist<bestDist)
        {
            bestDist2=bestDist;
            bestDist=dist;
            bestIdx2=i2;
        } else if(dist<bestDist2)
        {
            bestDist2=dist;
        }
    }

    if(bestDist<=TH_LOW || (bestDist<=bestDist2*mfNNratio && bestDist<=TH_HIGH ))
    {
        pt_cur = vCellKeyPoints[bestIdx2];
        pt_cur.octave= pt.octave;
        pt_cur.pt*= pORBextractor->GetScaleFactor(search_level_);
        descriptor_cur.row(bestIdx2).copyTo(descriptor_kp);
        return true;
    }
    else
        return false;
}
// assume left and right image already have stereo matches, to accept a match,
// we need to match a map point to both left and right image
int ORBmatcher::SearchByProjectionStereo(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        vector<size_t> vNearIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.GetScaleFactor(nPredictedLevel),nPredictedLevel-1,nPredictedLevel);

        if(vNearIndices.empty())
            continue;

        cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=INT_MAX;
        int bestLevel= -1;
        int bestDist2=INT_MAX;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints for the left image
        for(vector<size_t>::iterator vit=vNearIndices.begin(), vend=vNearIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                continue;

            cv::Mat d=F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }
        int bestRightDist=INT_MAX;
        int bestRightLevel= -1;
        int bestRightDist2=INT_MAX;
        int bestRightLevel2 = -1;
        int bestRightIdx =-1 ;

        // Get best and second matches with near keypoints for the right image,
        // here we assume features in the right image is already matched to the left image
        for(vector<size_t>::iterator vit=vNearIndices.begin(), vend=vNearIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;
            if(F.mvpMapPoints[idx])
                continue;

            cv::Mat d=F.mRightDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestRightDist)
            {
                bestRightDist2=bestRightDist;
                bestRightDist=dist;
                bestRightLevel2 = bestRightLevel;
                bestRightLevel = F.mvKeysUn[idx].octave;
                bestRightIdx=idx;
            }
            else if(dist<bestRightDist2)
            {
                bestRightLevel2 = F.mvKeysUn[idx].octave;
                bestRightDist2=dist;
            }
        }
        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestIdx==bestRightIdx && (bestDist<=TH_HIGH || bestRightDist<=TH_HIGH))
        {// if bestIdx==bestRightIdx, loose the requirements
            if((bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2) &&
                    (bestRightLevel==bestRightLevel2 && bestRightDist>mfNNratio*bestRightDist2))
                continue;
            assert(F.mvpMapPoints[bestIdx]==NULL);
            F.mvpMapPoints[bestIdx]=pMP;
//            ++pMP->mnObservationsInDoubleWindow; //Huai: count in the observation of current frame
            nmatches++;
        }
    }
    return nmatches;
}
float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}


bool CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,
                           const Eigen::Matrix3d &F12,const Frame* pF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12(0,0)+kp1.pt.y*F12(1,0)+F12(2,0);
    const float b = kp1.pt.x*F12(0,1)+kp1.pt.y*F12(1,1)+F12(2,1);
    const float c = kp1.pt.x*F12(0,2)+kp1.pt.y*F12(1,2)+F12(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pF2->GetSigma2(kp2.octave);
}

int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<MapPoint*>(F.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));

    DBoW2::FeatureVector vFeatVecKF = pKF->GetFeatureVector();

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            vector<unsigned int> vIndicesKF = KFit->second;
            vector<unsigned int> vIndicesF = Fit->second;

            for(size_t iKF=0, iendKF=vIndicesKF.size(); iKF<iendKF; iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;

                cv::Mat dKF= pKF->GetDescriptor(realIdxKF);

                int bestDist1=INT_MAX;
                int bestIdxF =-1 ;
                int bestDist2=INT_MAX;

                for(size_t iF=0, iendF=vIndicesF.size(); iF<iendF; iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF])
                        continue;

                    cv::Mat dF = F.mDescriptors.row(realIdxF).clone();

                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<=TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF]=pMP;

                        cv::KeyPoint kp = pKF->GetKeyPointUn(realIdxKF);

                        if(mbCheckOrientation)
                        {
                            float rot = kp.angle-F.mvKeysUn[bestIdxF].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }
            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=NULL;
                nmatches--;
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame* pKF, g2o::Sim3 Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const float fx = pKF->cam_.fx();
    const float fy = pKF->cam_.fy();
    const float cx = pKF->cam_.cx();
    const float cy = pKF->cam_.cy();

    const int nMaxLevel = pKF->GetScaleLevels()-1;
    vector<float> vfScaleFactors = pKF->GetScaleFactors();

    // Decompose Scw    
    const float scw = Scw.scale();
    Eigen::Matrix3d Rcw =Scw.rotation().toRotationMatrix();
    Eigen::Vector3d tcw = Scw.translation()/scw;
    Eigen::Vector3d Ow = -Rcw.transpose()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(NULL);

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        Eigen::Vector3d p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        Eigen::Vector3d p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc(2);
        const float x = p3Dc(0)*invz;
        const float y = p3Dc(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3d PO = p3Dw-Ow;
        const float dist = PO.norm();

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3d Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        // Compute predicted scale level
        const float ratio = dist/minDistance;

        vector<float>::iterator it = lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
        const int nPredictedLevel = min(static_cast<int>(it-vfScaleFactors.begin()),nMaxLevel);

        // Search in a radius
        const float radius = th*pKF->GetScaleFactor(nPredictedLevel);

        vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int kpLevel= pKF->GetKeyPointScaleLevel(idx);

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            cv::Mat dKF = pKF->GetDescriptor(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }

        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int ORBmatcher::WindowSearch(Frame &F1, Frame &F2, int windowSize, vector<MapPoint *> &vpMapPointMatches2, int minScaleLevel, int maxScaleLevel)
{
    int nmatches=0;
    vpMapPointMatches2 = vector<MapPoint*>(F2.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
    vector<int> vnMatches21 = vector<int>(F2.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const bool bMinLevel = minScaleLevel>0;
    const bool bMaxLevel= maxScaleLevel<INT_MAX;

    for(size_t i1=0, iend1=F1.mvpMapPoints.size(); i1<iend1; i1++)
    {
        MapPoint* pMP1 = F1.mvpMapPoints[i1];

        if(!pMP1)
            continue;
        if(pMP1->isBad())
            continue;

        const cv::KeyPoint &kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;

        if(bMinLevel)
            if(level1<minScaleLevel)
                continue;

        if(bMaxLevel)
            if(level1>maxScaleLevel)
                continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(kp1.pt.x,kp1.pt.y, windowSize, level1, level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
        {
            size_t i2 = *vit;

            if(vpMapPointMatches2[i2])
                continue;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            } else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=bestDist2*mfNNratio && bestDist<=TH_HIGH)
        {
            vpMapPointMatches2[bestIdx2]=pMP1;
            vnMatches21[bestIdx2]=i1;
            nmatches++;

            float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
            if(rot<0.0)
                rot+=360.0f;
            int bin = round(rot*factor);
            if(bin==HISTO_LENGTH)
                bin=0;
            assert(bin>=0 && bin<HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    vpMapPointMatches2[rotHist[i][j]]=NULL;
                    vnMatches21[rotHist[i][j]]=-1;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

// for each mappoint in F1.mvpMapPoints, search for keypoint matches in F2, and add them to F2.mvpMapPoints
int ORBmatcher::SearchByProjection(Frame &F1, Frame &F2, int windowSize, vector<MapPoint *> &vpMapPointMatches2)
{
    vpMapPointMatches2 = F2.mvpMapPoints;
    set<MapPoint*> spMapPointsAlreadyFound(vpMapPointMatches2.begin(),vpMapPointMatches2.end());

    int nmatches = 0;

    const Eigen::Matrix3d Rc2w = F2.mTcw.rotationMatrix();
    const Eigen::Vector3d tc2w = F2.mTcw.translation();

    for(size_t i1=0, iend1=F1.mvpMapPoints.size(); i1<iend1; i1++)
    {
        MapPoint* pMP1 = F1.mvpMapPoints[i1];

        if(!pMP1)
            continue;
        if(pMP1->isBad() || spMapPointsAlreadyFound.count(pMP1))
            continue;

        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;

        Eigen::Vector3d x3Dw = pMP1->GetWorldPos();
        Eigen::Vector3d x3Dc2 = Rc2w*x3Dw+tc2w;

        const float xc2 = x3Dc2(0);
        const float yc2 = x3Dc2(1);
        const float invzc2 = 1.0/x3Dc2(2);

        float u2 = F2.cam_.fx()*xc2*invzc2+F2.cam_.cx();
        float v2 = F2.cam_.fy()*yc2*invzc2+F2.cam_.cy();

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(u2,v2, windowSize, level1, level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;


        for(vector<size_t>::iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
        {
            size_t i2 = *vit;

            if(vpMapPointMatches2[i2])
                continue;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            } else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(static_cast<float>(bestDist)<=static_cast<float>(bestDist2)*mfNNratio && bestDist<=TH_HIGH)
        {
            vpMapPointMatches2[bestIdx2]=pMP1;
            nmatches++;
        }

    }

    return nmatches;
}



int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2,
                                        vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

// match features in KF1 to map points in KF2
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    vector<cv::KeyPoint> vKeysUn1 = pKF1->GetKeyPointsUn();
    DBoW2::FeatureVector vFeatVec1 = pKF1->GetFeatureVector();
    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    cv::Mat Descriptors1 = pKF1->GetDescriptors();

    vector<cv::KeyPoint> vKeysUn2 = pKF2->GetKeyPointsUn();
    DBoW2::FeatureVector vFeatVec2 = pKF2->GetFeatureVector();
    vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    cv::Mat Descriptors2 = pKF2->GetDescriptors();

    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
        {
            if(f1it->first == f2it->first)
            {
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    size_t idx1 = f1it->second[i1];

                    MapPoint* pMP1 = vpMapPoints1[idx1];
                    if(!pMP1)
                        continue;
                    if(pMP1->isBad())
                        continue;

                    cv::Mat d1 = Descriptors1.row(idx1);

                    int bestDist1=INT_MAX;
                    int bestIdx2 =-1 ;
                    int bestDist2=INT_MAX;

                    for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                    {
                        size_t idx2 = f2it->second[i2];

                        MapPoint* pMP2 = vpMapPoints2[idx2];

                        if(vbMatched2[idx2] || !pMP2)
                            continue;

                        if(pMP2->isBad())
                            continue;

                        cv::Mat d2 = Descriptors2.row(idx2);

                        int dist = DescriptorDistance(d1,d2);

                        if(dist<bestDist1)
                        {
                            bestDist2=bestDist1;
                            bestDist1=dist;
                            bestIdx2=idx2;
                        }
                        else if(dist<bestDist2)
                        {
                            bestDist2=dist;
                        }
                    }

                    if(bestDist1<TH_LOW)
                    {
                        if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                        {
                            vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2]=true;

                            if(mbCheckOrientation)
                            {
                                float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LENGTH)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LENGTH);
                                rotHist[bin].push_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first)
            {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else
            {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=NULL;
                //vnMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    return nmatches;
}

// stereo matching between left (1) and right (2) frame
// input: assume left image keys and keysun are computed,
// output: right image keys are to be computed, and vnMatches
int ORBmatcher::SearchForStereoMatching(Frame *pF, Frame *pRightF, const Sophus::SE3d & Tl2r, vector<int> &vnMatches12)
{/*
    const vk::PinholeCamera right_cam= pRightF->cam_;
    Eigen::Matrix3d Flr= pF->ComputeFlr(pRightF, Tl2r);

    vector<cv::Point2f> vfPoints[3];//vfPoints[0-1] for forward searching, vfPoints[2-3] for backward searching,
    vector<uchar> status[2]; //status[0] forward searching indicator, status[1] backward searching indicator
    vector<float> err;
// compute prior feature position
    Sophus::SE3d Trw=Tl2r*pF->mTcw;
    std::vector<MapPoint*> &vpMapPoints= pF->mvpMapPoints;
    size_t sz=pF->mvKeysUn.size();
    vfPoints[0].reserve(sz);
    vfPoints[1].reserve(sz);

    float mean_depth=pF->ComputeSceneMedianDepth();
    assert(sz==vpMapPoints.size());
        for(size_t i=0; i<sz; ++i)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                Eigen::Vector3d x3Dw = pMP->GetWorldPos();
                Eigen::Vector2f px_cur=right_cam.world2cam(Trw*x3Dw).cast<float>();
                if(right_cam.isInFrame(Eigen::Vector2i(floor(px_cur[0]), floor(px_cur[1])))){
                    vfPoints[0].push_back(pF->mvKeys[i].pt); //position in left image
                    vfPoints[1].push_back(cv::Point2f(px_cur[0],px_cur[1]));
                }
                else
                {
                    vfPoints[0].push_back(cv::Point2f(-1,-1));
                    vfPoints[1].push_back(cv::Point2f(-1,-1)); //preventing detect anything
                }
            }
            else
            {
                vfPoints[0].push_back(pF->mvKeys[i].pt); //position in left frame
                vfPoints[1].push_back(cv::Point2f(pF->mvKeys[i].pt.x
                                                  +(Tl2r.translation()[0]*right_cam.fx()/mean_depth),
                                      pF->mvKeys[i].pt.y)); //position in right frame
                 // if a point is out of boundary, it will ended up failing to track
            }
        }

    // compute image pyramid
    int nWinWidth=21;
    cv::Size winSize(nWinWidth,nWinWidth);   
    vector<cv::Mat> vRightImagePyramid, vLeftImagePyramid;
    const int LEVELS=pF->GetScaleLevels();
    cv::buildOpticalFlowPyramid(pF->img_pyr_[0], vLeftImagePyramid, winSize, LEVELS-1);
    cv::buildOpticalFlowPyramid(pRightF->img_pyr_[0], vRightImagePyramid, winSize, LEVELS-1);

    // klt tracking to find matches from left to right image, and test epipolar constraint
    cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    calcOpticalFlowPyrLK(vLeftImagePyramid, vRightImagePyramid,
                         vfPoints[0], vfPoints[1], status[0], err, winSize,
                3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 0.001);
    //what is returned for points failed to track? the initial position
    // if a point is out of boundary, it will ended up failing to track
    vfPoints[2]=vfPoints[0];
    calcOpticalFlowPyrLK(vRightImagePyramid, vLeftImagePyramid,
                         vfPoints[1], vfPoints[2], status[1], err, winSize,
                3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 0.001);
    // recover matches 
    vector<cv::KeyPoint> & right_keys= pRightF->mvKeys;
    right_keys.reserve(sz);
    int nmatches=0;
    for(size_t which=0; which<status[0].size(); ++which){
        if(status[0][which] && status[1][which]){
            cv::Point2f delta=vfPoints[2][which] - vfPoints[0][which];
            bool bInImage=(vfPoints[1][which].x>=0.f)&&(vfPoints[1][which].y>=0.f)&&
                (vfPoints[1][which].x<right_cam.width())&&(vfPoints[1][which].y<right_cam.height());
            cv::KeyPoint kp2=pF->mvKeysUn[which];
            kp2.pt=vfPoints[1][which]; //TODO: should undistort this feature before epipolar check
            if(bInImage &&  CheckDistEpipolarLine(pF->mvKeysUn[which],kp2,Flr,pF) && (delta.dot(delta)) <= 3)
                // delta motion is very harsh
            {
                right_keys.push_back(kp2);
                ++nmatches;
            }
            else
            {
                right_keys.push_back(cv::KeyPoint(-1, -1, 21));
            }
        }
        else
        {
            right_keys.push_back(cv::KeyPoint(-1, -1, 21));
        }
    }
    assert(right_keys.size()==sz);
    vnMatches12.resize(sz, -1);
    //remove unmatched points
    std::vector<cv::KeyPoint> vMatchedKeys2;
    vMatchedKeys2.reserve(nmatches);

    int detective=0;
    for(size_t i=0; i<sz; ++i)
    {
        if(right_keys[i].pt.x==-1)
            continue;
        vMatchedKeys2.push_back(right_keys[i]);
        vnMatches12[i]=detective;
        ++detective;
    }
    assert(detective== nmatches);
    pRightF->mvKeys= vMatchedKeys2;

    return nmatches;*/
    return 0;
}
int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, Eigen::Matrix3d F12,
vector<cv::KeyPoint> &vMatchedKeys1, vector<cv::KeyPoint> &vMatchedKeys2, vector<pair<size_t, size_t> > &vMatchedPairs)
{
    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<cv::KeyPoint> vKeysUn1 = pKF1->GetKeyPointsUn();
    cv::Mat Descriptors1 = pKF1->GetDescriptors();
    DBoW2::FeatureVector vFeatVec1 = pKF1->GetFeatureVector();

    vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    vector<cv::KeyPoint> vKeysUn2 = pKF2->GetKeyPointsUn();
    cv::Mat Descriptors2 = pKF2->GetDescriptors();
    DBoW2::FeatureVector vFeatVec2 = pKF2->GetFeatureVector();

    // Find matches between not tracked keypoints
    // Matching speeded-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    vector<bool> vbMatched2(vKeysUn2.size(),false);
    vector<int> vMatches12(vKeysUn1.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];

                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const cv::KeyPoint &kp1 = vKeysUn1[idx1];

                cv::Mat d1 = Descriptors1.row(idx1);

                vector<pair<int,size_t> > vDistIndex;

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    cv::Mat d2 = Descriptors2.row(idx2);

                    const int dist = DescriptorDistance(d1,d2);

                    if(dist>TH_LOW)
                        continue;

                    vDistIndex.push_back(make_pair(dist,idx2));
                }

                if(vDistIndex.empty())
                    continue;

                sort(vDistIndex.begin(),vDistIndex.end());
                int BestDist = vDistIndex.front().first;
                int DistTh = round(2*BestDist);

                for(size_t id=0; id<vDistIndex.size(); id++)
                {
                    if(vDistIndex[id].first>DistTh)
                        break;

                    int currentIdx2 = vDistIndex[id].second;
                    cv::KeyPoint &kp2 = vKeysUn2[currentIdx2];
                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
                    {
                        vbMatched2[currentIdx2]=true;
                        vMatches12[idx1]=currentIdx2;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            float rot = kp1.angle-kp2.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }

                        break;
                    }

                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    vMatchedKeys1.clear();
    vMatchedKeys1.reserve(nmatches);
    vMatchedKeys2.clear();
    vMatchedKeys2.reserve(nmatches);
    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;

        vMatchedKeys1.push_back(vKeysUn1[i]);
        vMatchedKeys2.push_back(vKeysUn2[vMatches12[i]]);
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, vector<MapPoint *> &vpMapPoints, float th)
{
    Eigen::Matrix3d Rcw = pKF->GetRotation();
    Eigen::Vector3d tcw = pKF->GetTranslation();

    const float &fx = pKF->cam_.fx();
    const float &fy = pKF->cam_.fy();
    const float &cx = pKF->cam_.cx();
    const float &cy = pKF->cam_.cy();

    const int nMaxLevel = pKF->GetScaleLevels()-1;
    vector<float> vfScaleFactors = pKF->GetScaleFactors();

    Eigen::Vector3d Ow = pKF->GetCameraCenter();

    int nFused=0;

    for(size_t i=0; i<vpMapPoints.size(); i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        Eigen::Vector3d p3Dw = pMP->GetWorldPos();
        Eigen::Vector3d p3Dc = Rcw*p3Dw + tcw;

        // Depth must be positive
        if(p3Dc(2)<0.0f)
            continue;

        const float invz = 1/p3Dc(2);
        const float x = p3Dc(0)*invz;
        const float y = p3Dc(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3d PO = p3Dw-Ow;
        const float dist3D = PO.norm();

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3d Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const float ratio = dist3D/minDistance;

        vector<float>::iterator it = lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
        const int nPredictedLevel = min(static_cast<int>(it-vfScaleFactors.begin()),nMaxLevel);

        // Search in a radius
        const float radius = th*vfScaleFactors[nPredictedLevel];

        vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            const int kpLevel= pKF->GetKeyPointScaleLevel(idx);

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            cv::Mat dKF = pKF->GetDescriptor(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW && !(pMP->isBad())) // this point may be bad already
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    pMP->Replace(pMPinKF);                
            }
            else
            {
          
                pMP->AddObservation(pKF,bestIdx);
#ifndef MONO
                pMP->AddObservation(pKF,bestIdx, false);
#endif
                pKF->AddMapPoint(pMP,bestIdx);

            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, g2o::Sim3 Scw, const vector<MapPoint *> &vpPoints, float th)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->cam_.fx();
    const float &fy = pKF->cam_.fy();
    const float &cx = pKF->cam_.cx();
    const float &cy = pKF->cam_.cy();

    // Decompose Scw

    const float scw = Scw.scale();
    Eigen::Matrix3d Rcw = Scw.rotation().toRotationMatrix();
    Eigen::Vector3d tcw = Scw.translation()/scw;
    Eigen::Vector3d Ow = -Rcw.transpose()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    const int nMaxLevel = pKF->GetScaleLevels()-1;
    vector<float> vfScaleFactors = pKF->GetScaleFactors();

    int nFused=0;

    // For each candidate MapPoint project and match
    for(size_t iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        Eigen::Vector3d p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        Eigen::Vector3d p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc(2);
        const float x = p3Dc(0)*invz;
        const float y = p3Dc(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3d PO = p3Dw-Ow;
        const float dist3D = PO.norm();

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3d Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const float ratio = dist3D/minDistance;

        vector<float>::iterator it = lower_bound(vfScaleFactors.begin(), vfScaleFactors.end(), ratio);
        const int nPredictedLevel = min(static_cast<int>(it-vfScaleFactors.begin()),nMaxLevel);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF->GetScaleFactor(nPredictedLevel);

        vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int kpLevel = pKF->GetKeyPointScaleLevel(idx);

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            cv::Mat dKF = pKF->GetDescriptor(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    pMPinKF->Replace(pMP);
            }
            else
            {
            
                pMP->AddObservation(pKF,bestIdx);
#ifndef MONO
                pMP->AddObservation(pKF,bestIdx, false);
#endif
                pKF->AddMapPoint(pMP,bestIdx);

            }
            nFused++;
        }
    }
    return nFused;

}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                                   const float &s12, const Eigen::Matrix3d &R12,
                             const Eigen::Vector3d &t12, float th)
{
    const float fx = pKF1->cam_.fx();
    const float fy = pKF1->cam_.fy();
    const float cx = pKF1->cam_.cx();
    const float cy = pKF1->cam_.cy();

    // Camera 1 from world
    Eigen::Matrix3d R1w = pKF1->GetRotation();
    Eigen::Vector3d t1w = pKF1->GetTranslation();

    //Camera 2 from world
    Eigen::Matrix3d R2w = pKF2->GetRotation();
    Eigen::Vector3d t2w = pKF2->GetTranslation();

    //Transformation between cameras
    Eigen::Matrix3d sR12 = s12*R12;
    Eigen::Matrix3d sR21 = (1.0/s12)*R12.transpose();
    Eigen::Vector3d t21 = -sR21*t12;

    const int nMaxLevel1 = pKF1->GetScaleLevels()-1;
    vector<float> vfScaleFactors1 = pKF1->GetScaleFactors();

    vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const int nMaxLevel2 = pKF2->GetScaleLevels()-1;
    vector<float> vfScaleFactors2 = pKF2->GetScaleFactors();

    vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        Eigen::Vector3d p3Dw = pMP->GetWorldPos();
        Eigen::Vector3d p3Dc1 = R1w*p3Dw + t1w;
        Eigen::Vector3d p3Dc2 = sR21*p3Dc1 + t21;

        // Depth must be positive
        if(p3Dc2(2)<0.0)
            continue;

        float invz = 1.0/p3Dc2(2);
        float x = p3Dc2(0)*invz;
        float y = p3Dc2(1)*invz;

        float u = fx*x+cx;
        float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF2->IsInImage(u,v))
            continue;

        float maxDistance = pMP->GetMaxDistanceInvariance();
        float minDistance = pMP->GetMinDistanceInvariance();
        float dist3D = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        float ratio = dist3D/minDistance;

        vector<float>::iterator it = lower_bound(vfScaleFactors2.begin(), vfScaleFactors2.end(), ratio);
        const int nPredictedLevel = min(static_cast<int>(it-vfScaleFactors2.begin()),nMaxLevel2);

        // Search in a radius
        float radius = th*vfScaleFactors2[nPredictedLevel];

        vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;

            cv::KeyPoint kp = pKF2->GetKeyPointUn(idx);

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            cv::Mat dKF = pKF2->GetDescriptor(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        Eigen::Vector3d p3Dw = pMP->GetWorldPos();
        Eigen::Vector3d p3Dc2 = R2w*p3Dw + t2w;
        Eigen::Vector3d p3Dc1 = sR12*p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1(2)<0.0)
            continue;

        float invz = 1.0/p3Dc1(2);
        float x = p3Dc1(0)*invz;
        float y = p3Dc1(1)*invz;

        float u = fx*x+cx;
        float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        float maxDistance = pMP->GetMaxDistanceInvariance();
        float minDistance = pMP->GetMinDistanceInvariance();
        float dist3D = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        float ratio = dist3D/minDistance;

        vector<float>::iterator it = lower_bound(vfScaleFactors1.begin(), vfScaleFactors1.end(), ratio);
        const int nPredictedLevel = min(static_cast<int>(it-vfScaleFactors1.begin()),nMaxLevel1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        float radius = th*vfScaleFactors1[nPredictedLevel];

        vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit;

            cv::KeyPoint kp = pKF1->GetKeyPointUn(idx);

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            cv::Mat dKF = pKF1->GetDescriptor(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, float th)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const Eigen::Matrix3d Rcw = CurrentFrame.mTcw.rotationMatrix();
    const Eigen::Vector3d tcw = CurrentFrame.mTcw.translation();

    for(size_t i=0, iend=LastFrame.mvpMapPoints.size(); i<iend; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // Project
                Eigen::Vector3d x3Dw = pMP->GetWorldPos();
                Eigen::Vector3d x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0/x3Dc(2);

                float u = CurrentFrame.cam_.fx()*xc*invzc+CurrentFrame.cam_.cx();
                float v = CurrentFrame.cam_.fy()*yc*invzc+CurrentFrame.cam_.cy();

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

               int nPredictedOctave = LastFrame.mvKeysUn[i].octave;

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame.GetScaleFactor(nPredictedOctave);

                vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nPredictedOctave-1, nPredictedOctave+1);

                if(vIndices2.empty())
                    continue;

                cv::Mat dMP = LastFrame.mDescriptors.row(i);

                int bestDist = INT_MAX;
                int bestIdx2 = -1;

                for(vector<size_t>::iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    cv::Mat d = CurrentFrame.mDescriptors.row(i2);

                    int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }  

   //Apply rotation consistency
   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

       for(int i=0; i<HISTO_LENGTH; i++)
       {
           if(i!=ind1 && i!=ind2 && i!=ind3)
           {
               for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
               {
                   CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                   nmatches--;
               }
           }
       }
   }

   return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, float th ,int ORBdist)
{
    int nmatches = 0;

    const Eigen::Matrix3d Rcw = CurrentFrame.mTcw.rotationMatrix();
    const Eigen::Vector3d tcw = CurrentFrame.mTcw.translation();
    const Eigen::Vector3d Ow = -Rcw.transpose()*tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                Eigen::Vector3d x3Dw = pMP->GetWorldPos();
                Eigen::Vector3d x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0/x3Dc(2);

                float u = CurrentFrame.cam_.fx()*xc*invzc+CurrentFrame.cam_.cx();
                float v = CurrentFrame.cam_.fy()*yc*invzc+CurrentFrame.cam_.cy();

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Compute predicted scale level
                float minDistance = pMP->GetMinDistanceInvariance();
                Eigen::Vector3d PO = x3Dw-Ow;
                float dist3D = PO.norm();
                float ratio = dist3D/minDistance;
                std::vector<float> vScaleFactors =CurrentFrame.GetScaleFactors();
                vector<float>::iterator it = lower_bound(vScaleFactors.begin(), vScaleFactors.end(), ratio);
                const int nPredictedLevel = min(static_cast<int>(it-vScaleFactors.begin()),CurrentFrame.GetScaleLevels()-1);

                // Search in a window
                float radius = th*vScaleFactors[nPredictedLevel];

                vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = INT_MAX;
                int bestIdx2 = -1;

                for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    cv::Mat d = CurrentFrame.mDescriptors.row(i2);

                    int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->GetKeyPointUn(i).angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }


   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

       for(int i=0; i<HISTO_LENGTH; i++)
       {
           if(i!=ind1 && i!=ind2 && i!=ind3)
           {
               for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
               {
                   CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                   nmatches--;
               }
           }
       }
   }

    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
