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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <Eigen/Dense>

namespace ORB_SLAM
{
class ORBmatcher;
class Frame;
class ORBextractor
{
    friend class ORBmatcher;
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures = 1000, float scaleFactor = 1.2f, int nlevels = 8,
                 int scoreType=FAST_SCORE, int fastTh = 20, const float sigmaLevel0 =1.0f);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);
    // compute evenly distributed ORB features on an image
    /*void operator()(std::vector<cv::KeyPoint>& keypoints,  const std::vector<cv::Mat> & vImagePyramid,
                    const std::vector<cv::Mat> & vBlurredImagePyramid,
    cv::OutputArray descriptors,  const float detection_threshold=0.f);*/
    // Compute gravity aligned ORB features and descriptors on an image
    void operator()(cv::InputArray image, cv::InputArray mask,
                    std::vector<cv::KeyPoint>& keypoints,  cv::OutputArray descriptors,
                    std::vector<cv::KeyPoint>& _keypointsUn,
                    const cv::Mat & K, const cv::Mat & distCoef, const Eigen::Vector3d& ginc);
    // comptue ORB features on level 0 given keypoints,
    // bGAFD true means keypoints' angle is already determined by gravity direction
    void operator()( cv::InputArray image,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors, bool bGAFD=false);
    void operator()(std::vector<cv::KeyPoint>& keypoints,cv::OutputArray _descriptors,
                    const float detection_threshold);
    int inline GetLevels(){
        return nlevels;
    }

    float inline GetScaleFactor(int level=1)const{
        return mvScaleFactor[level];
    }

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
   
    void ComputePyramid(cv::Mat image);
    void ClonePyramid(std::vector<cv::Mat> & vImagePyramid);
    void ComputePyramid(const cv::Mat & image,   std::vector<cv::Mat>& vImagePyramid );
    void ComputeBlurredPyramid(const std::vector<cv::Mat> & vImagePyramid,
                                             std::vector<cv::Mat> & vBlurredImagePyramid);

 
    void MakeKeyPoints_Rest(std::vector<cv::KeyPoint> & vKeys,
                            const std::vector<cv::Mat>& vImagePyramid,
                              const std::vector<cv::Mat>& vBlurredImagePyramid,
                                          cv::OutputArray _descriptors);

    int nlevels;
    std::vector<float> mvScaleFactor; //scaleFactor^i, i is index of mvScaleFactors based 0, e.g., 1, 1.2, 1.2^2
    std::vector<float> mvInvScaleFactor;

    std::vector<float> mvLevelSigma2; // e.g, 1, 1.2^2, 1.2^4 ...
    std::vector<float> mvInvLevelSigma2; //e.g., 1, 1/1.2^2, 1/1.2^4
protected:

// brutal force compute keypoints without adaptive thresholding
    void ComputeKeyPointsBF(std::vector<std::vector<cv::KeyPoint> >& allKeypoints,
                            const std::vector<cv::Mat>& vImagePyramid);
    void ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, bool bGAFD=false);

    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;

    int scoreType;
    int fastTh;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;



    std::vector<cv::Mat> mvImagePyramid;
    std::vector<cv::Mat> mvBlurredImagePyramid;
};

void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                 const cv::Mat& img, const cv::Point* pattern,
                                 uchar* desc);
void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax);
void computeDescriptors(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
const std::vector<cv::Point>& pattern);
void UndistortKeyPoints(const std::vector<cv::KeyPoint>& vKeys, std::vector<cv::KeyPoint>& vKeysUn,
                         const cv::Mat & K, const cv::Mat & distCoef);

void computeKeyPointGAO(std::vector< cv::KeyPoint>& vKeys, std::vector< cv::KeyPoint>& vKeysUn,
                        const cv::Mat& K, const cv::Mat& distCoef, Eigen::Vector3d ginc);
void nonMaximaSuppression(const cv::Mat& src, const int sz, cv::Mat& dst, const cv::Mat mask= cv::Mat());
} //namespace ORB_SLAM

#endif

