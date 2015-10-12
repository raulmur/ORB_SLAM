#ifndef STEREO_SFM_H
#define STEREO_SFM_H
#include "viso2/p_match.h"
#include "/home/jhuai/qcv/qcv-code/modules/misc/feature.h"
#include "sophus/se3.hpp"
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
namespace ORB_SLAM{
class StereoSFM{
public:
    StereoSFM(): mnId(-2), m_pIdx_i (0), m_cIdx_i (1), m_trackHistoryACTUAL_v (2), mb_Tr_valid(false)
    {
    }
    void init(std::string feature_tracks= "", std::string delta_motion="")
    {
        mTracks.open(feature_tracks, std::ifstream::in);
        mDelta.open(delta_motion, std::ifstream::in);
        if (!mTracks.is_open()) {
            std::cerr<< "Error in opening feature tracks file"<<std::endl;
        }
        if (!mDelta.is_open()) {
            std::cerr<< "Error in opening delta motion file"<<std::endl;
        }
    }
    ~StereoSFM()
    {
        mTracks.close();
        mDelta.close();
    }
    /// read in feature tracks and incremental motion for the next frame
    void nextFrame();
    ///get matches between left and right images from the current feature vector
    void getStereoMatches(std::vector<p_match> &p_matched);
    ///get matches between two stereo pairs from the previous and currrent feature vector
    void getQuadMatches(std::vector<p_match> &p_matched);

    Sophus::SE3d getDeltaMotion(){ return mTcp;}

    size_t getNumDenseFeatures(){return m_trackHistoryACTUAL_v[m_cIdx_i].size();}
public:
    std::ifstream mTracks;
    std::ifstream mDelta;
    /// absolute current frame id as from the feature track file, often 0 based
    int mnId;
    /// delta motion, SE3 transform from previous to current frame, camera frame is right, down, forward
    Sophus::SE3d mTcp;
    /// binarized previous frame index, only taking values of 0 or 1.
    int                               m_pIdx_i;
    /// binarized current frame index.
    int                               m_cIdx_i;
    /// feature vector, only stores previous and current feature vectors which are used to extract quad matches
    std::vector<QCV::CFeatureVector>       m_trackHistoryACTUAL_v;
    ///is the incremental motion valid
    bool mb_Tr_valid;
};
std::ostream& operator << (std::ostream &os, const QCV::SFeature & rhs);
std::istream & operator>>(std::istream &is, QCV::SFeature &rhs);

}
#endif
