#include "stereoSFM.h"
#include <assert.h>
using namespace std;
using namespace QCV;
namespace ORB_SLAM{
const double minDisparity_d (0.01);
const int cropOffset = 293; //for KITTI dataset, stereo SFM used a FOV scale factor 0.6
//in kitti/params_stereoSFM.xml. This results in better location accuracy and an offset in cx of principal point
// as a result, u = u' + [(w-w')/2.f], where w is original image width, w' is cropped image width,
// u is original feature x axis coordinate, u' is stored feature x coordinate
std::ostream& operator << (std::ostream &os, const QCV::SFeature & rhs)
{
    os<< rhs.u<<' '<<rhs.v<<' '<<rhs.d<<' '<<
         rhs.t<<' '<<rhs.e<<' '<<rhs.f<<' '<<
         rhs.idx<<' '<<rhs.state<<endl;
    return os;
}

std::istream & operator>>(std::istream &is, QCV::SFeature &rhs)
{    
    int stateNum(-1);
    is>>rhs.u>>rhs.v>>rhs.d>> rhs.t>>rhs.e>>rhs.f>> rhs.idx>>stateNum;
    rhs.state= (QCV::SFeature::EFeatureState)stateNum;
    return is;
}
void StereoSFM::nextFrame()
{
    //featrue tracks
    int nFrameId(-1),nFrameId2(-1), nFeatures(-1), lineNum(0);
    QCV::CFeatureVector featureVector;
    featureVector.reserve(3200);
    string tempStr;
    QCV::SFeature pat;

    mTracks>> nFrameId >> nFeatures;
    getline(mTracks, tempStr); //read out the enter character at the line end
    while(lineNum< nFeatures){
        getline(mTracks, tempStr);
        if(mTracks.fail())
            break;
        stringstream line_str(tempStr);
        line_str>>pat;
        pat.u += cropOffset;
        featureVector.push_back(pat);
        ++lineNum;
    }
    assert(nFeatures==3072);
    // delta motion frame id + 3x4 row major matrix
    Eigen::Matrix3d rot;
    Eigen::Vector3d trans;
    mDelta>> nFrameId2>> rot(0,0) >> rot(0,1) >> rot(0,2)>> trans(0) >>
                                rot(1,0) >> rot(1,1) >> rot(1,2)>> trans(1) >>
                                rot(2,0) >> rot(2,1) >> rot(2,2)>> trans(2);
    assert(nFrameId2 == nFrameId);
    mTcp=Sophus::SE3d(rot, trans);
    mb_Tr_valid= nFrameId == mnId+1;
    mnId= nFrameId;
    m_pIdx_i = m_cIdx_i;
    m_cIdx_i++; m_cIdx_i %= 2;

    m_trackHistoryACTUAL_v[m_cIdx_i] = featureVector;
}
void StereoSFM::getStereoMatches(std::vector<p_match> &p_matched)
{
    p_matched.clear();

    for (int i = 0; i < (signed)m_trackHistoryACTUAL_v[m_cIdx_i].size(); ++i)
    {
        const SFeature &curr = m_trackHistoryACTUAL_v[m_cIdx_i][i];
        if ( ( curr.state == SFeature::FS_TRACKED ||
               curr.state == SFeature::FS_NEW ) &&
             curr.d > minDisparity_d )
        {
            p_matched.push_back(p_match(-1,-1,-1,-1,-1,-1, curr.u,curr.v,i,curr.u -curr.d,curr.v,i));
        }
    }
}
void StereoSFM::getQuadMatches(std::vector<p_match> &p_matched)
{
    p_matched.clear();
    if ( m_trackHistoryACTUAL_v[m_cIdx_i].size() == m_trackHistoryACTUAL_v[m_pIdx_i].size() )
    {
        for (int i = 0; i < (signed)m_trackHistoryACTUAL_v[m_cIdx_i].size(); ++i)
        {
            if ( m_trackHistoryACTUAL_v[m_cIdx_i][i].state == SFeature::FS_TRACKED &&
                 ( m_trackHistoryACTUAL_v[m_pIdx_i][i].state == SFeature::FS_TRACKED ||
                   m_trackHistoryACTUAL_v[m_pIdx_i][i].state == SFeature::FS_NEW ) &&
                 m_trackHistoryACTUAL_v[m_pIdx_i][i].d > minDisparity_d )
            {

                const SFeature &curr = m_trackHistoryACTUAL_v[m_cIdx_i][i];
                const SFeature &prev = m_trackHistoryACTUAL_v[m_pIdx_i][i];
                if (curr.t >= 1 && curr.d > minDisparity_d)
                {
                    p_matched.push_back(p_match(prev.u,prev.v,i,prev.u - prev.d,prev.v,i,
                                                curr.u,curr.v,i,curr.u - curr.d,curr.v,i));
                }
            }
        }
    }
    else{
        printf("m_trackHistoryACTUAL_v[m_cIdx_i] and [m_pIdx_i] not equal size %d, %d\n",
               (int) m_trackHistoryACTUAL_v[m_cIdx_i].size(), (int)m_trackHistoryACTUAL_v[m_pIdx_i].size());
        assert(mnId==0);
    }
}

}
