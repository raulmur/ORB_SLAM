#ifndef STEREO_IMAGE_LOADER_H_
#define STEREO_IMAGE_LOADER_H_

#include "vio/timegrabber.h"
#include <opencv2/core/core.hpp>

enum dataset_type {KITTIOdoSeq=0, Tsukuba, MalagaUrbanExtract6, CrowdSourcedData, DiLiLi };

class StereoImageLoader
{
public:
    vio::TimeGrabber tg;
    dataset_type experim;
    std::string dir; //data directory

    cv::Mat M1l_,M2l_,M1r_,M2r_;

    StereoImageLoader(std::string time_file, dataset_type _experim, std::string input_path,
                      std::string settingFile);

    bool GetTimeAndRectifiedStereoImages(double &time_frame, cv::Mat& left_img, cv::Mat& right_img, int imageIndex);
};

void testStereoRectify();

void testReadDiLiLiSettingFile();

#endif
