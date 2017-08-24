#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H

#include <iostream>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "vio/timegrabber.h" //for timegrabber
namespace vio{
class FrameGrabber{
public:
    /**
     * @param video filename of the video
     * @param startFrameIndex 0 based starting frame index within the video
     * @param finishFrameIndex 0 based finishing frame index within the video, if not set, then the whole video is processed
     */
    FrameGrabber(const std::string video, const int startFrameIndex, const int finishFrameIndex = -1);

    /**
     * @param imageFolder the directory that contains the left and right image seq without slash
     * @param timeFile file that contains the timestamps of all images in the folder
     * @param startFrameIndex 0 based starting frame index within the image directory
     * @param finishFrameIndex 0 based finishing frame index within the image directory, if not set, then the whole video is processed
     */
    FrameGrabber(const std::string imageFolder, const std::string timeFile,
                 const int startFrameIndex, const int finishFrameIndex = -1);

    FrameGrabber(const FrameGrabber &)=delete;
    FrameGrabber & operator=(const FrameGrabber &)=delete;
    ~FrameGrabber(){}

    /**
     * @brief Get next frame and its timestamp
     * @param frame grabbed frame
     * @param tk timestamp of the grabbed frame
     */
    bool grabFrame(cv::Mat & frame, double & tk);
    int getCurrentId(){return mnCurrentId - 1;} //because it's incremented once a frame is obtained
protected:
    bool is_measurement_good;         //does the measurement fit our requirements?
    std::string mVideoFile;
    cv::VideoCapture mCapture;

    std::string mImageFolder;
    std::string mTimestampFile;

    TimeGrabber mTG;
    double mTk; // tk, timestamp of the current frame, i.e., the last grabbed frame

    const int mnStartId; /// 0 based starting frame index within the video or the image directory
    int mnFinishId; /// 0 based finishing frame index within the video or the image directory
    int mnCurrentId; /// current frame index 0 based starting frame index within the video or the image directory
    int mnDownScale;
    bool mbRGB; ///if color, RGB or BGR
    enum DatasetType {KITTIOdoSeq=0, Tsukuba, MalagaUrbanExtract6, CrowdSourcedData };
    DatasetType experim;
};
}
#endif // FRAMEGRABBER_H
