
#include "vio/FrameGrabber.h"
#include "vio/utils.h"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;

namespace vio{
FrameGrabber::FrameGrabber(const std::string video, const int startFrameIndex, const int finishFrameIndex):
    is_measurement_good(false), mVideoFile(video), mCapture(video), mTk(-1),
    mnStartId(startFrameIndex),mnFinishId(finishFrameIndex), mnCurrentId(mnStartId), mbRGB(false), experim(CrowdSourcedData)
{
    double rate = mCapture.get(CV_CAP_PROP_FPS);
    if(!rate) std::cerr<<"Error opening video file "<<video<<endl;
    mCapture.set(CV_CAP_PROP_POS_FRAMES, mnStartId); //start from mnStartId, 0 based index
    if(mnFinishId == -1)
        mnFinishId = (int)(mCapture.get(CV_CAP_PROP_FRAME_COUNT)-1);
    else
        mnFinishId = std::min(mnFinishId, (int)(mCapture.get(CV_CAP_PROP_FRAME_COUNT)-1));

    int width= mCapture.get(CV_CAP_PROP_FRAME_WIDTH), height= mCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
    mnDownScale = GetDownScale(width, height, 1280);
    std::cout<<"Reading in video "<< mVideoFile <<std::endl;
    std::cout <<"start and finish index "<< startFrameIndex <<" "<< finishFrameIndex<< std::endl;
}

FrameGrabber::FrameGrabber(const std::string imageFolder, const std::string timeFile,
                           const int startFrameIndex, const int finishFrameIndex):
    is_measurement_good(false), mImageFolder(imageFolder), mTimestampFile(timeFile), mTG(timeFile), mTk(-1),
    mnStartId(startFrameIndex),mnFinishId(finishFrameIndex), mnCurrentId(mnStartId),
    mnDownScale(1), mbRGB(false), experim(Tsukuba)
{
    std::cout<<"Reading in images in folder "<< mImageFolder <<std::endl;
    std::cout <<"start and finish index "<< startFrameIndex <<" "<< finishFrameIndex<< std::endl;
}

bool FrameGrabber::grabFrame(cv::Mat & left_img, double & tk)
{
    double time_frame(-1);                  //timestamp of current frame
    double time_pair[2]={-1,mTk};              // timestamps of the previous and current images
    if(mnCurrentId > mnFinishId){
        left_img.release();
        return false;
    }

    if(experim == CrowdSourcedData){
        cv::Mat dst;
        assert(mCapture.get(CV_CAP_PROP_POS_FRAMES) == mnCurrentId);
        time_frame= mCapture.get(CV_CAP_PROP_POS_MSEC)/1000.0;
        mCapture.read(left_img);

        while(left_img.empty()){ // this happens when a frame is missed
            ++mnCurrentId;
            if(mnCurrentId > mnFinishId){
                left_img.release();
                return false;
            }
            assert(mCapture.get(CV_CAP_PROP_POS_FRAMES) == mnCurrentId);
            time_frame= mCapture.get(CV_CAP_PROP_POS_MSEC)/1000.0;
            mCapture.read(left_img);
        }

        if(mnDownScale>1){
            cv::pyrDown(left_img, dst);
            left_img= dst;
        }
        time_pair[0]=time_pair[1];
        time_pair[1]=time_frame;

        if(left_img.channels()==3)
        {
            cv::Mat temp;
            if(mbRGB)
                cvtColor(left_img, temp, CV_RGB2GRAY);
            else
                cvtColor(left_img, temp, CV_BGR2GRAY);
            left_img=temp;
        }
    }
    else{
        //for other types of image sequences
        char base_name[256];                // input file names
        string left_img_file_name;
        string right_img_file_name;

        switch(experim){
        case KITTIOdoSeq:
            sprintf(base_name,"%06d.png",mnCurrentId);
            left_img_file_name  = mImageFolder + "/image_0/" + base_name;
            right_img_file_name = mImageFolder + "/image_1/" + base_name;
            time_frame=mTG.readTimestamp(mnCurrentId);
            break;
        case Tsukuba:
            sprintf(base_name,"%05d.png",mnCurrentId + 1);
            left_img_file_name  = mImageFolder + "/tsukuba_daylight_L_" + base_name;
            right_img_file_name = mImageFolder + "/tsukuba_daylight_R_" + base_name;
            //time_frame=mnCurrentId/30.0;
            time_frame=mTG.readTimestamp(mnCurrentId);
            break;
        case MalagaUrbanExtract6:
            time_frame=mTG.extractTimestamp(mnCurrentId);
            left_img_file_name=mTG.last_left_image_name;
            right_img_file_name=left_img_file_name.substr(0, 30)+ "right"+left_img_file_name.substr(left_img_file_name.length()-4, 4);
            left_img_file_name= mImageFolder + "/"+ left_img_file_name;
            right_img_file_name= mImageFolder + "/"+ right_img_file_name;
        default:
            std::cerr<<"Please implement interface fot this dataset!"<<std::endl;
            break;
        }
        time_pair[0]=time_pair[1];
        time_pair[1]=time_frame;
        left_img=cv::imread(left_img_file_name, 0);
        //                 cv::Mat right_img=cv::imread(right_img_file_name, 0);

    }
    tk= time_frame;
    mTk =tk;
    ++mnCurrentId;
    return true;
}
}