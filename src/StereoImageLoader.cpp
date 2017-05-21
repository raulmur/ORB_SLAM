#include "StereoImageLoader.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

StereoImageLoader::StereoImageLoader(std::string time_file, dataset_type _experim, std::string input_path,
                                     std::string settingFile):
    tg(time_file, _experim == DiLiLi? 1:0), experim(_experim), dir(input_path)
{
    if(_experim != DiLiLi)
        return;
    if(settingFile.empty())
    {
        std::cerr<<"You should calculate the rectification parameters for DiLiLi dataset!"<<std::endl;
        return;
    }

    cv::FileStorage fsSettings(settingFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings in StereoImageLoader" << std::endl;
        return;
    }

    cv::Mat K_l, K_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
        return;
    }

    cv::FileNode T_lr = fsSettings["Stereo.se3Right2LeftRaw"];
    cv::FileNode T_rl = fsSettings["Stereo.se3Left2RightRaw"];
    cv::Mat T_l2r; //float type
    if(!T_rl.empty()) {
        T_rl>>T_l2r;
    }
    else
    {
        assert(!T_lr.empty());
        T_lr>>T_l2r;
        assert(T_l2r.depth() == CV_64F);
        cv::Mat R_l2r = T_l2r.colRange(0,3).rowRange(0,3).t();
        T_l2r.col(3).rowRange(0,3)= - R_l2r*T_l2r.col(3).rowRange(0,3);
        T_l2r.colRange(0,3).rowRange(0,3)= R_l2r;
    }

    cv::Mat Rl2r = T_l2r.colRange(0,3).rowRange(0,3);
    cv::Mat tlinr = T_l2r.col(3).rowRange(0,3);

    //output
    cv::Mat R_l, R_r, P_l, P_r, Q;

    int flags= cv::CALIB_ZERO_DISPARITY;
    double alpha= 0;
    cv::Size newImageSize=cv::Size(0,0);
    cv::Rect validPixROI1;
    cv::Rect validPixROI2;

    cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l, rows_l), Rl2r, tlinr, R_l, R_r,
                      P_l, P_r, Q, flags, alpha,
                      newImageSize, &validPixROI1, &validPixROI2);

    std::cout<<"Rectified R_l "<<std::endl<< R_l<< std::endl;
    std::cout<<"Rectified R_r "<<std::endl<< R_r<< std::endl;
    std::cout<<"Rectified P_l "<<std::endl<< P_l<< std::endl;
    std::cout<<"Rectified P_r "<<std::endl<< P_r<< std::endl;
    std::cout <<"Rectified baseline "<< std::endl<< P_r.colRange(0,3).inv()*P_r.col(3)<< std::endl;

    // debug
    double fx = fsSettings["Camera.fx"];
    double fy = fsSettings["Camera.fy"];
    double cx = fsSettings["Camera.cx"];
    double cy = fsSettings["Camera.cy"];
    cv::Mat K_expected= (cv::Mat_<double>(3,3)<<fx, 0, cx, 0, fy, cy, 0, 0, 1);
    assert(cv::norm(P_l.colRange(0,3) - K_expected)<1e-6);

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l_,M2l_);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r_,M2r_);

}

bool StereoImageLoader::GetTimeAndRectifiedStereoImages(double &time_frame, cv::Mat& left_img, cv::Mat& right_img, int imageIndex){

    const bool mbRGB= false;
    char base_name[256];                // input file names
    std::string left_img_file_name;
    std::string right_img_file_name;
    cv::Mat tempLeftImg, tempRightImg;
    switch(experim){
    case KITTIOdoSeq:
        sprintf(base_name,"%06d.png",imageIndex);
        left_img_file_name  = dir + "/image_0/" + base_name;
        right_img_file_name = dir + "/image_1/" + base_name;
        time_frame=tg.readTimestamp(imageIndex);
        left_img=cv::imread(left_img_file_name, 0);
        right_img=cv::imread(right_img_file_name, 0);

        break;
    case Tsukuba:
        sprintf(base_name,"%05d.png",imageIndex);
        left_img_file_name  = dir + "/tsukuba_daylight_L_" + base_name;
        right_img_file_name = dir + "/tsukuba_daylight_R_" + base_name;
        time_frame=(imageIndex-1)/30.0;
        left_img=cv::imread(left_img_file_name, 0);
        right_img=cv::imread(right_img_file_name, 0);

        break;
    case MalagaUrbanExtract6:
        time_frame=tg.extractTimestamp(imageIndex);
        left_img_file_name=tg.last_left_image_name;
        right_img_file_name=left_img_file_name.substr(0, 30)+ "right"+left_img_file_name.substr(left_img_file_name.length()-4, 4);
        left_img_file_name=dir+ "/"+ left_img_file_name;
        right_img_file_name=dir+ "/"+ right_img_file_name;
        left_img=cv::imread(left_img_file_name, 0);
        right_img=cv::imread(right_img_file_name, 0);
        break;
    case DiLiLi:
        time_frame=tg.extractTimestamp(imageIndex, false);
        sprintf(base_name,"%d.pbm",imageIndex);
        left_img_file_name  = dir + "/left_" + base_name;
        right_img_file_name = dir + "/right_" + base_name;
        tempLeftImg=cv::imread(left_img_file_name, CV_LOAD_IMAGE_UNCHANGED);
        tempRightImg=cv::imread(right_img_file_name, CV_LOAD_IMAGE_UNCHANGED);

        if(tempLeftImg.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                 << std::string(left_img_file_name) << std::endl;
            return false;
        }

        if(tempRightImg.empty())
        {
            std::cerr << std::endl << "Failed to load image at: "
                 << std::string(right_img_file_name) << std::endl;
            return false;
        }

        cv::remap(tempLeftImg,left_img, M1l_,M2l_,cv::INTER_LINEAR);
        cv::remap(tempRightImg,right_img,M1r_,M2r_,cv::INTER_LINEAR);

        if(left_img.channels()==3)
        {
            if(mbRGB)
            {
                cv::cvtColor(left_img,left_img,CV_RGB2GRAY);
                cv::cvtColor(right_img,right_img,CV_RGB2GRAY);
            }
            else
            {
                cv::cvtColor(left_img,left_img,CV_BGR2GRAY);
                cv::cvtColor(right_img,right_img,CV_BGR2GRAY);
            }
        }
        else if(left_img.channels()==4)
        {
            if(mbRGB)
            {
                cv::cvtColor(left_img,left_img,CV_RGBA2GRAY);
                cv::cvtColor(right_img,right_img,CV_RGBA2GRAY);
            }
            else
            {
                cv::cvtColor(left_img,left_img,CV_BGRA2GRAY);
                cv::cvtColor(right_img,right_img,CV_BGRA2GRAY);
            }
        }
        break;

    default:    
        std::cerr<<"Please implement interface fot this dataset!"<<std::endl;
        return false;
    }
    return true;
}
