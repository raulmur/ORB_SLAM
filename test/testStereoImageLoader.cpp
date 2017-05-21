#include <opencv2/core/core.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

//compute the rectify matrices for the Euroc stereo images, the input parameters
// for the Euroc dataset can be found in OKVIS/config, and the expected output
// matrices are given in ORB_SLAM2/Examples/Stereo/EuRoc.yaml
void testStereoRectifyEuroc(){
    cv::Mat T_SCl = (cv::Mat_<double>(4, 4)<<
                     0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                     0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                     -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                     0.0, 0.0, 0.0, 1.0);
    cv::Size left_dimension(752, 480);
    cv::Mat D_l= (cv::Mat_<double>(4,1)<<-0.28340811217, 0.0739590738929, 0.000193595028569, 1.76187114545e-05);
    cv::Point2d left_focal(458.654880721, 457.296696463);
    cv::Point2d left_principal(367.215803962, 248.37534061);

    cv::Mat T_SCr = (cv::Mat_<double>(4, 4)<<
                     0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
                     0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
                     -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
                     0.0, 0.0, 0.0, 1.0);
    cv::Size right_dimension(752, 480);
    cv::Mat D_r= (cv::Mat_<double>(4,1)<<-0.283683654496, 0.0745128430929, -0.000104738949098, -3.55590700274e-05);

    cv::Point2d right_focal(457.587426604, 456.13442556);
    cv::Point2d right_principal(379.99944652, 255.238185386);


    cv::Mat K_l = (cv::Mat_<double>(3,3)<<left_focal.x, 0, left_principal.x, 0, left_focal.y, left_principal.y, 0, 0, 1);
    cv::Mat K_r = (cv::Mat_<double>(3,3)<<right_focal.x, 0, right_principal.x, 0, right_focal.y, right_principal.y, 0, 0, 1);

    cv::Mat Rl2r = T_SCr.colRange(0,3).rowRange(0,3).t()*T_SCl.colRange(0,3).rowRange(0,3);
    cv::Mat tlinr = T_SCr.colRange(0,3).rowRange(0,3).t()*(T_SCl.colRange(3,4).rowRange(0,3)-T_SCr.colRange(3,4).rowRange(0,3));
    cv::Mat Rw2l, Rw2r, P_l, P_r, Q;
    int flags= cv::CALIB_ZERO_DISPARITY;
    double alpha= 0;
    cv::Size newImageSize=cv::Size(0,0);
    cv::Rect validPixROI1;
    cv::Rect validPixROI2;

    cv::stereoRectify(K_l, D_l, K_r, D_r, left_dimension, Rl2r, tlinr, Rw2l, Rw2r,
                      P_l, P_r, Q, flags, alpha,
                      newImageSize, &validPixROI1, &validPixROI2);

    std::cout<<"R_l "<<std::endl<< Rw2l<< std::endl;
    std::cout<<"R_r "<<std::endl<< Rw2r<< std::endl;
    std::cout<<"P_l "<<std::endl<< P_l<< std::endl;
    std::cout<<"P_r "<<std::endl<< P_r<< std::endl;
}
void ConcatenateTwoImages(const cv::Mat& img1, const cv::Mat& img2, cv::Mat & res)
{
    // Get dimension of final image
      int rows = std::max(img1.rows, img2.rows);
      int cols = img1.cols + img2.cols;

      // Create a black image
//      cv::Mat3b res(rows, cols, cv::Vec3b(0,0,0));
      res = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));

      // Copy images in correct position
      img1.copyTo(res(cv::Rect(0, 0, img1.cols, img1.rows)));
      img2.copyTo(res(cv::Rect(img1.cols, 0, img2.cols, img2.rows)));
}

//test opencv rectification with DiLiLi dataset, these input parameters come from its data specs
void testStereoRectifyDiLiLi(){

    // Read camera parameters
    std::string settingFile ="/home/jhuai/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo_imu/dilili.yaml";

    std::string path = "/media/jhuai/Seagate/data/DILILI-0510_square_400_no_calib/0510_square_400_no_calib";
    std::string strImageLeft = path+ "/left_1.pbm";
    std::string strImageRight = path+ "/right_1.pbm";
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

    std::cout<<"R_l "<<std::endl<< R_l<< std::endl;
    std::cout<<"R_r "<<std::endl<< R_r<< std::endl;
    std::cout<<"P_l "<<std::endl<< P_l<< std::endl;
    std::cout<<"P_r "<<std::endl<< P_r<< std::endl;
    std::cout <<"Rectified baseline "<< std::endl<< P_r.colRange(0,3).inv()*P_r.col(3)<< std::endl;

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    //rectify a pair of images
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    imLeft = cv::imread(strImageLeft,CV_LOAD_IMAGE_UNCHANGED);
    imRight = cv::imread(strImageRight,CV_LOAD_IMAGE_UNCHANGED);

    if(imLeft.empty())
    {
        std::cerr << std::endl << "Failed to load image at: "
             << std::string(strImageLeft) << std::endl;
        return;
    }

    if(imRight.empty())
    {
        std::cerr << std::endl << "Failed to load image at: "
             << std::string(strImageRight) << std::endl;
        return;
    }

    cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

    cv::Mat leftComp;
    ConcatenateTwoImages(imLeft, imLeftRect, leftComp);
    cv::Mat rightComp;
    ConcatenateTwoImages(imRight, imRightRect, rightComp);
    cv::imshow("Left Rectified ", leftComp);
    cv::imshow("Right Rectified ", rightComp);
    cv::waitKey();

    cv::Mat mImGray = imLeftRect;
    cv::Mat imGrayRight = imRightRect;

    bool mbRGB= false;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cv::cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cv::cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cv::cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cv::cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cv::cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cv::cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cv::cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cv::cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }
}


//This function will crash because the DiLiLi yaml format is non canonical
void testReadDiLiLiSettingFile()
{
    std::string path = "/media/jhuai/Seagate/data/DILILI-0510_square_400_no_calib/DILILI-D001_20170511.yaml";
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: cannot open settings in testReadDiLiLiSettingFile" << std::endl;
        return;
    }

    cv::Mat accel_Tk;
    cv::FileNode accel_Tk_node = fsSettings["Imu"]["accel_Tk"];
    if(accel_Tk_node.isSeq()){
        accel_Tk_node >> accel_Tk;
        std::cout<<"accel_Tk"<<std::endl<< accel_Tk<< std::endl;
    }
    else
    {
        std::cout<<"accel_Tk not a sequence node "<<std::endl;
    }
}

int main()
{
    testStereoRectifyDiLiLi();
    return 0;
}
