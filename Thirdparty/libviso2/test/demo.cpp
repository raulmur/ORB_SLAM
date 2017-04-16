/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage:e.g.1: ./viso2 /path/to/2010_03_09_drive_0019 /media/jianzhuhuai0108/Mag/2010_03_09_drive_0019.txt
  e.g.2: ./viso2 /media/jianzhuhuai0108/Mag/2010_03_09_drive_0019 /media/jianzhuhuai0108/Mag/2010_03_09_drive_0019.txt
  e.g.3: ./viso2 /media/jianzhuhuai0108/Mag/NewTsukubaStereoDataset/illumination/daylight /media/jianzhuhuai0108/Mag/Tsukuba.txt
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdint.h>
#include <sys/time.h> // for timer
#include "viso_stereo.h"
#include "viso_mono.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace libviso2;
enum dataset_type {KITTISeq=0, Tsukuba, MalagaUrbanExtract6} experim=KITTISeq;

class   StopWatch
{
public:
    StopWatch()
    {
        running_ = false;
        time_ = 0;
    }

    void start()
    {
        assert(running_==false);
        gettimeofday(&start_time_, NULL);
        running_ = true;
    }

    void stop()
    {
        assert(running_);
        gettimeofday(&end_time_, NULL);
        long seconds  = end_time_.tv_sec  - start_time_.tv_sec;
        long useconds = end_time_.tv_usec - start_time_.tv_usec;
        time_ = ((seconds) + useconds*0.000001);
        running_ = false;
    }

    double read_current_time()
    {
        assert(running_);
        timeval cur_time;
        gettimeofday(&cur_time, NULL);
        long seconds  = cur_time.tv_sec  - start_time_.tv_sec;
        long useconds = cur_time.tv_usec - start_time_.tv_usec;
        return ((seconds) + useconds*0.000001);
    }

    double get_stopped_time()
    {
        assert(running_==false);
        return time_;
    }

    inline void reset()
    {
        time_ = 0;
    }

private:
    timeval start_time_;
    timeval end_time_;
    double time_;
    bool running_;
};
cv::Mat viso2cv(const libviso2::Matrix& pose)
{
    cv::Mat res(pose.m, pose.n, CV_32F);
    for(int i=0; i<pose.m; ++i)
        for(int j=0; j<pose.n;++j)
            res.at<float>(i,j)=pose.val[i][j];
    return res.clone();
}
int main (int argc, char** argv) {

    // we need the path name to 2010_03_09_drive_0019 as input argument
    if (argc<2) {
        cerr << "Usage: path/to/settings.yaml" << endl;
        return 1;
    }
    cv::FileStorage fSettings(argv[1], cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr<<"Wrong path to settings file!"<<endl;
        return 1;
    }
    string dir = fSettings["input_path"]; // sequence directory
    string output_file=fSettings["output_file"];
    ofstream out_stream(output_file.c_str(), std::ios::out);
    out_stream<<"%Each row is index, P_{c_i}^w, 3x4 matrix, in row major order"<<endl;
    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h


    int startIndex=fSettings["startIndex"];
    int finishIndex=fSettings["finishIndex"];

#ifdef STEREO
    libviso2::VisualOdometryStereo::parameters param;
    // calibration parameters for sequence 2010_03_09_drive_0019
    param.calib.f  = fSettings["calib.f"]; // focal length in pixels
    param.calib.cu = fSettings["calib.cu"]; // principal point (u-coordinate) in pixels
    param.calib.cv = fSettings["calib.cv"]; // principal point (v-coordinate) in pixels

    param.base     = fSettings["baseline"]; // baseline in meters
    param.inlier_threshold =sqrt(5.991);
#else
    libviso2::VisualOdometryMono::parameters param;
    // calibration parameters for sequence 2010_03_09_drive_0019
    param.calib.f  = fSettings["calib.f"]; // focal length in pixels
    param.calib.cu = fSettings["calib.cu"]; // principal point (u-coordinate) in pixels
    param.calib.cv = fSettings["calib.cv"]; // principal point (v-coordinate) in pixels

    param.match.refinement=2;
#endif

    // init visual odometry
#ifdef STEREO
    libviso2::VisualOdometryStereo viso(param);
#else
    libviso2::VisualOdometryMono viso(param);
#endif
    std::vector<p_match> pMatches2, pMatches;
    vector<cv::KeyPoint> keys1, keys2, prev_keys1, prev_keys2;
    cv::Mat prev_left_img;
    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    libviso2::Matrix pose = libviso2::Matrix::eye(4);
    libviso2::Matrix pose_old= libviso2::Matrix::eye(4);
    StopWatch sw;
    sw.start();
    // loop through all frames i=0:372
    for (int32_t i=startIndex; i<finishIndex+1; i++) {
        // input file names
        char base_name[256];
        string left_img_file_name;
        string right_img_file_name;
        switch(experim){
        case KITTISeq:
            sprintf(base_name,"%06d.png",i);
            left_img_file_name  = dir + "/image_0/" + base_name;
            right_img_file_name = dir + "/image_1/" + base_name;
            break;
        case Tsukuba:
            sprintf(base_name,"%05d.png",i);
            left_img_file_name  = dir + "/tsukuba_daylight_L_" + base_name;
            right_img_file_name = dir + "/tsukuba_daylight_R_" + base_name;

            break;
        default:
            cerr<<"Please implement interface fot this dataset!"<<endl;
            break;
        }
        // catch image read/write errors here
        try {       
            // load left and right input image
            Mat left_img=imread(left_img_file_name, 0);
            Mat right_img=imread(right_img_file_name, 0);
            const int width= left_img.cols;
            const int height= left_img.rows;

            cout << "Processing: Frame: " << i<<endl;
            // compute visual odometry
            int32_t dims[] = {width,height,width};
            viso.Tr_valid =false; //disable prior aided feature matching
            if(i==startIndex)
                out_stream<< i<<" "<<pose.getMat(0, 0, 0,3)<<"\t"<<pose.getMat(1, 0, 1,3)<<"\t"<<pose.getMat(2, 0, 2,3)<<endl;
#ifdef STEREO
            if (viso.process(left_img.data,right_img.data,dims)) {
#else
            if (viso.process2(left_img.data,dims)) {
#endif
                          // on success, update current pose
                pose = pose * Matrix::inv(viso.getMotion());
                //examine pMatches
                /*for(int jack=0; jack<pMatches2.size();++jack)
                {
                    for(int kim=0; kim<pMatches.size();++kim)
                    {
                        if(pMatches[kim].i1c==pMatches2[jack].i1p)
                        {
                            int diff1=abs(pMatches[kim].u1c-pMatches2[jack].u1p);
                            int diff2=abs(pMatches[kim].v1c-pMatches2[jack].v1p);
                            int diff3=abs(pMatches[kim].u2c-pMatches2[jack].u2p);
                      //      if(diff1+diff2>0)
                      //          cerr<< diff1 <<" "<< diff2<< " "<<diff3<<endl;
                        }
                    }
                }
                vector<cv::KeyPoint> keys_Geiger;
                for(int jack=0; jack<pMatches2.size();++jack)
                {
                    keys_Geiger.push_back(cv::KeyPoint(pMatches2[jack].u1c, pMatches2[jack].v1c, 7));
                }
                // draw pMatches and compare to FAST corners
                const int fast_th = 20; // corner detector response threshold
                vector<cv::KeyPoint> keys_FAST;
                cv::FAST(left_img, keys_FAST, fast_th, true);
                Ptr<FeatureDetector> detector = FeatureDetector::create("FAST");
                Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("ORB");
                detector->detect( left_img, keys_FAST );
                Mat descriptors1;
                descriptorExtractor->compute( left_img, keys_FAST, descriptors1 );

                cv::Mat out_image;
                cv::drawKeypoints(left_img, keys_FAST, out_image, Scalar(0,255,0));
                cv::drawKeypoints(left_img, keys_Geiger, out_image, Scalar(0,0, 255), 1);
                cv::imshow("comparison", out_image);
                cv::waitKey();*/
           /*     keys1.clear();
                keys2.clear();

                cv::Mat Tw2c1= viso2cv(pose);

                cv::Mat Rw2c1= Tw2c1.rowRange(0,3).colRange(0,3).clone();
                cv::Mat twinc1=Tw2c1.rowRange(0,3).col(3).clone();
                cv::Mat Tw2c2= viso2cv(pose_old);
                cv::Mat Rw2c2= Tw2c2.rowRange(0,3).colRange(0,3).clone();
                cv::Mat twinc2=Tw2c2.rowRange(0,3).col(3).clone();

                vector<cv::DMatch> matches122_cv;
                for(int i=0; i<pMatches2.size(); ++i)
                {
                    //Assume left right image rectified and no distortion
                    const cv::KeyPoint kp1 = cv::KeyPoint(pMatches2[i].u1c, pMatches2[i].v1c, 7);//current keyframe
                    const cv::KeyPoint kp2 = cv::KeyPoint(pMatches2[i].u2c, pMatches2[i].v2c, 7);
                    const cv::KeyPoint kp3 = cv::KeyPoint(pMatches2[i].u1p, pMatches2[i].v1p, 7);
                    const cv::KeyPoint kp4 = cv::KeyPoint(pMatches2[i].u2p, pMatches2[i].v2p, 7);
                    if(kp1.pt.x -kp2.pt.x<3 || kp3.pt.x- kp4.pt.x<3)//parallax
                        continue;

                    float base=param.base;
                    float base_disp = base/(kp1.pt.x -kp2.pt.x);
                    cv::Mat x3D1(3,1,CV_32F);
                    x3D1.at<float>(0) = (kp1.pt.x- param.calib.cu)*base_disp;
                    x3D1.at<float>(1) = ((kp1.pt.y+ kp2.pt.y)/2 - param.calib.cv)*base_disp;
                    x3D1.at<float>(2) = param.calib.f*base_disp;
                    x3D1= Rw2c1.t()*(x3D1- twinc1);
                    base_disp = base/(kp3.pt.x -kp4.pt.x);
                    cv::Mat x3D2(3,1,CV_32F);
                    x3D2.at<float>(0) = (kp3.pt.x- param.calib.cu)*base_disp;
                    x3D2.at<float>(1) = ((kp3.pt.y+ kp4.pt.y)/2 - param.calib.cv)*base_disp;
                    x3D2.at<float>(2) = param.calib.f*base_disp;
                    x3D2= Rw2c2.t()*(x3D2 - twinc2);
                    if(abs(x3D1.at<float>(2)- x3D2.at<float>(2))>0.2)
                        continue;
                    cv::Mat x3D= (x3D1+ x3D2)/2;
                    keys1.push_back(cv::KeyPoint(pMatches2[i].u1p, pMatches2[i].v1p, 7));
                    keys2.push_back(cv::KeyPoint(pMatches2[i].u1c, pMatches2[i].v1c, 7));

                }
                cout<<"created new points:"<<keys2.size()<<endl;
                Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("ORB");
                Mat descriptors1, descriptors2;
                descriptorExtractor->compute( prev_left_img, keys1, descriptors1 );
                descriptorExtractor->compute( left_img, keys2, descriptors2 );
                Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create("BruteForce");
                descriptorMatcher->match(descriptors1,descriptors2, matches122_cv, Mat());
                cv::Mat drawImg;
//                drawMatches( prev_left_img, keys1, left_img, keys2, matches122_cv, drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255));
                drawKeypoints( left_img, keys2, drawImg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
                cv::imshow("Matches", drawImg);
                cv::waitKey(10);*/

                // output some statistics
                double num_all_matches= viso.getAllMatches().size();
                double num_matches = viso.getNumberOfMatches();
                double num_inliers = viso.getNumberOfInliers();
                cout << "Tr valid, Inliers out of bucketed Matches of all matches: " <<viso.Tr_valid<<" "<<num_inliers
                     <<" "<< num_matches<<" "<< num_all_matches<<endl;
                //            cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
//                cout << pose << endl << endl;
#ifdef STEREO
                out_stream<< i<<" "<<pose.getMat(0, 0, 0,3)<<"\t"<<pose.getMat(1, 0, 1,3)<<"\t"<<pose.getMat(2, 0, 2,3)<<endl;
#else
                Matrix delta= viso.getMotion();
                out_stream<< i<<" "<<delta.getMat(0, 0, 0,3)<<"\t"<<delta.getMat(1, 0, 1,3)<<"\t"<<delta.getMat(2, 0, 2,3)<<endl;
#endif
            } else {
                cout << "Processing: Frame: " << i<<" failed!" << endl;
            }
            prev_left_img=left_img;
            // catch image read errors here
        } catch (...) {
            cerr << "ERROR: Couldn't read input files!" << endl;
            return 1;
        }
    }
    sw.stop();
    double calc_time = sw.get_stopped_time();
    double time_per_frame=calc_time/(finishIndex-startIndex+1);
    cout<<"Calc_time:"<<calc_time<<";"<<"time per frame:"<<time_per_frame<<endl;

    // output
    cout << "Demo complete! Exiting ..." << endl;
    out_stream.close();
    // exit
    return 0;
}

