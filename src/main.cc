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

#include<iostream>
#include<fstream>

#ifdef SLAM_USE_ROS
#include<ros/ros.h>
#include<ros/package.h>
#endif

#include<boost/thread.hpp>

#include<opencv2/core/core.hpp>

#include "global.h"

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Converter.h"
#include "StereoImageLoader.h"

#include "vio/utils.h"
#include "vikit/pinhole_camera.h"
#include <vikit/timer.h>

using namespace std;
std::string slamhome;

int main(int argc, char **argv)
{
#ifdef SLAM_USE_ROS
    ros::init(argc, argv, "ORBSLAM_DWO");
    ros::start();
#endif
    cout << endl << "ORBSLAM_DWO Copyright (C) 2014 Raul Mur-Artal, 2015 Jianzhu Huai" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;

    if(argc < 2)
    {
        cerr << "Usage:"<< argv[0]<<" <path_to_settings.yaml> [folder of /Vocabulary/ORBvoc.txt/yaml and /data]" << endl;
#ifdef SLAM_USE_ROS
        ros::shutdown();
#endif
        return 1;
    }

    // Load Settings and Check
    string strSettingsFile = argv[1];
//            ros::package::getPath("ORB_SLAM")+"/"+argv[2];

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr<<("Wrong path to settings. Path must be absolute or relative to ORB_SLAM package directory.");
 #ifdef SLAM_USE_ROS
        ros::shutdown();
#endif
        return 1;
    }

    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher framePublisher;

    if(argc > 2){
        slamhome = argv[2];
        if(!(slamhome.back() == '/' || slamhome.back() == '\\'))
            slamhome += '/';
    }

    //Load ORB Vocabulary
    string strVocFile = slamhome + (std::string)fsSettings["voc_file_path"];   //ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    std::cout << "VocFile path: " << strVocFile  << std::endl;

    size_t found = strVocFile.find_last_of('.');
    string extension = strVocFile.substr(found+1);

    ORB_SLAM::ORBVocabulary Vocabulary;

    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    if (extension == "yml" || extension == "YML"){
        // Old version to load vocabulary using cv::FileStorage
        cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
        if(!fsVoc.isOpened())
        {
            cerr << "Wrong path to vocabulary. Path must be absolute or relative to ORB_SLAM package directory." << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
#ifdef SLAM_USE_ROS
            ros::shutdown();
#endif
            return 1;
        }
        Vocabulary.load(fsVoc);
    }else if(extension == "txt" || extension == "TXT"){
        // New version to load vocabulary from text file "Data/ORBvoc.txt".
        // If you have an own .yml vocabulary, use the function
        // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
        bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. Path must be absolute or relative to ORB_SLAM package directory." << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
#ifdef SLAM_USE_ROS
            ros::shutdown();
#endif
            return 1;
        }
    }else {
        cerr << "Wrong path to vocabulary. Path must be absolute or relative to ORB_SLAM package directory." << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
#ifdef SLAM_USE_ROS
        ros::shutdown();
#endif
        return 1;
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    framePublisher.SetMap(&World);
#ifdef SLAM_USE_ROS
    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher mapPublisher(&World);
    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &framePublisher, &mapPublisher, &World, strSettingsFile);
#else
    ORB_SLAM::Tracking Tracker(&Vocabulary, &framePublisher, &World, strSettingsFile);
#endif

//    boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    int nStartId= fsSettings["startIndex"];
    int totalImages=fsSettings["finishIndex"];
    int numImages=nStartId;

    // slamhome is initialized in main.cpp
    string dir = slamhome + (std::string)fsSettings["input_path"]; // sequence directory
    string output_file= slamhome + (std::string)fsSettings["output_file"];
    string output_point_file = slamhome + (std::string)fsSettings["output_point_file"];
    std::cout << "input data dir: " << dir << std::endl;

    ofstream out_stream(output_file.c_str(), std::ios::out);
    if(!out_stream.is_open())
        SLAM_ERROR_STREAM("Error opening output file "<<output_file);
    out_stream<<"%Each row is timestamp, pose of camera in custom world frame, [txyz,qxyzw], v_Wc_s, ba, bg"<<endl;
    out_stream << fixed;

    vk::Timer timer;             //!< Stopwatch to measure time to process frame.
    timer.start();

    double time_frame(-1);                  //timestamp of current frame
    double time_pair[2]={-1,-1};              // timestamps of the previous and current images
    dataset_type experim = Tracker.experimDataset;

    int bRGB = fsSettings["Camera.RGB"];

    if(bRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if( experim == CrowdSourcedData){
        cv::VideoCapture cap(dir);

        double rate = cap.get(CV_CAP_PROP_FPS);
        if(!rate) cerr<<"Error opening video file "<<dir<<endl;
        cap.set(CV_CAP_PROP_POS_FRAMES, numImages); //start from numImages, 0 based index
        totalImages =std::min(totalImages, (int)cap.get(CV_CAP_PROP_FRAME_COUNT));
        int width= cap.get(CV_CAP_PROP_FRAME_WIDTH), height= cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        int downscale = vio::GetDownScale(width, height, 1280);

        Tracker.ResizeCameraModel(downscale);

        cv::Mat left_img, dst;
#ifdef SLAM_USE_ROS
        ros::Rate r(mFps);
        while(ros::ok()&& numImages<=totalImages)
#else
        while(numImages<=totalImages)
#endif
        {
            assert(cap.get(CV_CAP_PROP_POS_FRAMES) == numImages);
            time_frame= cap.get(CV_CAP_PROP_POS_MSEC)/1000.0;
            cap.read(left_img);

            while(left_img.empty()){ // this happens when a frame is missed
                ++numImages;
                if(numImages > totalImages){
                    left_img.release();
                    return 0;
                }
                assert(cap.get(CV_CAP_PROP_POS_FRAMES) == numImages);
                time_frame= cap.get(CV_CAP_PROP_POS_MSEC)/1000.0;
                cap.read(left_img);
            }

            if(downscale>1){
                cv::pyrDown(left_img, dst, cv::Size((width+1)/2, (height+1)/2));
                left_img= dst;
            }
            time_pair[0]=time_pair[1];
            time_pair[1]=time_frame;

            if(left_img.channels()==3)
            {
                cv::Mat temp;
                if(bRGB)
                    cvtColor(left_img, temp, CV_RGB2GRAY);
                else
                    cvtColor(left_img, temp, CV_BGR2GRAY);
                left_img=temp;
            }
            SLAM_LOG(time_frame);
//            SLAM_DEBUG_STREAM("processing frame "<< numImages);
            std::cout <<"processing frame "<< numImages <<"th of timestamp "<< time_frame<<std::endl;
            SLAM_START_TIMER("tot_time");

            Tracker.ProcessAMonocularFrame(left_img, time_frame);

            SLAM_STOP_TIMER("tot_time");

#ifdef SLAM_TRACE
            g_permon->writeToFile();
#endif

            ORB_SLAM::TrackingResult trackingResult;
            Tracker.GetLastestPoseEstimate(trackingResult);
            out_stream << trackingResult<<endl;

            ++numImages;

            framePublisher.Refresh();
            Tracker.CheckResetByPublishers();
#ifdef SLAM_USE_ROS
            mapPublisher->Refresh();
            r.sleep();
#endif
        }
    }else{
#ifdef SLAM_OUTPUT_VISO2
        std::size_t pos = output_file.find(".txt");
        string viso2_output_file= output_file.substr(0, pos) +"_viso2.txt";
        ofstream viso2_stream(viso2_output_file);
#endif

        cv::Mat left_img;
        cv::Mat right_img;
        string time_filename = slamhome + (std::string)fsSettings["time_file"]; //timestamps for frames

        StereoImageLoader sil(time_filename, experim, dir, strSettingsFile);
#ifdef SLAM_USE_ROS
        ros::Rate r(mFps);
        while(ros::ok()&& numImages<=totalImages)
#else
        while(numImages<=totalImages)
#endif
        {
            sil.GetTimeAndRectifiedStereoImages(time_frame, left_img, right_img, numImages);

            if(time_frame == -1.0){
                std::cout <<"Unable to grab time for image "<< numImages<< std::endl;
                break;
            }
            time_pair[0]=time_pair[1];
            time_pair[1]=time_frame;

            SLAM_LOG(time_frame);
//            SLAM_DEBUG_STREAM("processing frame "<< numImages);
            std::cout <<"processing frame "<< numImages <<" of timestamp "<< time_frame<<std::endl;
            SLAM_START_TIMER("tot_time");
#ifdef MONO
            Tracker.ProcessAMonocularFrame(left_img, time_frame);
#else
            Tracker.ProcessAStereoFrame(left_img, right_img, time_frame);
#endif
            SLAM_STOP_TIMER("tot_time");

#ifdef SLAM_TRACE
            g_permon->writeToFile();
#endif

            ORB_SLAM::TrackingResult trackingResult;
#ifdef SLAM_OUTPUT_VISO2
            Tracker.GetViso2PoseEstimate(trackingResult);
            viso2_stream<<trackingResult<<endl;
#endif
            Tracker.GetLastestPoseEstimate(trackingResult);
            out_stream<< trackingResult << endl;

            ++numImages;

            framePublisher.Refresh();
            Tracker.CheckResetByPublishers();
#ifdef SLAM_USE_ROS
            mapPublisher->Refresh();
            r.sleep();
#endif
        }
#ifdef SLAM_OUTPUT_VISO2
        viso2_stream.close();
#endif
    }

    //do we need to wait for loop closing
    while(LocalMapper.isStopped() || LocalMapper.stopRequested()){
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
    double calc_time =  timer.stop();
    double time_per_frame=calc_time/(numImages-nStartId+1);
    cout<<"Calc_time:"<<calc_time<<";"<<"time per frame:"<<time_per_frame<<endl; //do not use ROS_INFO because ros::shutdown may be already invoked

    vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

#if 0 //disable this section or change the output file so as to avoid dumping keyframe trajectory
      //into the file for frame trajectory estimated by the DWO algorithm
    cout<<"Saving Keyframe Trajectory to "<<output_file<<endl;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        Eigen::Vector3d t = pKF->GetCameraCenter();
        Eigen::Matrix<double,4,1> q = pKF->GetPose().unit_quaternion().conjugate().coeffs();
        out_stream << setprecision(6) << pKF->mTimeStamp << " " << t.transpose()<< " "
                     << q.transpose() <<" "<< pKF->speed_bias.transpose()<<endl;
    }
#endif

    out_stream.close();
    out_stream.open(output_point_file.c_str(), std::ios::out);
    out_stream<<"%Each row is point id, position xyz in world frame"<<endl;
    out_stream << fixed;
    vector<ORB_SLAM::MapPoint*> vpMPs = World.GetAllMapPoints();
    for(size_t i=0; i<vpMPs.size(); ++i)
    {
        ORB_SLAM::MapPoint* pMP = vpMPs[i];
        if(pMP->isBad())
            continue;
        Eigen::Vector3d t= pMP->GetWorldPos();
        out_stream << setprecision(6) << pMP->mnId<< " " << t.transpose()<<endl;
    }
    out_stream.close();
    cout<<"Saved MapPoints to "<<output_point_file<<endl;

    assert(!(LocalMapper.stopRequested() || LocalMapper.isStopped()));
#ifdef SLAM_USE_ROS
    ros::shutdown();
#else
    loopClosingThread.interrupt();// we cannot interrupt a thread that does not use boost::sleep()
    localMappingThread.interrupt();
#endif

    loopClosingThread.join();
    localMappingThread.join();
    return 0;
}
