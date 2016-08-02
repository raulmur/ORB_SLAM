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

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include "Converter.h"


using namespace std;

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

    if(argc != 2)
    {
        cerr << endl << "Usage: executable_name path_to_settings (absolute or relative to package directory)" << endl;
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
    ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
    string strVocFile = fsSettings["voc_file_path"];   //            ros::package::getPath("ORB_SLAM")+"/"+argv[1];
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

    FramePub.SetMap(&World);
#ifdef SLAM_USE_ROS
    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);
    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
#else
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, /*&MapPub,*/ &World, strSettingsFile);
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

	Tracker.Run();    
    loopClosingThread.join();
    localMappingThread.join();

    assert(!(LocalMapper.stopRequested() || LocalMapper.isStopped()));
#ifdef SLAM_USE_ROS
    ros::shutdown();
#else
    loopClosingThread.interrupt();// we cannot interrupt a thread that does not use boost::sleep()
    localMappingThread.interrupt();
#endif

    return 0;
}
