# ORBSLAM_DWO

ORBSLAM_DWO is developed on top of ORB-SLAM with double window optimization by Jianzhu Huai. The major differences from ORB-SLAM are: (1) it can run with or without ROS, (2) it does not use the modified version of g2o shipped in ORB-SLAM, instead it uses the g2o from github, (3) it uses Eigen vectors and Sophus members instead of Opencv Mat to represent pose entities, (4) it incorporates the pinhole camera model from [rpg_vikit](https://github.com/uzh-rpg/rpg_vikit.git) and a decay velocity motion model from [Stereo PTAM](https://github.com/lrse/sptam.git), (5) currently, it supports monocular, stereo, and stereo + inertial input for SLAM, note it does not work with monocular + inertial input. 

ORB-SLAM is a versatile and accurate Monocular SLAM solution able to compute in real-time the camera trajectory and a sparse 3D reconstruction of the scene in a wide variety of environments, ranging from small hand-held sequences to a car driven around several city blocks. It is able to close large loops and perform global relocalisation in real-time and from wide baselines.

Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. Submitted to IEEE Transactions on Robotics. arXiv preprint: http://arxiv.org/abs/1502.00956

[2] Jianzhu Huai, Charles K. Toth and Dorota A. Grejner-Brzezinska: Stereo-inertial odometry using nonlinear optimization. Proceedings of the 28th International Technical Meeting of The Satellite Division of the Institute of Navigation (ION GNSS+ 2015) 2015.

#1. License

ORBSLAM_DWO is released under a GPLv3 license. Please note that we provide along ORB-SLAM a modified version of DBoW2 which is under BSD. 

If you use ORBSLAM_DWO in an academic work, please cite:

	@inproceedings{Huai2015stereo,
	title={Stereo-inertial odometry using nonlinear optimization},
	author={Huai, Jianzhu and Toth, Charles and Grejner-Brzezinska, Dorota},
	booktitle={Proceedings of the 28th International Technical Meeting of The Satellite Division of the Institute of Navigation (ION GNSS+ 2015)},
	year={2015},
	organization={ION}
	}

#2. Installing dependencies

The following installation procedure has been tested on Ubuntu 14.04 and 14.04.2.

##2.1 Boost
The Boost library is used to launch different threads of the SLAM system.
		sudo apt-get install libboost-all-dev 
##2.2 OpenCV 2.4.x
OpenCV is mainly used for feature extraction, and matching. It can be installed via: 
		sudo apt-get install libopencv-dev

##2.3 Eigen >=3.0
Eigen can be installed via
		sudo apt-get install libeigen3-dev
However, in Ubuntu 14.04, the above command may install Eigen 3.0 rather than newer versions. If a newer version is desired, you may build and install Eigen from the source. To do that, first download the proper source archive package from [here](http://eigen.tuxfamily.org/index.php?title=Main_Page), the following assuming Eigen 3.2.10 is downloaded into /home/username/ folder. Second, open a terminal and type

	tar xvjf eigen-eigen-b9cd8366d4e8.tar.bz2
	mv eigen-eigen-b9cd8366d4e8 eigen-3.2.10
	cd eigen-3.2.10
	mkdir build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ..
	make
	sudo make install

##2.4 Install ROS (optional) and download orbslam_dwo

To build the program with the ROS support, a ROS distribution (e.g. Indigo) needs to be installed. For now, the program is only tested with Indigo, but it should work with Hydro and Jade because it only relies on a few basic features of ROS, e.g. ros::rate, ROS_INFO(). After ROS is installed, a catkin workspace has to be set up using this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). From now on, let's suppose the workspace path is `~/home/username/catkin_ws`.

If the program is to be built without ROS support, you may simply create a folder to contain the source of ORBSLAM_DWO, let's assume that the folder is created such that its path is '~/home/username/catkin_ws/src'.

Regardless of the ROS support, download ORBSLAM_DWO into the workspace folder via:
		
		cd catkin_ws/src		
		git clone --recursive https://github.com/JzHuai0108/ORB_SLAM.git
		mv $HOME/catkin_ws/src/ORB_SLAM $HOME/catkin_ws/src/orbslam_dwo

The last command renames the downloaded folder into orbslam_dwo. That is the folder where the following dependencies are to be built and installed.

##2.5 g2o and its dependencies 
g2o is used to perform optimization tasks. Its dependencies(Eigen, CMAKE, SuiteSparse, QGLViewer) can be installed via:
		sudo apt-get install cmake libsuitesparse-dev libqglviewer-dev
Note this step assumes that Eigen is already installed.
To avoid system wide installation, g2o is recommended to be installed into a local folder, e.g., '~/home/username/g2o/local_install'. To do that, navigate to the /home/username/ folder in a terminal, and run the following commands:
		
		git clone https://github.com/RainerKuemmerle/g2o.git
		cd g2o
		mkdir local_install
		mkdir build
		cd build
		cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/g2o/local_install -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR:PATH=/usr/local/include/eigen3
		make -j4
		make install
		cd

##2.6 Sophus
The modified version of Sophus by Steven Lovegrove is used to manipulate Lie group members. The reason why Hauke Strasdat's version is not used is that Lovegrove's version is a header only library. More importantly, it complies with the jet numbers used in the autodiff module of the ceres solver. In this case, assume it is to be put in /home/username/Sophus. To do that, navigate to the /home/username folder in a terminal and run: 
		git clone https://github.com/stevenlovegrove/Sophus.git
		cd Sophus
		git checkout b474f0
		cmake ..
		make 

##2.7 vio_common, vio_g2o, DBoW2, libviso2, vikit (all included in /Thirdparty)
vio_common is a library of common functions to read videos, inertial data files, and to manipulate Eigen matrices.
	
vio_g2o extends g2o by defining additional (some even redundant) vertices, edges, parameters used in visual inertial optimization. vio_g2o depends on vio_common and apparently, g2o. In vio_g2o, the autodiff module of the [ceres solver](https://github.com/ceres-solver/ceres-solver.git) is used to compute Jacobians in the IMU constraint. Automatic differentiation is necessary because in the program the full exponential map is used to map se(3) to SE(3). If, however, a first order approximation is used, e.g., in [OKVIS](https://github.com/ethz-asl/okvis.git), analytic Jacobians can be derived. 

Interestingly, g2o also borrows the autodiff module of the ceres solver, the related files of autodiff are put into a folder, g2o/EXTERNAL/ceres. But these files in g2o/EXTERNAL/ceres have problems in compiling orbslam_dwo, possibly because they are quite old. So I extracted these files from a recent version of the ceres solver, and put them into the vio_g2o/include/g2o_EXTERNAL folder.

Some components of the DBoW2 library are used for place recognition and feature matching. A modified copy of the library by Raul Mur-Artal
including only the necessary components and some modifications listed in Thirdparty/DBoW2/LICENSE.txt is provided at Thirdparty/DBoW2.
Note it depends on OpenCV. 

libviso2 is used to detect matches between two pairs of stereo images, also called quad matches. These matches are used in ORBSLAM_DWO for tracking and triangulating features. Because a few changes are made, it is included in the /Thirdparty folder.

[rpg-vikit](https://github.com/uzh-rpg/rpg_vikit.git) is used to deal with camera models. It depended on Strasdat's version of Sophus. To make it work with Lovegrove's Sophus, SE3 is changed to SE3d, rotation_matrix() to rotationMatrix(), #include <sophus/se3.h> to #include <sophus/se3.hpp> in several of its files. Also, in rpg_vikit/vikit_common/CMakeLists.txt I set the flag USE_ROS to FALSE. For easy compilation, it is also included in the orbslam_dwo distribution. 

To build these five dependencies, navigate to the /orbslam_dwo folder in a terminal and run: 

		chmod +x ./build.sh
                ./build.sh

#3. Build ORBSLAM_DWO and test with KITTI seq 00 and Tsukuba CG stereo dataset

##3.1 stereo or stereo + inertial configuration
In orbslam_dwo/CMakeLists.txt make sure line "SET(MONO_SLAM FALSE)". Note whether to use IMU data is determined by the use_imu_data field in the yaml setting file. Then build the program either with or without ROS as follows.

###3.1.1 With ROS
In orbslam_dwo/CMakeLists.txt make sure line "SET(USE_ROS TRUE)".

		cd catkin_ws
		source devel/setup.bash		
		catkin_make -DCMAKE_BUILD_TYPE=Release --pkg orbslam_dwo
		make

To test the program, first launch roscore and related viewers for inspection via:

		launch ROS in a terminal: roscore
		launch image view in another terminal: rosrun image_view image_view image:=/ORBSLAM_DWO/Frame _autosize:=true
		launch rviz in another terminal: rosrun rviz rviz -d $HOME/catkin_ws/src/orbslam_dwo/data/rviz.rviz

Then depending on the data for tests, execute one of the following commands in another terminal. Note you may need to change the paths for data in the yaml setting file. The simulated IMU data for both KITTI seq 00 and Tsukuba dataset is included in the orbslam_dwo/data folder.

To test the program on KITTI seq 00 (stereo)
		rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo/kittiseq00.yaml

To test the program on Tsukuba dataset (stereo)
                rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo/tsukuba.yaml

To test the program on KITTI seq 00 with simulated inertial data (stereo + inertial)
		rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo_imu/kittiseq00_imu.yaml

To test the program on Tsukuba dataset with simulated inertial data (stereo + inertial)
		rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo_imu/tsukuba_imu.yaml

###3.1.2 Without ROS

In orbslam_dwo/CMakeLists.txt make sure line "SET(USE_ROS FALSE)", then in a terminal, execute:

		cd catkin_ws/src/orbslam_dwo
		mkdir build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release ..
		make

Then depending on the data for tests, execute one of the following commands in another terminal. Note you may need to change the paths for data in the yaml setting file. The simulated IMU data for both KITTI seq 00 and Tsukuba dataset is included in the orbslam_dwo/data folder.

To test the program on KITTI seq 00 (stereo)
		cd catkin_ws/src/orbslam_dwo/bin
		./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo/kittiseq00.yaml

To test the program on Tsukuba dataset (stereo)
		cd catkin_ws/src/orbslam_dwo/bin
                ./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo/tsukuba.yaml

To test the program on KITTI seq 00 with simulated inertial data (stereo + inertial)
		cd catkin_ws/src/orbslam_dwo/bin
		./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo_imu/kittiseq00_imu.yaml

To test the program on Tsukuba dataset with simulated inertial data (stereo + inertial)
		cd catkin_ws/src/orbslam_dwo/bin
		./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/settingfiles_stereo_imu/tsukuba_imu.yaml

#3.2 monocular configuration

In orbslam_dwo/CMakeLists.txt make sure line "SET(MONO_SLAM TRUE)". Note for now this program does not support monocular + inertial SLAM. Then build the program either with or without ROS as follows.

###3.2.1 With ROS
In orbslam_dwo/CMakeLists.txt make sure line "SET(USE_ROS TRUE)".

		cd catkin_ws
		source devel/setup.bash		
		catkin_make -DCMAKE_BUILD_TYPE=Release --pkg orbslam_dwo
		make

To test the program, first launch roscore and related viewers for inspection via:

		launch ROS in a terminal: roscore
		launch image view in another terminal: rosrun image_view image_view image:=/ORBSLAM_DWO/Frame _autosize:=true
		launch rviz in another terminal: rosrun rviz rviz -d $HOME/catkin_ws/src/orbslam_dwo/data/rviz.rviz

Then depending on the data for tests, execute one of the following commands in another terminal. Note you may need to change the paths for data in the yaml setting file.

To test the program on KITTI seq 00 (monocular)
		rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/setttingfiles_mono/kittiseq00_mono.yaml

To test the program on Tsukuba dataset (monocular)
                rosrun orbslam_dwo test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/setttingfiles_mono/tsukuba_mono.yaml

###3.2.2 Without ROS

In orbslam_dwo/CMakeLists.txt make sure line "SET(USE_ROS FALSE)", then in a terminal, execute:

		cd catkin_ws/src/orbslam_dwo
		mkdir build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release ..
		make

Then depending on the data for tests, execute one of the following commands in another terminal. Note you may need to change the paths for data in the yaml setting file. 

To test the program on KITTI seq 00 (monocular)

		cd catkin_ws/src/orbslam_dwo/bin
		./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/setttingfiles_mono/kittiseq00_mono.yaml

To test the program on Tsukuba dataset (monocular)
		cd catkin_ws/src/orbslam_dwo/bin
                ./test_orbslam $HOME/catkin_ws/src/orbslam_dwo/data/setttingfiles_mono/tsukuba_mono.yaml

#4. Failure modes

In general our stereo SLAM solution is expected to have a bad time in the following situations:
- Pure rotations and features are very distant in exploration
- Low texture environments
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

#5. Known issues

##5.1 Memory drain

In testing on KITTI seq 00 with either stereo + inertial or stereo configuration, the computer memory is slowly chipped away. Finally this blocks loop closure and chokes the tracking thread. Empricially, the maximum number of accommodated keyframes is around 1200 with 8GB RAM. But this issue is not observed when testing ORBSLAM_DWO with the monocular configuration on KITTI seq 00. 

I suspect it is caused by too many large keyframes stored in memory. The Frame::PartialRelease function is used to release part of the frame before it is copied into a keyframe, but this does not solve the memory drain. One approach to circumvent this issue is to discourage frequent creation of keyframes by reducing the value of Tracking.min_tracked_features in the setting file. For KITTI seq 00, I set it to 120 instead of its default value, 200. Another tactic to alleviate this issue is to stick with shared pointers, e.g., std::shared_ptr, when dealing with keyframes and map points. This way, invalid map points and culled keyframes may get released from RAM and free up some space. This remains future work.

##5.2 Result is not robust/stable

Because there is some non-deterministic behavior in the program, it may give good results only 6 out of 10 times. It is also observed that when the program is fed with data at a high frequency, its performance degrades compared to processing the same data with a lower frequency.

If you have any trouble installing or running ORBSLAM_DWO, contact the authors.


