# ORB-SLAM-DWO

ORB-SLAM-DWO is an adapted version of ORB-SLAM with double window optimization by Jianzhu Huai. In contrast to ORB-SLAM, (1) it does not rely on ROS, (2) it does not use the modified version of g2o shipped in ORB-SLAM, instead it used the g2o from github, (3) it used Eigen vectors and Sophus members instead of Opencv Mat to represent pose entities, and (4) it also incorporate a camera model from vikit of SVO and a motion model of Stereo PTAM for KITTI.

ORB-SLAM is a versatile and accurate Monocular SLAM solution able to compute in real-time the camera trajectory and a sparse 3D reconstruction of the scene in a wide variety of environments, ranging from small hand-held sequences to a car driven around several city blocks. It is able to close large loops and perform global relocalisation in real-time and from wide baselines.

Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. Submitted to IEEE Transactions on Robotics. arXiv preprint: http://arxiv.org/abs/1502.00956
[2] Jianzhu Huai, Charles K. Toth and Dorota A. Grejner-Brzezinska: Stereo-inertial odometry using nonlinear optimization. Proceedings of the 27th International Technical Meeting of The Satellite Division of the Institute of Navigation (ION GNSS+ 2015) 2015.

#1. License

ORB-SLAM is released under a GPLv3 license. Please note that we provide along ORB-SLAM a modified version of g2o and DBoW2 which are both BSD. 

For a closed-source version of ORB-SLAM for commercial purposes, please contact the authors. 

If you use ORB-SLAM in an academic work, please cite:

	@article{murSubTro2015,
	  title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
	  author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
	  journal={Submitted to IEEE Transaction on Robotics. arXiv preprint arXiv:1502.00956},
	  year={2015}
	}

I tested the following installation procedure on Ubuntu 14.04 and 14.04.2 with Qt5
#2. Prerequisites (dependencies)

##2.1 Boost
We use the Boost library to launch the different threads of our SLAM system.

	sudo apt-get install libboost-all-dev 

##2.2 g2o and its dependencies 
We use g2o to perform several optimizations. I recommend installing g2o on a local folder, say $HOME/svslocal
	mkdir $HOME/svslocal
To install g2o dependencies(Eigen, CMAKE, SuiteSparse, QGLViewer), in terminal:
	sudo apt-get install cmake libeigen3-dev libsuitesparse-dev libqglviewer-dev
To install g2o, in terminal:
	cd ~
	git clone https://github.com/RainerKuemmerle/g2o.git
	cd g2o
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/svslocal -DCMAKE_BUILD_TYPE=Release
	make -j4
	make install
	cd ~
##2.3 OpenCV 2.4.10 tested
You may install openCV following instructions https://help.ubuntu.com/community/OpenCV. Using a shell file greatly simplifies its installation.

##2.4 DBoW2 (included)
We make use of some components of the DBoW2 library for place recognition and feature matching. We include a modified copy of the library
including only the components we need and also some modifications that are listed in Thirdparty/DBoW2/LICENSE.txt. 
It only depends on OpenCV, but it should be included in the distribution. Its installation will be detailed later on.

##2.5 Sophus
We used the modified version of Sophus by Steven Lovegrove to manipulate lie group members. The reason we do not use Hauke Strasdat's version is that Lovegrove's version only needs to include headers for use. And it also complies with Jet numbers used in Ceres Solver's automatic differentiation.
To install it, in terminal
	git clone https://github.com/stevenlovegrove/Sophus.git
	cd Sophus
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/svslocal
	make -j4
	make install
	cd ~

##2.6 vikit
Drawing lessons from SVO, vikit is used to deal with camera models. 
	git clone https://github.com/uzh-rpg/rpg_vikit.git
It depends on Strasdat's version of Sophus. To make it work with Lovegrove's Sophus, you need to change SE3 to SE3d, rotation_matrix() to rotationMatrix(), #include <sophus/se3.h> to #include <sophus/se3.hpp> in several files. Then, in rpg_vikit/vikit_common/CMakeLists.txt set the flag USE_ROS to FALSE.
To install it, in terminal,
	cd rpg_vikit/vikit_common
	mkdir build
	cd build
	cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/svslocal
	make
For vikit, it is not necessary to install it. But you need to set its path in the CMakeLists.txt of ORB-SLAM-DWO.
#3. Installation

1. Make sure you have installed all library dependencies.

2. Clone this ORB_SLAM into your project folder

3. Build DBoW2. Go into Thirdparty/DBoW2/ and execute:

		mkdir build
		cd build
		cmake .. -DCMAKE_BUILD_TYPE=Release
		make  

	Tip: Set your favorite compilation flags in line 4 and 5 of Thirdparty/DBoW2/CMakeLists.txt
		  (by default -03 -march=native)

4. Build ORB_SLAM. In the ORB-SLAM-DWO root execute:

		mkdir build
		cd build
		cmake .. -DCMAKE_BUILD_TYPE=Release
		make

	Tip: Set your favorite compilation flags in line 12 and 13 of Thirdparty/DBoW2/CMakeLists.txt
		  (by default -03 -march=native)

#4. Usage

1. Launch ORB-SLAM from the terminal:

		ORB_SLAM PATH_TO_SETTINGS_FILE

You have to provide the path to the settings file which contains path to the vocabulary file. The paths must be absolute or relative to the ORB_SLAM directory.  
We already provide the vocabulary file we use in ORB_SLAM/Data. Uncompress the file, as it will be loaded much faster.


2. The Settings File

We provide the settings of example sequences in Data folder, KITTI odometry and Tsukuba CG Stereo dataset daylight.
ORB_SLAM_DWO reads the camera calibration and setting parameters from a YAML file. We provide an example in Data/Tsukuba.yaml, where you will find all parameters and their description. We use the camera calibration model of OpenCV.

Please make sure you write and call your own settings file for your camera (copy the example file and modify the calibration)

#5. Failure Modes

In general our stereo SLAM solution is expected to have a bad time in the following situations:
- Pure rotations and features are very distant in exploration
- Low texture environments
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

#6. Need Help?

If you have any trouble installing or running ORB-SLAM, contact the authors.


