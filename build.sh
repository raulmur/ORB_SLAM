# Example usage
# ./build.sh /opt/ros/kinetic/share/OpenCV-3.2.0-dev /usr/include/eigen3
OPENCV_CONFIG_PATH=$1
EIGEN3_INCLUDE_FOLDER=$2
if [[ -z $OPENCV_CONFIG_PATH ]] || [[ -z $EIGEN3_INCLUDE_FOLDER ]]; then
    echo "Usage: " $0 " OPENCV_CONFIG_PATH EIGEN3_INCLUDE_FOLDER"
    echo "Ex. " $0 " /opt/ros/kinetic/share/OpenCV-3.2.0-dev /usr/include/eigen3"
    exit -1
else
    echo "OPENCV_CONFIG_PATH " $OPENCV_CONFIG_PATH
    echo "EIGEN3_INCLUDE_FOLDER " $EIGEN3_INCLUDE_FOLDER
fi

build_type=Debug
FULL_PATH=`pwd`
INSTALL_FOLDER=$HOME/slam_devel
mkdir -p $HOME/slam_devel

cd Thirdparty
echo "Configuring and building Thirdparty/Sophus ..."
git clone https://github.com/stevenlovegrove/Sophus.git
cd Sophus
git checkout b474f05
#rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN3_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER -DCMAKE_INSTALL_PREFIX:PATH=$INSTALL_FOLDER/sophus
make -j4
make install

echo "Configuring and building Thirdparty/g2o ..."
cd ../..
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 46f615ca32bd98a0f8a7ab1cb429566c43000e98
#rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$INSTALL_FOLDER/g2o -DEIGEN3_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER
make -j4
make install

echo "Configuring and building Thirdparty/DBoW2 ..."
cd ../../DBoW2
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH
make -j

cd ../../libviso2

echo "Configuring and building Thirdparty/libviso2 ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH
make -j

cd ../../vikit_common

echo "Configuring and building Thirdparty/vikit_common ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH -DEIGEN_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER
make -j

cd ../../vio_common

echo "Configuring and building Thirdparty/vio_common ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH -DEIGEN_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER
make -j

cd ../..
echo "Configuring and building Thirdparty/vio_g2o ..."
git clone https://github.com/JzHuai0108/vio_g2o.git
cd vio_g2o
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH -DEIGEN_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER -DINSTALL_PREFIX=$INSTALL_FOLDER
make -j4

cd ../../..
# now in orbslam_dwo folder
echo "Configuring and building orbslam_dwo ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=$build_type -DOpenCV_DIR=$OPENCV_CONFIG_PATH -DEIGEN3_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER -DINSTALL_PREFIX=$INSTALL_FOLDER
make -j2

cd ../Thirdparty
#rm -rf Sophus
#rm -rf g2o
