echo "Configuring and building Thirdparty/DBoW2 ..."

FULL_PATH=`pwd`
OPENCV_CONFIG_PATH=/usr/local/share/OpenCV
EIGEN3_INCLUDE_FOLDER=/usr/local/include/eigen3

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH
make -j

cd ../../libviso2

echo "Configuring and building Thirdparty/libviso2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH
make -j

cd ../../vikit_common

echo "Configuring and building Thirdparty/vikit_common ..."

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

cd ../../vio_g2o

echo "Configuring and building Thirdparty/vio_g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=$OPENCV_CONFIG_PATH -DEIGEN_INCLUDE_DIR:PATH=$EIGEN3_INCLUDE_FOLDER -DG2O_INSTALL_PREFIX=$FULL_PATH/Thirdparty/g2o/local_install
make -j
