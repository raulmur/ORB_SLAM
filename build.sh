echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j



cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir local_install
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=$HOME/catkin_ws/src/orbslam_dwo/Thirdparty/g2o/local_install -DCMAKE_BUILD_TYPE=Release
make -j4
make install

cd ../../libviso2

echo "Configuring and building Thirdparty/libviso2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../vikit_common

echo "Configuring and building Thirdparty/vikit_common ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
