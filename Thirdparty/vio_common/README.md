vio_common

# Build instructions

1. Install opencv 2.4 and eigen 3.3

```
#e.g., 
sudo apt-get install libopencv-dev libeigen3-dev
```

If you want to install a specific version of these libraries or install to a specific path(note -DCMAKE\_INSTALL\_PREFIX), you may build them from source.

2. Build vio_common

```
cd 
cd workspace
git clone https://github.com/JzHuai0108/vio_common.git vio_common
mkdir build && cd build
#cmake .. -DOpenCV_DIR=/folder/of/OpenCVConfig.cmake -DEIGEN_INCLUDE_DIR=/folder/of/Eigen -DCMAKE_INSTALL_PREFIX=$HOME/workspace/local_install/vio_common
#e.g.,
cmake .. -DOpenCV_DIR=$HOME/workspace/local_install/opencv/share/OpenCV -DEIGEN_INCLUDE_DIR=$HOME/workspace/local_install/eigen -DCMAKE_INSTALL_PREFIX=$HOME/workspace/local_install/vio_common
make
make install
```


