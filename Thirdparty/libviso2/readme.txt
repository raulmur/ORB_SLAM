####################################################################################
# Copyright 2012. All rights reserved.                                             #
# Institute of Measurement and Control Systems                                     #
# Karlsruhe Institute of Technology, Germany                                       #
#                                                                                  #
# This file is part of libviso2.                                                   #
# Authors:  Andreas Geiger                                                         #
#           Please send any bugreports to geiger@kit.edu                           #
#                                                                                  #
# libviso2 is free software; you can redistribute it and/or modify it under the    #
# terms of the GNU General Public License as published by the Free Software        #
# Foundation; either version 2 of the License, or any later version.               #
#                                                                                  #
# libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY      #
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A  #
# PARTICULAR PURPOSE. See the GNU General Public License for more details.         #
#                                                                                  #
# You should have received a copy of the GNU General Public License along with     #
# libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin       #
# Street, Fifth Floor, Boston, MA 02110-1301, USA                                  #
####################################################################################

+++++++++++++++++++++++++++++++++++
+          INTRODUCTION           +
+++++++++++++++++++++++++++++++++++

Libviso 2 (LIBrary for ViSual Odometry 2) is a cross-platfrom (Linux, Windows) C++
library with MATLAB wrappers for computing the 6 DOF motion of a moving stereo
camera. Input is a sequence of rectified stereo images. Output is a 4x4 matrix
which projects a point from the previous to the current camera coordinates.

Version 2 now also supports visual odometry from monocular sequences, and the
feature matching functions implement sparse stereo, scene flow and optical flow.
For more details, please have a look at the corresponding publication.

If you distribute a software that uses libviso, you have to distribute it under GPL
with the source code. Another option is to contact us to purchase a commercial license.

If you find this software useful or if you use this software for your research,
we would be happy if you cite the following related publication:

@INPROCEEDINGS{Geiger11,
 author = {Andreas Geiger and Julius Ziegler and Christoph Stiller},
 title = {StereoScan: Dense 3d Reconstruction in Real-time},
 booktitle = {IEEE Intelligent Vehicles Symposium},
 year = {2011},
 month = {June},
 address = {Baden-Baden, Germany}
}

+++++++++++++++++++++++++++++++++++
+    COMPILING MATLAB WRAPPERS    +
+++++++++++++++++++++++++++++++++++

Prerequisites needed for compiling libviso2:
- Matlab (if you want to compile the matlab wrappers)
- gcc or Visual Studio (if you want to include the code in your C++ project)

If you want to use libviso directly from MATLAB you can easily do this by using
the MATLAB wrappers provided. They also include some demo files for testing.

In the MATLAB directory of libviso, simply run 'make.m' to generate the mex wrappers.
(Run mex -setup before to choose your desired compiler)

Now try to run the demo*.m files!
For some of them you will need the Karlsruhe dataset from www.cvlibs.net.

+++++++++++++++++++++++++++++++++++
+     BUILDING A C++ LIBRARY      +
+++++++++++++++++++++++++++++++++++

Prerequisites needed for compiling and running the libviso2 demonstration
tool via c++:

- libpng (available at: http://www.libpng.org/pub/png/libpng.html)
- libpng++ (available at: http://www.nongnu.org/pngpp/)
- sequence 2010_03_09_drive_0019 (available at: http://www.cvlibs.net/)

libpng and png++ are needed for reading the png input images. On a ubuntu
box you can get them via apt:

- sudo apt-get install libpng12-dev
- sudo apt-get install libpng++-dev

Linux:

1) Move to libviso2 root directory
2) Type 'cmake .'
3) Type 'make'
4) Run './viso2 path/to/sequence/2010_03_09_drive_0019'

Windows:

1) Modify CMakefile according to your needs (libpng(++) must be found)
2) Start CMake GUI
3) Set directories to libviso2 root directory
4) Run configure, configure and generate
5) Open the resulting Visual Studio solution with Visual Studio
6) Switch to 'Release' mode and build all
9) Run 'viso2.exe path/to/sequence/2010_03_09_drive_0019'

For more information on CMake, have a look at the CMake documentation.

For more information on the usage of the library, have a look into the
MATLAB wrappers, demo.cpp, viso.h, viso_stereo.h, viso_mono.h and
matcher.h.

Please send any feedback and bugreports to geiger@kit.edu
Andreas Geiger

+++++++++++++++++++++++++++++++++++
+        STEREOMAPPER.ZIP         +
+++++++++++++++++++++++++++++++++++

Due to many requests, we have also included a zip file with the sources
of the full stereomapper application as presented in the paper, which allows
for dense reconstruction in real-time. It depends on Qt, libviso2 for visual
odometry and libelas for stereo estimation. libelas is available from:
www.cvlibs.net. Note that this code has not been refactored, is very weakly
commented and comes with absolutely no support. In order to get it running,
you need to adjust several hard coded paths and maybe fix some OS specific
components. We won't be able to provide any feedback or support for this
piece of software, but maybe you still find it useful ..

Best,
Andreas Geiger

