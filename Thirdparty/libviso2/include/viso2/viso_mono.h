/*
Copyright 2011. All rights reserved.
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

#ifndef VISO_MONO_H
#define VISO_MONO_H

#include "viso.h"
namespace libviso2 {

class VisualOdometryMono : public VisualOdometry {

public:

  // monocular-specific parameters (mandatory: height,pitch)
  struct parameters : public VisualOdometry::parameters {
    double                      height;           // camera height above ground (meters)
    double                      pitch;            // camera pitch (rad, negative=pointing down)
    int32_t                     ransac_iters;     // number of RANSAC iterations
    double                      inlier_threshold; // fundamental matrix inlier threshold
    double                      motion_threshold; // directly return false on small motions
    parameters () {
      height           = 1.0;
      pitch            = 0.0;
      ransac_iters     = 2000;
      inlier_threshold = 0.00001;
      motion_threshold = 100.0;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryMono (parameters param);
  
  // deconstructor
  ~VisualOdometryMono ();
  
  // process a new image, pushs the image back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I ......... pointer to rectified image (uint8, row-aligned)
  //         dims[0] ... width of I
  //         dims[1] ... height of I
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current image with I, without copying last current
  //                     image to previous image internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if motion too small or an error occured
  bool process (uint8_t *I,int32_t* dims,bool replace=false);
#ifdef USE_OPENCV
  //changed update motion method w.r.t process(), method 0 opencv 5 point algorithm, 1, viso2 monocular algorithm
  bool process2 (uint8_t *I,int32_t* dims,bool replace=false, bool bUseViso2=false);
  // use opencv five point algorithm to estimate monocular motion, method 0 opencv 5 point algorithm, 1, viso2 monocular algorithm
  std::vector<double> estimateMotion2 (const std::vector<p_match> &p_matched,
                                                     bool bUseViso2= false);
#endif
private:

  template<class T> struct idx_cmp {
    idx_cmp(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const { return arr[a] < arr[b]; }
    const T arr;
  };  
public:
  std::vector<double>  estimateMotion (const std::vector<p_match> &p_matched,
                                       const std::vector<double> tr_delta= std::vector<double>(6,0));
private:
  Matrix               smallerThanMedian (Matrix &X,double &median);
  bool                 normalizeFeaturePoints (std::vector<p_match> &p_matched,Matrix &Tp,Matrix &Tc);
  void                 fundamentalMatrix (const std::vector<p_match> &p_matched,const std::vector<int32_t> &active,Matrix &F);
  void                 EtoRt(Matrix &E,Matrix &K,const std::vector<p_match> &p_matched,Matrix &X,Matrix &R,Matrix &t);
  int32_t              triangulateChieral (const std::vector<p_match> &p_matched,Matrix &K,Matrix &R,Matrix &t,Matrix &X);
  std::vector<int32_t> getInlier (const std::vector<p_match> &p_matched,Matrix &F);
  
  // parameters
  parameters param;  
};
}
#endif // VISO_MONO_H

