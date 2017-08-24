/*
Copyright 2012. All rights reserved.
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

#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>
#include "p_match.h"
#include "matrix.h"
namespace libviso2{
//copied from fast_score.hpp of opencv
void makeOffsets(int pixel[25], int row_stride, int patternSize);

template<int patternSize>
int cornerScore(const unsigned char* ptr, const int pixel[], int threshold);

class Matcher {

public:

  // parameter settings
  struct parameters {
  
    int32_t nms_n;                  // non-max-suppression: min. distance between maxima (in pixels)
    int32_t nms_tau;                // non-max-suppression: interest point peakiness threshold
    int32_t match_binsize;          // matching bin width/height (affects efficiency only)
    int32_t match_radius;           // matching radius (du/dv in pixels)
    int32_t match_disp_tolerance;   // dv tolerance for stereo matches (in pixels)
    int32_t outlier_disp_tolerance; // outlier removal: disparity tolerance (in pixels)
    int32_t outlier_flow_tolerance; // outlier removal: flow tolerance (in pixels)
    int32_t multi_stage;            // 0=disabled,1=multistage matching (denser and faster)
    int32_t half_resolution;        // 0=disabled,1=match at half resolution, refine at full resolution
    int32_t refinement;             // refinement (0=none,1=pixel,2=subpixel)
    double  f,cu,cv,base;           // calibration (only for match prediction)
    
    // default settings
    parameters () {
      nms_n                  = 3;
      nms_tau                = 50;
      match_binsize          = 50;
      match_radius           = 200;
      match_disp_tolerance   = 2;
      outlier_disp_tolerance = 5;
      outlier_flow_tolerance = 5;
      multi_stage            = 1;
      half_resolution        = 1;
      refinement             = 1;
    }    
  };

  // constructor (with default parameters)
  Matcher(parameters param);

  // deconstructor
  ~Matcher();
  
  // intrinsics
  void setIntrinsics(double f,double cu,double cv,double base) {
    param.f = f;
    param.cu = cu;
    param.cv = cv;
    param.base = base;
  }


  // computes features from left/right images and pushes them back to a ringbuffer,
  // which interally stores the features of the current and previous image pair
  // use this function for stereo or quad matching
  // input: I1,I2 .......... pointers to left and right image (row-aligned), range [0..255]
  //        dims[0,1] ...... image width and height (both images must be rectified and of same size)
  //        dims[2] ........ bytes per line (often equals width)
  //        replace ........ if this flag is set, the current image is overwritten with
  //                         the input images, otherwise the current image is first copied
  //                         to the previous image (ring buffer functionality, descriptors need
  //                         to be computed only once)    
  void pushBack (uint8_t *I1,uint8_t* I2,int32_t* dims,const bool replace);
  
  // computes features from a single image and pushes it back to a ringbuffer,
  // which interally stores the features of the current and previous image pair
  // use this function for flow computation
  // parameter description see above
  void pushBack (uint8_t *I1,int32_t* dims,const bool replace) { pushBack(I1,0,dims,replace); }

  // match features currently stored in ring buffer (current and previous frame)
  // input: method ... 0 = flow, 1 = stereo, 2 = quad matching
  //        Tr_delta: uses motion from previous frame to better search for
  //                  matches, if specified
  void matchFeatures(int32_t method, const Matrix *Tr_delta = 0);

  // feature bucketing: keeps only max_features per bucket, where the domain
  // is split into buckets of size (bucket_width,bucket_height)
  void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

  // return vector with matched feature points and indices
  std::vector<p_match> getMatches() { return p_matched_2; }

  // given a vector of inliers computes gain factor between the current and
  // the previous frame. this function is useful if you want to reconstruct 3d
  // and you want to cancel the change of (unknown) camera gain.
  float getGain (std::vector<int32_t> inliers);

  //refine feature positions with FAST corner score in case of half_resolution,
  // choose the best pixel from 5 connected pixels centered on current pixel
  void refineFeatures(std::vector<p_match> & vMatches);
  int32_t getNumDenseFeatures(bool left)
  {
      if(left)
          return n1c2;
      else
          return n2c2;
  }
  int32_t getNumSparseFeatures(bool left)
  {
      if(left)
          return n1c1;
      else
          return n2c1;
  }
private:

  // structure for storing interest points
  struct maximum {
    int32_t u;   // u-coordinate
    int32_t v;   // v-coordinate
    int32_t val; // blob filter or checkerboard filter quantitized response
    int32_t c;   // class (0 blob min, 1 blob max, 2 corner min, 3 corner max)
    int32_t d1,d2,d3,d4,d5,d6,d7,d8; // descriptor (16 bytes horizontal and 16 bytes vertical Sobel responses intertwined)
    maximum() {}
    maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c) {}
  };
  
  // u/v ranges for matching stage 0-3
  struct range {
    float u_min[4];
    float u_max[4];
    float v_min[4];
    float v_max[4];
  };
  
  struct delta {
    float val[8];
    delta () {}
    delta (float v) {
      for (int32_t i=0; i<8; i++)
        val[i] = v;
    }
  };
  
  // computes the address offset for coordinates u,v of an image of given width
  inline int32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
    return v*width+u;
  }

  // Alexander Neubeck and Luc Van Gool: Efficient Non-Maximum Suppression, ICPR'06, algorithm 4
  void nonMaximumSuppression (int16_t* I_f1,int16_t* I_f2,const int32_t* dims,std::vector<Matcher::maximum> &maxima,int32_t nms_n);

  // descriptor functions
  inline uint8_t saturate(int16_t in);
  void filterImageAll (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,int16_t* I_f1,int16_t* I_f2,const int* dims);
  void filterImageSobel (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,const int* dims);
  inline void computeDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
  inline void computeSmallDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
  void computeDescriptors (uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<Matcher::maximum> &maxima);
  
  void getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half);
  uint8_t* createHalfResolutionImage(uint8_t *I,const int32_t* dims);

  // compute sparse set of features from image
  // inputs:  I ........ image
  //          dims ..... image dimensions [width,height]
  //          n ........ non-max neighborhood
  //          tau ...... non-max threshold
  // outputs: max ...... vector with maxima [u,v,value,class,descriptor (128 bits)]
  //          I_du ..... gradient in horizontal direction
  //          I_dv ..... gradient in vertical direction
  // WARNING: max,I_du,I_dv has to be freed by yourself!
  // max1 sparse descriptors, max2 dense descriptors, each 16byte aligned memory, descriptor is acctually 4x8x8=256 bits
  void computeFeatures (uint8_t *I,const int32_t* dims,int32_t* &max1,int32_t &num1,int32_t* &max2,int32_t &num2,uint8_t* &I_du,uint8_t* &I_dv,uint8_t* &I_du_full,uint8_t* &I_dv_full);

  // matching functions
  void computePriorStatistics (std::vector<p_match> &p_matched,int32_t method);
  void createIndexVector (int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num);
  // u_, v_ predicted feature position
  inline void findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                         std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                         int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);
  void matching (int32_t *m1p,int32_t *m2p,int32_t *m1c,int32_t *m2c,
                 int32_t n1p,int32_t n2p,int32_t n1c,int32_t n2c,
                 std::vector<p_match> &p_matched,int32_t method,bool use_prior,const Matrix *Tr_delta = 0);

  // outlier removal
  void removeOutliers (std::vector<p_match> &p_matched,int32_t method);

  // parabolic fitting
  bool parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                        const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                        const float &u1,const float &v1,
                        float       &u2,float       &v2,
                        Matrix At,Matrix AtA,
                        uint8_t* desc_buffer);
  void relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                       const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                       const float &u1,const float &v1,
                       float       &u2,float       &v2,
                       uint8_t* desc_buffer);
  void refinement (std::vector<p_match> &p_matched,int32_t method);

  // mean for gain computation
  inline float mean(const uint8_t* I,const int32_t &bpl,const int32_t &u_min,const int32_t &u_max,const int32_t &v_min,const int32_t &v_max);

  // parameters
  parameters param;
  int32_t    margin;
  
  int32_t *m1p1,*m2p1,*m1c1,*m2c1; //maxima left previous sparse, maxima right previous sparse, ...
  int32_t *m1p2,*m2p2,*m1c2,*m2c2; // maxima left previous dense
  int32_t n1p1,n2p1,n1c1,n2c1;
  int32_t n1p2,n2p2,n1c2,n2c2;
  uint8_t *I1p,*I2p,*I1c,*I2c;
  uint8_t *I1p_du,*I2p_du,*I1c_du,*I2c_du;
  uint8_t *I1p_dv,*I2p_dv,*I1c_dv,*I2c_dv;
  uint8_t *I1p_du_full,*I2p_du_full,*I1c_du_full,*I2c_du_full; // only needed for
  uint8_t *I1p_dv_full,*I2p_dv_full,*I1c_dv_full,*I2c_dv_full; // half-res matching
  int32_t dims_p[3],dims_c[3];

  std::vector<p_match> p_matched_1;
  std::vector<p_match> p_matched_2;
  std::vector<Matcher::range>   ranges;
};
}
#endif

