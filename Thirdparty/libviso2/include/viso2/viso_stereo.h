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

#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"
#include <opencv2/core/core.hpp> //cv::Mat
namespace libviso2{
class VisualOdometryStereo : public VisualOdometry {

public:

  // stereo-specific parameters (mandatory: base)
  struct parameters : public VisualOdometry::parameters {
    double  base;             // baseline (meters)
    int32_t ransac_iters;     // number of RANSAC iterations
    double  inlier_threshold; // fundamental matrix inlier threshold
    bool    reweighting;      // lower border weights (more robust to calibration errors)
    parameters () {
      base             = 1.0;
      ransac_iters     = 200;
      inlier_threshold = 2.0;
      reweighting      = true;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryStereo (parameters param = parameters() );
  
  // deconstructor
  ~VisualOdometryStereo ();
  
  // process a new images, push the images back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
  //         I2 ........ pointer to rectified right image (uint8, row-aligned)
  //         dims[0] ... width of I1 and I2 (both must be of same size)
  //         dims[1] ... height of I1 and I2 (both must be of same size)
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current images with I1 and I2, without copying last current
  //                     images to previous images internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if an error occured
  bool process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace=false);

  const parameters & getParameters()const {return param;}
  void setParameters(const parameters & rhs) {param=rhs;}
  using VisualOdometry::process;


public:
  std::vector<double>  estimateMotion(const std::vector<p_match> & p_matched,
                                       const std::vector<double> tr_delta= std::vector<double>(6,0));
  bool estimateMotion5Point (const std::vector<p_match>& p_matched,
                             const std::vector<double> tr_delta_init= std::vector<double>(3,0));
  bool estimateMotionKlein (const std::vector<p_match>& p_matched, const std::vector<std::vector<float> >&mean_features);
  void getAllInlier(const std::vector<p_match> &p_matched,const Matrix & tr, std::vector<bool>& vInliers);
  std::vector<double>  unprojectPoint(double u1, double v1, double u2)
  {
      double d = std::max(u1 - u2,0.0001);
      std::vector<double> v3XYZ;
      v3XYZ.resize(3);
      v3XYZ[0] = (u1-param.calib.cu)*param.base/d;
      v3XYZ[1] = (v1-param.calib.cv)*param.base/d;
      v3XYZ[2] = param.calib.f*param.base/d;
      return v3XYZ;
  }

private:
  enum                 result { UPDATED, FAILED, CONVERGED };
  result               updateParameters(const std::vector<p_match> &p_matched,const std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
  result               updateParameters2(const std::vector<p_match> &p_matched,const std::vector<int32_t> &active,
                                         const cv::Mat Rf2s, std::vector<double> &tr,double step_size,double eps);
  void                 computeObservations(const std::vector<p_match> &p_matched,const std::vector<int32_t> &active);
  void                 computeResidualsAndJacobian(const std::vector<double> &tr,const std::vector<int32_t> &active);
  void                 computeResidualsAndJacobian2(const std::vector<double> &tr,const std::vector<int32_t> &active, const cv::Mat Rf2s);
  std::vector<int32_t> getInlier(const std::vector<p_match> &p_matched, const std::vector<double> &tr);
  std::vector<int32_t> getInlier2(const std::vector<p_match> &p_matched,const std::vector<double> &tr, const cv::Mat Rf2s);

  double *X,*Y,*Z;    // 3d points
  double *p_residual; // residuals (p_residual=p_observe-p_predict)

  // parameters
  parameters param;
};
}
#endif // VISO_STEREO_H

