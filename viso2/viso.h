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

#ifndef VISO_H
#define VISO_H

#include "matrix.h"
#include "matcher.h"
namespace libviso2 {

class VisualOdometry {

public:

  // camera parameters (all are mandatory / need to be supplied)
  struct calibration {  
    double f;  // focal length (in pixels)
    double cu; // principal point (u-coordinate)
    double cv; // principal point (v-coordinate)
    calibration () {
      f  = 1;
      cu = 0;
      cv = 0;
    }
  };
  
  // bucketing parameters
  struct bucketing {  
    int32_t max_features;  // maximal number of features per bucket 
    double  bucket_width;  // width of bucket
    double  bucket_height; // height of bucket
    bucketing () {
      max_features  = 2;
      bucket_width  = 50;
      bucket_height = 50;
    }
  };
  
  // general parameters
  struct parameters {
    Matcher::parameters         match;            // matching parameters
    VisualOdometry::bucketing   bucket;           // bucketing parameters
    VisualOdometry::calibration calib;            // camera calibration parameters
  };

  // constructor, takes as input a parameter structure:
  // do not instanciate this class directly, instanciate VisualOdometryMono
  // or VisualOdometryStereo instead!
  VisualOdometry (parameters param);
  
  // deconstructor
  ~VisualOdometry ();

  // call this function instead of the specialized ones, if you already have
  // feature matches, and simply want to compute visual odometry from them, without
  // using the internal matching functions.
  bool process (std::vector<p_match> p_matched_) {
    p_matched = p_matched_;
    return updateMotion();
  }

  // returns transformation from previous to current coordinates as a 4x4
  // homogeneous transformation matrix Tr_delta, with the following semantics:
  // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
  // at time t_1 and maps it to the camera coordinate system at time t.
  // note: getMotion() returns the last transformation even when process()
  // has failed. this is useful if you wish to linearly extrapolate occasional
  // frames for which no correspondences have been found
  Matrix getMotion () { return Tr_delta; }
  //column major data from Eigen::Matrix4d
  void setMotion (double *data) {
      Matrix& Tr=Tr_delta;
      Tr.val[0][0] = data[0]; Tr.val[0][1] = data[4];  Tr.val[0][2] = data[8];  Tr.val[0][3] = data[12];
      Tr.val[1][0] = data[1]; Tr.val[1][1] = data[5];  Tr.val[1][2] = data[9];  Tr.val[1][3] = data[13];
      Tr.val[2][0] = data[2]; Tr.val[2][1] = data[6];  Tr.val[2][2] = data[10]; Tr.val[2][3] = data[14];
      Tr.val[3][0] = data[3]; Tr.val[3][1] = data[7];  Tr.val[3][2] = data[11]; Tr.val[3][3] = data[15];
      Tr_valid = true;
  }
  // returns previous to current feature matches from internal matcher
  std::vector<p_match> getMatches () { return matcher->getMatches(); }
  
  // returns the number of successfully matched points, after bucketing
  int32_t getNumberOfMatches () { return p_matched.size(); }
  
  // returns the number of inliers: num_inliers <= num_matched
  int32_t getNumberOfInliers () { return inliers.size(); }
    
  // returns the indices of all inliers
  std::vector<int32_t> getInlierIndices () { return inliers; }
  
  // given a vector of inliers computes gain factor between the current and
  // the previous frame. this function is useful if you want to reconstruct 3d
  // and you want to cancel the change of (unknown) camera gain.
  float getGain (std::vector<int32_t> inliers_) { return matcher->getGain(inliers_); }

  // streams out the current transformation matrix Tr_delta 
  friend std::ostream& operator<< (std::ostream &os,VisualOdometry &viso) {
    Matrix p = viso.getMotion();
    os << p.val[0][0] << " " << p.val[0][1] << " "  << p.val[0][2]  << " "  << p.val[0][3] << " ";
    os << p.val[1][0] << " " << p.val[1][1] << " "  << p.val[1][2]  << " "  << p.val[1][3] << " ";
    os << p.val[2][0] << " " << p.val[2][1] << " "  << p.val[2][2]  << " "  << p.val[2][3];
    return os;
  }
  
protected:

  // calls bucketing and motion estimation
  bool updateMotion ();
public:
  // compute transformation matrix from transformation vector  
  Matrix transformationVectorToMatrix (std::vector<double> tr);
  std::vector<double> transformationMatrixToVector (Matrix Tr);

  // compute motion from previous to current coordinate system
  // if motion could not be computed, resulting vector will be of size 0
  virtual std::vector<double> estimateMotion (const std::vector<p_match> &p_matched,
                                              const std::vector<double> tr_delta= std::vector<double>(6,0)) = 0;
  
  // get random and unique sample of num numbers from 1:N
  std::vector<int32_t> getRandomSample (int32_t N,int32_t num);
public:
  Matrix                         Tr_delta;   // transformation (previous -> current frame)  
  bool                           Tr_valid;   // motion estimate exists?
  Matcher                       *matcher;    // feature matcher
protected:
  std::vector<int32_t>           inliers;    // inlier set
  double                        *J;          // jacobian
  double                        *p_observe;  // observed 2d points
  double                        *p_predict;  // predicted 2d points
  std::vector<p_match>  p_matched;  // feature point matches
  
private:
  
  parameters                    param;     // common parameters
};
}
#endif // VISO_H

