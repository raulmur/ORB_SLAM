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

#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "matcher.h"
#include "matrix.h"
namespace libviso2 {

class Reconstruction {

public:
  
  // constructor
  Reconstruction ();
  
  // deconstructor
  ~Reconstruction ();
  
  // a generic 3d point
  struct point3d {
    float x,y,z;
    point3d () {}
    point3d (float x,float y,float z) : x(x),y(y),z(z) {}
  };

  // set calibration parameters (intrinsics), must be called at least once
  // input: f ....... focal length (assumes fu=fv)
  //        cu,cv ... principal point
  void setCalibration (FLOAT f,FLOAT cu,FLOAT cv);
  
  // takes a set of monocular feature matches (flow method) and the egomotion
  // estimate between the 2 frames Tr, tries to associate the features with previous
  // frames (tracking) and computes 3d points once tracks gets lost.
  // point types: 0 ..... everything
  //              1 ..... road and above
  //              2 ..... only above road
  // min_track_length ... min number of frames a point needs to be tracked for reconstruction
  // max_dist ........... maximum point distance from camera
  // min_angle .......... avoid angles smaller than this for reconstruction (in degrees)
  void update (std::vector<p_match> p_matched,Matrix Tr,int32_t point_type=1,int32_t min_track_length=2,double max_dist=30,double min_angle=2);
  
  // return currently computed 3d points (finished tracks)
  std::vector<point3d> getPoints() { return points; }

private:
  
  struct point2d {
    float u,v;
    point2d () {}
    point2d (float u,float v) : u(u),v(v) {}
  };
  
  struct track {
    std::vector<point2d> pixels;
    int32_t first_frame;
    int32_t last_frame;
    int32_t last_idx;
  };
  
  enum result { UPDATED, FAILED, CONVERGED };
  
  bool    initPoint(const track &t,point3d &p);
  bool    refinePoint(const track &t,point3d &p);
  double  pointDistance(const track &t,point3d &p);
  double  rayAngle(const track &t,point3d &p);
  int32_t pointType(const track &t,point3d &p);
  result  updatePoint(const track &t,point3d &p,const FLOAT &step_size,const FLOAT &eps);
  void    computeObservations(const std::vector<point2d> &p);
  bool    computePredictionsAndJacobian(const std::vector<Matrix>::iterator &P_begin,const std::vector<Matrix>::iterator &P_end,point3d &p);
  void    testJacobian();
  
  // calibration matrices
  Matrix K,Tr_cam_road;
  
  std::vector<track>   tracks;
  std::vector<Matrix>  Tr_total;
  std::vector<Matrix>  Tr_inv_total;
  std::vector<Matrix>  P_total;
  std::vector<point3d> points;
  
  FLOAT *J;                     // jacobian
  FLOAT *p_observe,*p_predict;  // observed and predicted 2d points

};
}
#endif // RECONSTRUCTION_H
