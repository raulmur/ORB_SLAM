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

#include "reconstruction.h"
#include <fstream>

using namespace std;
namespace libviso2{
Reconstruction::Reconstruction () {
  K = Matrix::eye(3);

  // First transformation matrix
  Tr_total.push_back(Matrix::eye(4));

  // First inverse transformation matrix
  Tr_inv_total.push_back(Matrix::eye(4));
}

Reconstruction::~Reconstruction () {
}

void Reconstruction::setCalibration (FLOAT f,FLOAT cu,FLOAT cv) {
  FLOAT K_data[9]       = {f,0,cu,0,f,cv,0,0,1};
  K                     = Matrix(3,3,K_data);
  FLOAT cam_pitch       = -0.08;
  FLOAT cam_height      = 1.6;
  Tr_cam_road           = Matrix(4,4);
  Tr_cam_road.val[0][0] = 1;
  Tr_cam_road.val[1][1] = +cos(cam_pitch);
  Tr_cam_road.val[1][2] = -sin(cam_pitch);
  Tr_cam_road.val[2][1] = +sin(cam_pitch);
  Tr_cam_road.val[2][2] = +cos(cam_pitch);
  Tr_cam_road.val[0][3] = 0;
  Tr_cam_road.val[1][3] = -cam_height;
  Tr_cam_road.val[2][3] = 0;

  // First camera projection matrix
  P_total.push_back(K*Matrix::eye(4).getMat(0,0,2,3));
}

void Reconstruction::update (vector<p_match> p_matched,Matrix Tr,int32_t point_type,int32_t min_track_length,double max_dist,double min_angle) {
  
  // update transformation vector
  Matrix Tr_total_curr;
  if (Tr_total.size()==0) Tr_total_curr = Matrix::inv(Tr);
  else                    Tr_total_curr = Tr_total.back()*Matrix::inv(Tr);
  Tr_total.push_back(Tr_total_curr);
  Tr_inv_total.push_back(Matrix::inv(Tr_total_curr));
  
  // update projection vector
  Matrix P_total_curr = K*Matrix::inv(Tr_total_curr).getMat(0,0,2,3);
  P_total.push_back(P_total_curr);
  
  // current frame
  int32_t current_frame = Tr_total.size() - 1; // 0-based frame number
  
  // create index vector
  int32_t track_idx_max = 0;
  for (vector<p_match>::iterator m=p_matched.begin(); m!=p_matched.end(); m++)
    if (m->i1p > track_idx_max)
      track_idx_max = m->i1p;
  for (vector<track>::iterator t=tracks.begin(); t!=tracks.end(); t++)
    if (t->last_idx > track_idx_max)
      track_idx_max = t->last_idx;
  int32_t *track_idx = new int32_t[track_idx_max+1];
  for (int32_t i=0; i<=track_idx_max; i++)
    track_idx[i] = -1;
  for (int32_t i=0; i<tracks.size(); i++)
    track_idx[tracks[i].last_idx] = i;
  
  // associate matches to tracks
  for (vector<p_match>::iterator m=p_matched.begin(); m!=p_matched.end(); m++) {
    
    // track index (-1 = no existing track)
    int32_t idx = track_idx[m->i1p];
    
    // add to existing track
    if (idx>=0 && tracks[idx].last_frame==current_frame-1) {
      
      tracks[idx].pixels.push_back(point2d(m->u1c,m->v1c));
      tracks[idx].last_frame = current_frame;
      tracks[idx].last_idx   = m->i1c;
      
    // create new track
    } else {
      track t;
      t.pixels.push_back(point2d(m->u1p,m->v1p));
      t.pixels.push_back(point2d(m->u1c,m->v1c));
      t.first_frame = current_frame-1;
      t.last_frame  = current_frame;
      t.last_idx    = m->i1c;
      tracks.push_back(t);
    }
  }
  
  // copy tracks
  vector<track> tracks_copy = tracks;
  tracks.clear();
  
  // devise tracks into active or reconstruct 3d points
  for (vector<track>::iterator t=tracks_copy.begin(); t!=tracks_copy.end(); t++) {
    
    // track has been extended
    if (t->last_frame==current_frame) {
      
      // push back to active tracks
      tracks.push_back(*t);
      
    // track lost
    } else {
      
      // add to 3d reconstruction
      if (t->pixels.size()>=min_track_length) {
        
        // 3d point
        point3d p;
        
        // try to init point from first and last track frame
        if (initPoint(*t,p)) {
          if (pointType(*t,p)>=point_type)
            if (refinePoint(*t,p))
              if(pointDistance(*t,p)<max_dist && rayAngle(*t,p)>min_angle)
                points.push_back(p);
        }
      }
    }
  }
  
  //cout << "P: " << points.size() << endl;
  //testJacobian();
  
  delete track_idx;
}

bool Reconstruction::initPoint(const track &t,point3d &p) {
  
  // projection matrices
  Matrix  P1 = P_total[t.first_frame];
  Matrix  P2 = P_total[t.last_frame];
  
  // observations
  point2d p1 = t.pixels.front();
  point2d p2 = t.pixels.back();
  
  // triangulation via orthogonal regression
  Matrix J(4,4);
  Matrix U,S,V;
  for (int32_t j=0; j<4; j++) {
    J.val[0][j] = P1.val[2][j]*p1.u - P1.val[0][j];
    J.val[1][j] = P1.val[2][j]*p1.v - P1.val[1][j];
    J.val[2][j] = P2.val[2][j]*p2.u - P2.val[0][j];
    J.val[3][j] = P2.val[2][j]*p2.v - P2.val[1][j];
  }
  J.svd(U,S,V);
  
  // return false if this point is at infinity
  float w = V.val[3][3];
  if (fabs(w)<1e-10)
    return false;

  // return 3d point
  p = point3d(V.val[0][3]/w,V.val[1][3]/w,V.val[2][3]/w);
  return true;
}

bool Reconstruction::refinePoint(const track &t,point3d &p) {
  
  int32_t num_frames = t.pixels.size();
  J         = new FLOAT[6*num_frames];
  p_observe = new FLOAT[2*num_frames];
  p_predict = new FLOAT[2*num_frames];
 
  int32_t iter=0;
  Reconstruction::result result = UPDATED;
  while (result==UPDATED) {     
    result = updatePoint(t,p,1,1e-5);
    if (iter++ > 20 || result==CONVERGED)
      break;
  }
  
  delete J;
  delete p_observe;
  delete p_predict;
  
  if (result==CONVERGED)
    return true;
  else
    return false;
}

double Reconstruction::pointDistance(const track &t,point3d &p) {
  int32_t mid_frame = (t.first_frame+t.last_frame)/2;
  double dx = Tr_total[mid_frame].val[0][3]-p.x;
  double dy = Tr_total[mid_frame].val[1][3]-p.y;
  double dz = Tr_total[mid_frame].val[2][3]-p.z;
  return sqrt(dx*dx+dy*dy+dz*dz);
}

double Reconstruction::rayAngle(const track &t,point3d &p) {
  Matrix c1 = Tr_total[t.first_frame].getMat(0,3,2,3);
  Matrix c2 = Tr_total[t.last_frame].getMat(0,3,2,3);
  Matrix pt(3,1);
  pt.val[0][0] = p.x;
  pt.val[1][0] = p.y;
  pt.val[2][0] = p.z;
  Matrix v1 = c1-pt;
  Matrix v2 = c2-pt;
  FLOAT  n1 = v1.l2norm();
  FLOAT  n2 = v2.l2norm();
  if (n1<1e-10 || n2<1e-10)
    return 1000;
  v1 = v1/n1;
  v2 = v2/n2;
  return acos(fabs((~v1*v2).val[0][0]))*180.0/M_PI;
}

int32_t Reconstruction::pointType(const track &t,point3d &p) {
  
  // project point to first and last camera coordinates
  Matrix x(4,1);
  x.val[0][0] = p.x;
  x.val[1][0] = p.y;
  x.val[2][0] = p.z;
  x.val[3][0] = 1;
  Matrix x1c = Tr_inv_total[t.first_frame]*x;
  Matrix x2c = Tr_inv_total[t.last_frame]*x;
  Matrix x2r = Tr_cam_road*x2c;
  
  // point not visible
  if (x1c.val[2][0]<=1 || x2c.val[2][0]<=1)
    return -1;
  
  // below road
  if (x2r.val[1][0]>0.5)
    return 0;
  
  // road
  if (x2r.val[1][0]>-1)
    return 1;
  
  // obstacle
  return 2;
}

Reconstruction::result Reconstruction::updatePoint(const track &t,point3d &p,const FLOAT &step_size,const FLOAT &eps) {
  
  // number of frames
  int32_t num_frames = t.pixels.size();
  
  // extract observations
  computeObservations(t.pixels);
  
  // compute predictions
  vector<Matrix>::iterator P_begin = P_total.begin()+t.first_frame;
  vector<Matrix>::iterator P_end   = P_begin+num_frames-1;
  
  if (!computePredictionsAndJacobian(P_begin,P_end,p))
    return FAILED;
  
  // init
  Matrix A(3,3);
  Matrix B(3,1);

  // fill matrices A and B
  for (int32_t m=0; m<3; m++) {
    for (int32_t n=0; n<3; n++) {
      FLOAT a = 0;
      for (int32_t i=0; i<2*num_frames; i++)
        a += J[i*3+m]*J[i*3+n];
      A.val[m][n] = a;
    }
    FLOAT b = 0;
    for (int32_t i=0; i<2*num_frames; i++)
      b += J[i*3+m]*(p_observe[i]-p_predict[i]);
    B.val[m][0] = b;
  }
  
  // perform elimination
  if (B.solve(A)) {
    p.x += step_size*B.val[0][0];
    p.y += step_size*B.val[1][0];
    p.z += step_size*B.val[2][0];
    if (fabs(B.val[0][0])<eps && fabs(B.val[1][0])<eps && fabs(B.val[2][0])<eps)
      return CONVERGED;
    else
      return UPDATED;
  }
  return FAILED;
}

void Reconstruction::computeObservations(const vector<point2d> &pixels) {
  for (int32_t i=0; i<pixels.size(); i++) {
    p_observe[i*2+0] = pixels[i].u;
    p_observe[i*2+1] = pixels[i].v;
  }
}

bool Reconstruction::computePredictionsAndJacobian(const vector<Matrix>::iterator &P_begin,const vector<Matrix>::iterator &P_end,point3d &p) {
  
  // for all frames do
  int32_t k=0;
  for (vector<Matrix>::iterator P=P_begin; P<=P_end; P++) {
    
    // precompute coefficients
    FLOAT a  = P->val[0][0]*p.x+P->val[0][1]*p.y+P->val[0][2]*p.z+P->val[0][3];
    FLOAT b  = P->val[1][0]*p.x+P->val[1][1]*p.y+P->val[1][2]*p.z+P->val[1][3];
    FLOAT c  = P->val[2][0]*p.x+P->val[2][1]*p.y+P->val[2][2]*p.z+P->val[2][3];
    FLOAT cc = c*c;
       
    // check singularities
    if (cc<1e-10)
      return false;
    
    // set jacobian entries
    J[k*6+0] = (P->val[0][0]*c-P->val[2][0]*a)/cc;
    J[k*6+1] = (P->val[0][1]*c-P->val[2][1]*a)/cc;
    J[k*6+2] = (P->val[0][2]*c-P->val[2][2]*a)/cc;
    J[k*6+3] = (P->val[1][0]*c-P->val[2][0]*b)/cc;
    J[k*6+4] = (P->val[1][1]*c-P->val[2][1]*b)/cc;
    J[k*6+5] = (P->val[1][2]*c-P->val[2][2]*b)/cc;
       
    // set prediction
    p_predict[k*2+0] = a/c; // u
    p_predict[k*2+1] = b/c; // v
    
    k++;
  }
  
  // success
  return true;
}

void Reconstruction::testJacobian() {
  cout << "=================================" << endl;
  cout << "TESTING JACOBIAN" << endl;
  FLOAT delta = 1e-5;
  vector<Matrix> P;
  Matrix A(3,4);
  A.setMat(K,0,0);
  P.push_back(A);
  A.setMat(Matrix::rotMatX(0.1)*Matrix::rotMatY(0.1)*Matrix::rotMatZ(0.1),0,0);
  A.val[1][3] = 1;
  A.val[1][3] = 0.1;
  A.val[1][3] = -1.5;
  P.push_back(K*A);
  cout << P[0] << endll;
  cout << P[1] << endll;
  J         = new FLOAT[6*2];
  p_observe = new FLOAT[2*2];
  p_predict = new FLOAT[2*2];
  
  point3d p_ref(0.1,0.2,0.3);
  
  FLOAT p_predict1[4];
  FLOAT p_predict2[4];
  point3d p1 = p_ref;
   
  for (int32_t i=0; i<3; i++) {
    point3d p2 = p_ref;
    if (i==0)      p2.x += delta;
    else if (i==1) p2.y += delta;
    else           p2.z += delta;
    cout << endl << "Checking parameter " << i << ":" << endl;
    cout << "param1: "; cout << p1.x << " " << p1.y << " " << p1.z << endl;
    cout << "param2: "; cout << p2.x << " " << p2.y << " " << p2.z << endl;
    computePredictionsAndJacobian(P.begin(),P.end(),p1);
    memcpy(p_predict1,p_predict,4*sizeof(FLOAT));
    computePredictionsAndJacobian(P.begin(),P.end(),p2);
    memcpy(p_predict2,p_predict,4*sizeof(FLOAT));
    for (int32_t j=0; j<4; j++) {
      cout << "num: " << (p_predict2[j]-p_predict1[j])/delta;
      cout << ", ana: " << J[j*3+i] << endl;
    }
  }
  
  delete J;
  delete p_observe;
  delete p_predict;
  cout << "=================================" << endl;
}
}
