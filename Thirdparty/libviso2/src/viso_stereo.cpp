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

#include "viso_stereo.h"
#ifdef USE_OPENCV
#include "five-point.h"
#include <opencv2/highgui/highgui.hpp> //for display
#include <opencv2/features2d/features2d.hpp> // drawmatches
#endif
#ifdef USE_TOON
#include "TooN/TooN.h" //for WLS incremental motion estimation
#include "TrackerData.h" //for WLS
#endif

using namespace std;
namespace libviso2{
VisualOdometryStereo::VisualOdometryStereo (parameters param_) :  VisualOdometry(param_), param(param_){
  matcher->setIntrinsics(param.calib.f,param.calib.cu,param.calib.cv,param.base);
}

VisualOdometryStereo::~VisualOdometryStereo() {
}

bool VisualOdometryStereo::process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace) {
    const enum Optimizer {RANSAC_Geiger, RANSAC_5Point, ROBUST_Klein} approach=RANSAC_5Point;
    // Based on some preliminary test, Klein's method gives similar result to Geiger's method,
    // In terms of speed, Geiger and Klein's method are similar

    // push back images
    matcher->pushBack(I1,I2,dims,replace);

    // bootstrap motion estimate if invalid
    // match features and update motion
    if (Tr_valid) matcher->matchFeatures(2, &Tr_delta);
    else          matcher->matchFeatures(2);
    /*//{Huai: draw matches with opencv
    p_matched = matcher->getMatches();
    vector<cv::KeyPoint> keys1, keys2;
    vector<vector<cv::DMatch> > matches122;
    for(int i=0; i<p_matched.size(); ++i)
    {
        keys1.push_back(cv::KeyPoint(p_matched[i].u1p, p_matched[i].v1p, 4));
        keys2.push_back(cv::KeyPoint(p_matched[i].u1c, p_matched[i].v1c, 4));
        vector<cv::DMatch> yummy;
        yummy.push_back(cv::DMatch(i,i, 1));
        matches122.push_back(yummy);
    }
    cv::Mat outimg;
    cv::Mat image1, image2;
    image1=cv::Mat(dims[1], dims[0], CV_8UC1, I1);
    image2=cv::Mat(dims[1], dims[0], CV_8UC1, I2);
    //  cv::drawMatches(image1,keys1,image2, keys2, matches122, outimg);
    cv::drawKeypoints(image1,keys1,outimg);
    cv::imshow("Matches", outimg);
    cv::waitKey();
    cout<<p_matched.size()<<" before bucketing!"<<endl;
    //Huai}*/
    p_all_matched= matcher->getMatches();
    matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);
    p_matched = matcher->getMatches();
    //  cout<<p_matched.size()<<" after bucketing!"<<endl;
    // estimate motion
    vector<double> tr_delta;
    switch (approach){
    case RANSAC_Geiger:
        tr_delta= estimateMotion(p_matched);
        // on failure
        if (tr_delta.size()!=6){
            Tr_delta= Matrix::eye(4);
            Tr_valid= false;
        }else{
            // set transformation matrix (previous to current frame)
            Tr_delta = transformationVectorToMatrix(tr_delta);
            Tr_valid = true;
        }
        return Tr_valid;
    case RANSAC_5Point:
        return estimateMotion5Point(p_matched);      
    case ROBUST_Klein:
        return estimateMotionKlein(p_matched, vector<vector<float> >());
    }
}

// p_matched, $m_{i,t-1}$ to $m_{i,t}$, if mean_features is not empty, they are $\bar{m}_{i,t-1}$ in (age(t-1), u1p, v1p, u2p) 4-tuple
bool VisualOdometryStereo::estimateMotionKlein (const vector<p_match> & p_matched, const vector<vector<float> >& mean_features) {
#ifdef USE_TOON
    using namespace TooN;
    int addition_features=0;
    for(unsigned int i=0; i<mean_features.size();++i)
    {//NB some features may not have mean features because their tracklength==2
        if(mean_features[i].size())
            ++addition_features;
    }
    // get number of matches
    int32_t N  = p_matched.size();
    if (N<6)
        return false;
    vector<TrackerData*> vIterationSet(2*(N+addition_features));
    for (int i=0; i<N; ++i){
        TrackerData temp;
        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        temp.v3WorldPos[0] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
        temp.v3WorldPos[1] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
        temp.v3WorldPos[2] = param.calib.f*param.base/d;
        temp.v2Found[0]=p_matched[i].u1c;
        temp.v2Found[1]=p_matched[i].v1c;
        vIterationSet[i*2]=new TrackerData(temp);
        temp.v2Found[0]=p_matched[i].u2c;
        temp.v2Found[1]=p_matched[i].v2c;
        vIterationSet[i*2+1]=new TrackerData(temp);
    }
    unsigned int jack=2*N;
    for(int i=0; i<N; ++i)
    {
         if(!mean_features[i].size())
            continue;
         TrackerData temp;
         temp.age=mean_features[i][0];
         double d = max(mean_features[i][1] - mean_features[i][3],0.0001f);
         temp.v3WorldPos[0] = (mean_features[i][1] -param.calib.cu)*param.base/d;
         temp.v3WorldPos[1] = (mean_features[i][2] -param.calib.cv)*param.base/d;
         temp.v3WorldPos[2] = param.calib.f*param.base/d;

         temp.v2Found[0]=p_matched[i].u1c;
         temp.v2Found[1]=p_matched[i].v1c;
         vIterationSet[jack]=new TrackerData(temp);
         temp.v2Found[0]=p_matched[i].u2c;
         temp.v2Found[1]=p_matched[i].v2c;
         vIterationSet[jack+1]=new TrackerData(temp);
         jack+=2;
    }
    assert(jack==vIterationSet.size());
    SE3<> mse3CamFromWorld;//in this case, the transformation from previous frame to current frame
    // ten gauss-newton pose update iterations.
    TooN::Vector<6> v6LastUpdate=Zeros;
    for(int iter = 0; iter<10; iter++)
    {
        bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
        // reprojection at every iteration - it really isn't necessary!
        if(iter == 0 || iter == 4 || iter == 9)
            bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
        else                            // iterations is for M-Estimator convergence rather than
            bNonLinearIteration = false;  // linearisation effects.
        if(bNonLinearIteration)
        {
            TrackerData::GetDerivsUnsafe(param.calib.f);
            for(unsigned int i=0; i<vIterationSet.size(); i+=2)
            {
                vIterationSet[i]->Project(mse3CamFromWorld, param.calib.f, param.calib.cu, param.calib.cv );
                vIterationSet[i]->CalcJacobian();
                vIterationSet[i+1]->Project(mse3CamFromWorld, param.calib.f, param.calib.cu, param.calib.cv, param.base);
                vIterationSet[i+1]->CalcJacobian();
            }
        }
        else
        {
            for(unsigned int i=0; i<vIterationSet.size(); i++)
                vIterationSet[i]->LinearUpdate(v6LastUpdate);
        }
        // Again, an M-Estimator hack beyond the fifth iteration.
        double dOverrideSigma = 0.0;
        if(iter > 5)
            dOverrideSigma = 16.0;
        // Calculate and update pose; also store update vector for linear iteration updates.
        TooN::Vector<6> v6Update =
                CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
        v6LastUpdate = v6Update;
    }
     //is converged
    int numInliers=0;
    for(int i=0; i<2*(N+addition_features);i+=2){// only consider left image points
        if(vIterationSet[i]->isInlier)
            ++numInliers;
        delete vIterationSet[i];
        delete vIterationSet[i+1];
    }
    if(numInliers<6){
        cerr<<"Too few inliers:"<<numInliers<<endl;
        return false;
    }
    if(norm_1(v6LastUpdate)>2e-3){
       cerr<<"Too few iterations:"<<norm_1(v6LastUpdate)<<endl;
    }
    TooN::Matrix<3> m3Rot=mse3CamFromWorld.get_rotation().get_matrix();
    Vector<3> v3Trans=mse3CamFromWorld.get_translation();
    Tr_delta.val[0][0]=m3Rot(0,0); Tr_delta.val[0][1]=m3Rot(0,1); Tr_delta.val[0][2]=m3Rot(0,2); Tr_delta.val[0][3]=v3Trans[0];
    Tr_delta.val[1][0]=m3Rot(1,0); Tr_delta.val[1][1]=m3Rot(1,1); Tr_delta.val[1][2]=m3Rot(1,2); Tr_delta.val[1][3]=v3Trans[1];
    Tr_delta.val[2][0]=m3Rot(2,0); Tr_delta.val[2][1]=m3Rot(2,1); Tr_delta.val[2][2]=m3Rot(2,2); Tr_delta.val[2][3]=v3Trans[2];
    Tr_delta.val[3][0]=0; Tr_delta.val[3][1]=0; Tr_delta.val[3][2]=0; Tr_delta.val[3][3]=1;
    Tr_valid = true;
    return true;
#else
    std::cerr<<"If estimateMotionKlein is desired, compile libviso2 with TooN support!"<<endl;
    return false;
#endif
}

vector<double> VisualOdometryStereo::estimateMotion (const vector<p_match> & p_matched,
                                                      const std::vector<double> tr_delta_init) {
  // return value
  bool success = true;
  // get number of matches
  int32_t N  = p_matched.size();
  if (N<6)
    return vector<double>();

  // allocate dynamic memory
  X          = new double[N];
  Y          = new double[N];
  Z          = new double[N];
  J          = new double[4*N*6];
  p_predict  = new double[4*N];
  p_observe  = new double[4*N];
  p_residual = new double[4*N];

  // project matches of previous image into 3d
  for (int32_t i=0; i<N; i++) {
    double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
    X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
    Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
    Z[i] = param.calib.f*param.base/d;
  }

  // loop variables
  vector<double> tr_delta;
  vector<double> tr_delta_curr;
  
  // clear parameter vector
  inliers.clear();
  // initial RANSAC estimate
  for (int32_t k=0;k<param.ransac_iters;k++) {

    // draw random sample set
    vector<int32_t> active = getRandomSample(N,3);

    // reinit parameter vector
    tr_delta_curr =tr_delta_init;

    // minimize reprojection errors
    VisualOdometryStereo::result result = UPDATED;
    int32_t iter=0;
    while (result==UPDATED) {
      result = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
      if (iter++ > 20 || result==CONVERGED)
        break;
    }

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
      if (inliers_curr.size()>inliers.size()) {
        inliers = inliers_curr;
        tr_delta = tr_delta_curr;
      }
    }
  }

  // final optimization (refinement)
  if (inliers.size()>=6) {
    int32_t iter=0;
    VisualOdometryStereo::result result = UPDATED;
    while (result==UPDATED) {
      result = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
      if (iter++ > 100 || result==CONVERGED)
        break;
    }

    // not converged
    if (result!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }
  // release dynamic memory
  delete[] X;
  delete[] Y;
  delete[] Z;
  delete[] J;
  delete[] p_predict;
  delete[] p_observe;
  delete[] p_residual;

  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return vector<double>();
}
// five point algorithm to estimate rotation, and 3 point algorithm to estimate translation
bool VisualOdometryStereo::estimateMotion5Point(const vector<p_match>& p_matched, const vector<double> tr_delta_init) {
#ifdef USE_OPENCV
    // get number of matches
    int32_t point_count  = p_matched.size();
    if (point_count<6){
        Tr_delta= Matrix::eye(4);
        Tr_valid= false;
        return false;
    }
    using namespace cv;
    cv::Mat cameraMatrix=(cv::Mat_<double>(3,3) << param.calib.f,0,param.calib.cu, 0, param.calib.f, param.calib.cv,0,0,1);
    cv::Mat Rf2s=cv::Mat::eye(3,3, CV_64F); //rotation from first to second camera frame
    cv::Mat tfins=cv::Mat::zeros(3,1, CV_64F);  //position of first camera frame in second camera frame

    vector<Point2f> points1(point_count);
    vector<Point2f> points2(point_count);
    for (int32_t kettle=0;kettle<point_count;++kettle)
    {
        points1[kettle].x=p_matched[kettle].u1p;
        points1[kettle].y=p_matched[kettle].v1p;
        points2[kettle].x=p_matched[kettle].u1c;
        points2[kettle].y=p_matched[kettle].v1c;
    }
    int num_inliers=ExtractCameras( points1,  points2, Rf2s, tfins, cameraMatrix);
    //estimate
    bool success = true;
    // allocate dynamic memory
    X          = new double[point_count];
    Y          = new double[point_count];
    Z          = new double[point_count];
    J          = new double[4*point_count*3]; //only refine translation
    p_predict  = new double[4*point_count];
    p_observe  = new double[4*point_count];
    p_residual = new double[4*point_count];

    // project matches of previous image into 3d
    for (int32_t i=0; i<point_count; i++) {
        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
    }
    // loop variables
    vector<double> tr_delta;
    vector<double> tr_delta_curr;

    // clear parameter vector
    inliers.clear();
    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;++k) {

        // draw random sample set
        vector<int32_t> active = getRandomSample(point_count,3);

        // reinit parameter vector
        tr_delta_curr =tr_delta_init;

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        int32_t iter=0;
        while (result==UPDATED) {
            result = updateParameters2(p_matched,active,Rf2s, tr_delta_curr,1,1e-6);
            if (iter++ > 20 || result==CONVERGED)
                break;
        }

        // overwrite best parameters if we have more inliers
        if (result!=FAILED) {
            vector<int32_t> inliers_curr = getInlier2(p_matched,tr_delta_curr, Rf2s);
            if (inliers_curr.size()>inliers.size()) {
                inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }

    // final optimization (refinement)
    if (inliers.size()>=6) {
        int32_t iter=0;
        VisualOdometryStereo::result result = UPDATED;
        while (result==UPDATED) {
            result = updateParameters2(p_matched,inliers,Rf2s,tr_delta,1,1e-8);
            if (iter++ > 100 || result==CONVERGED)
                break;
        }
        // not converged
        if (result!=CONVERGED)
            success = false;
        // not enough inliers
    } else {
        success = false;
    }
    // release dynamic memory
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] J;
    delete[] p_predict;
    delete[] p_observe;
    delete[] p_residual;

    // parameter estimate succeeded?
    if (success)
    {
        Tr_delta.val[0][0]= Rf2s.at<double>(0,0); Tr_delta.val[0][1]= Rf2s.at<double>(0,1); Tr_delta.val[0][2]= Rf2s.at<double>(0,2);
        Tr_delta.val[1][0]= Rf2s.at<double>(1,0); Tr_delta.val[1][1]= Rf2s.at<double>(1,1); Tr_delta.val[1][2]= Rf2s.at<double>(1,2);
        Tr_delta.val[2][0]= Rf2s.at<double>(2,0); Tr_delta.val[2][1]= Rf2s.at<double>(2,1); Tr_delta.val[2][2]= Rf2s.at<double>(2,2);
        Tr_delta.val[0][3]= tr_delta[0];
        Tr_delta.val[1][3]= tr_delta[1];
        Tr_delta.val[2][3]= tr_delta[2];
        Tr_valid =true;
    }
    else
    {
        Tr_delta= Matrix::eye(4);
        Tr_valid=false;

        for(int ink=0; ink<3;++ink){
            for(int jim=0; jim<3;++jim)
                cout<< Rf2s.at<double>(ink, jim)<<" ";
            cout<<endl;
        }

        cout<<"recoverPose num_inliers of points:"<<num_inliers<<" "<<point_count;
        cout<<", translation inliers:"<< inliers.size()<<endl;
    }
    return success;
#else
    std::cerr<<"If estimateMotion5Point is desired, compile libviso2 with OpenCV support!"<<endl;
    return false;
#endif
}
// tr previous to current transform
void VisualOdometryStereo::getAllInlier(const vector<p_match> &p_matched,
                                                   const Matrix &tr, std::vector<bool> & vInliers){
    // get number of matches
    int32_t N  = p_matched.size();
    // allocate dynamic memory
    X          = new double[N];
    Y          = new double[N];
    Z          = new double[N];

    p_predict  = new double[4*N];
    p_observe  = new double[4*N];

    // loop variables
    double X1p,Y1p,Z1p;
    double X1c,Y1c,Z1c,X2c;
    double r00    = tr.val[0][0];          double r01    = tr.val[0][1];          double r02    = tr.val[0][2];
    double r10   = tr.val[1][0];          double r11    = tr.val[1][1];          double r12    = tr.val[1][2];
    double r20    = tr.val[2][0];          double r21    = tr.val[2][1];          double r22    = tr.val[2][2];
    double tx    = tr.val[0][3];          double ty    = tr.val[1][3];          double tz    = tr.val[2][3];
    // project matches of previous image into 3d
    for (int32_t i=0; i<N; i++) {
        vector<double> res=unprojectPoint(p_matched[i].u1p, p_matched[i].v1p, p_matched[i].u2p);
        X[i] = res[0];
        Y[i] = res[1];
        Z[i] = res[2];
    }
    // extract observations and compute predictions
    // set all observations
    for (int32_t i=0; i<N; i++) {
        p_observe[4*i+0] = p_matched[i].u1c; // u1
        p_observe[4*i+1] = p_matched[i].v1c; // v1
        p_observe[4*i+2] = p_matched[i].u2c; // u2
        p_observe[4*i+3] = p_matched[i].v2c; // v2

        // get 3d point in previous coordinate system
        X1p = X[i];
        Y1p = Y[i];
        Z1p = Z[i];

        // compute 3d point in current left coordinate system
        X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
        Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
        Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

        // compute 3d point in current right coordinate system
        X2c = X1c-param.base;
        // set prediction (project via K)
        p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
        p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
        p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
        p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v
    }

    // compute inliers
    vInliers.resize(N);
    for (int32_t i=0; i<N; i++){
        if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
                pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) <
                param.inlier_threshold*param.inlier_threshold)
            vInliers[i]=true;
        else
            vInliers[i]=false;
    }
    // release dynamic memory
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] p_predict;
    delete[] p_observe;
}

vector<int32_t> VisualOdometryStereo::getInlier(const vector<p_match> &p_matched,const vector<double> &tr) {

  // mark all observations active
  vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(p_matched,active);  
  computeResidualsAndJacobian(tr,active); 
  // compute inliers
  vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
        pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
      inliers.push_back(i);
  return inliers;
}
vector<int32_t> VisualOdometryStereo::getInlier2(const vector<p_match> &p_matched,const vector<double> &tr, const cv::Mat Rf2s) {

  // mark all observations active
  vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian2(tr,active, Rf2s);
  // compute inliers
  vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
        pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
      inliers.push_back(i);
  return inliers;
}
VisualOdometryStereo::result VisualOdometryStereo::updateParameters(const vector<p_match> &p_matched, const vector<int32_t> &active, vector<double> &tr,double step_size,double eps) {
  
  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;
  
  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian(tr,active);

  // init
  const int32_t dim=6;
  Matrix A(dim,dim);
  Matrix B(dim,1);

  // fill matrices A and B
  for (int32_t m=0; m<dim; m++) {
    for (int32_t n=0; n<dim; n++) {
      double a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); ++i) {
        a += J[i*dim+m]*J[i*dim+n];
      }
      A.val[m][n] = a;
    }
    double b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); ++i) {
      b += J[i*dim+m]*(p_residual[i]);
    }
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<dim; m++) {
      tr[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}
// 3 point algorithm to estimate translation with fixed rotation, or estimate both
VisualOdometryStereo::result VisualOdometryStereo::updateParameters2(const vector<p_match> &p_matched,const vector<int32_t> &active,
                                                                     const cv::Mat Rf2s, vector<double> &tr,double step_size,double eps) {

  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;

  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian2(tr,active, Rf2s);

  // init
  const int32_t dim=3;
  Matrix A(dim,dim);
  Matrix B(dim,1);

  // fill matrices A and B
  for (int32_t m=0; m<dim; m++) {
    for (int32_t n=0; n<dim; n++) {
      double a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); ++i) {
        a += J[i*dim+m]*J[i*dim+n];
      }
      A.val[m][n] = a;
    }
    double b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); ++i) {
      b += J[i*dim+m]*(p_residual[i]);
    }
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<dim; m++) {
      tr[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}
void VisualOdometryStereo::computeObservations(const vector<p_match> &p_matched, const vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
    p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
  }
}

void VisualOdometryStereo::computeResidualsAndJacobian(const vector<double> &tr, const vector<int32_t> &active) {

  // extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  // compute rotation matrix and derivatives
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
    
    // weighting
    double weight = 1.0;
    if (param.reweighting)
      weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);
    
    // compute 3d point in current right coordinate system
    X2c = X1c-param.base;

    // for all paramters do
    for (int32_t j=0; j<6; j++) {

      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }

      // set jacobian entries (project via K)
      J[(4*i+0)*6+j] = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
      J[(4*i+1)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
      J[(4*i+2)*6+j] = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
      J[(4*i+3)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
    }

    // set prediction (project via K)
    p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
    p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
    p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
    p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v
    
    // set residuals
    p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
    p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
    p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
    p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
  }
}
//fix rotation, only refine translation
void VisualOdometryStereo::computeResidualsAndJacobian2(const vector<double> &tr,const vector<int32_t> &active, const cv::Mat Rf2s) {

  // extract motion parameters
  double tx = tr[0]; double ty = tr[1]; double tz = tr[2];
  // compute rotation matrix and derivatives
  double r00    = Rf2s.at<double>(0,0); double r01    = Rf2s.at<double>(0,1); double r02    = Rf2s.at<double>(0,2);
  double r10    = Rf2s.at<double>(1,0); double r11    = Rf2s.at<double>(1,1); double r12    = Rf2s.at<double>(1,2);
  double r20    = Rf2s.at<double>(2,0); double r21    = Rf2s.at<double>(2,1); double r22    = Rf2s.at<double>(2,2);

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

    // weighting
    double weight = 1.0;
    if (param.reweighting)
      weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);

    // compute 3d point in current right coordinate system
    X2c = X1c-param.base;

    // for all paramters do
    for (int32_t j=0; j<3; ++j) {
      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      switch (j) {
        case 0: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 1: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 2: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }
      // set jacobian entries (project via K)
      J[(4*i+0)*3+j] = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
      J[(4*i+1)*3+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
      J[(4*i+2)*3+j] = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
      J[(4*i+3)*3+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
    }

    // set prediction (project via K)
    p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
    p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
    p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
    p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v

    // set residuals
    p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
    p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
    p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
    p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
  }
}
}
