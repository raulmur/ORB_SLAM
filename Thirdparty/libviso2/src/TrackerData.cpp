#include "TrackerData.h"
#include "TooN/helpers.h"
#include "TooN/TooN.h"
#include "TooN/wls.h"
#include "MEstimator.h"
namespace libviso2 {

const double TrackerData::dSqrtInvNoise(1.0);
TooN::Matrix<2> TrackerData::m2CamDerivs=Zeros;
void TrackerData::Project(const SE3<> &se3CFromW, const double calib_f, const double calib_cu,const double calib_cv, const double baseline)
{
    v3Cam = se3CFromW * v3WorldPos;
    v3Cam[0]-=baseline;
//    assert(v3Cam[2] >= 0.001);
    Vector<2> v2ImPlane = project(v3Cam);
    v2Image[0] = v2ImPlane[0]*calib_f+calib_cu;
    v2Image[1] = v2ImPlane[1]*calib_f+calib_cv;
//    assert(v2Image[0]>= 0 && v2Image[1]>= 0);
}

// Jacobian of projection W.R.T. the camera position
// I.e. if  p_cam = SE3Old * p_world,
//         SE3New = SE3Motion * SE3Old
void TrackerData::CalcJacobian()
{
    double dOneOverCameraZ = 1.0 / v3Cam[2];
    for(int m=0; m<6; m++)
    {
        const Vector<4> v4Motion = SE3<>::generator_field(m, unproject(v3Cam));
        Vector<2> v2CamFrameMotion;
        v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
        v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
        m26Jacobian.T()[m] = m2CamDerivs * v2CamFrameMotion;
    };
}

TooN::Vector<6> CalcPoseUpdate(const std::vector<TrackerData*> &vTD, double dOverrideSigma, bool bMarkOutliers)
{
  // Which M-estimator are we using? 0, "Tukey", 1, "Cauchy", 2,  "Huber"
  int nEstimator = 0;
    
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  std::vector<double> vdErrorSquared;
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];     
      TD.v2Error_CovScaled = TrackerData::dSqrtInvNoise* (TD.v2Found - TD.v2Image);
      vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
    };
  
  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
    return makeVector( 0,0,0,0,0,0);
  
  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  else
    {
      if (nEstimator == 0)
	dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
	dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else 
	dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }
  
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];     
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;
      
      if(nEstimator == 0)
	dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
	dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else 
	dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      if(TD.age>0)// for integrated feature, always >0, otherwise, always==-1
        dWeight*=TD.age;
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
	{
	  if(bMarkOutliers)
	    TD.isInlier=false;
	  continue;
	}
      else{
	if(bMarkOutliers)
	  TD.isInlier=true;
      }
      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TrackerData::dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TrackerData::dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
  
  wls.compute();
  //if (bMarkOutliers)	//only true for last iteration, see code above... (iter==9)
	//mmCovariances=TooN::SVD<6>(wls.get_C_inv()).get_pinv();
	
  return wls.get_mu();
}

}
