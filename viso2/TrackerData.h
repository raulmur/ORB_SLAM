#ifndef TRACKERDATA_H
#define TRACKERDATA_H
#include "TooN/TooN.h"
#include "TooN/se3.h"
#include <vector>
namespace  libviso2 {

class TrackerData
{
public:
    TrackerData():isInlier(false), age(-1)
    {}
    TrackerData(const TrackerData& other):
        v3WorldPos(other.v3WorldPos),v2Image(other.v2Image),
        v2Found(other.v2Found), v2Error_CovScaled(other.v2Error_CovScaled),
        m26Jacobian(other.m26Jacobian),isInlier(other.isInlier), age(other.age)
    {}
    TrackerData & operator=(const TrackerData &other)
    {
        if(this==&other)
            return *this;
        v3WorldPos=other.v3WorldPos;
        v2Image=other.v2Image;
        v2Found=other.v2Found;
        v2Error_CovScaled=other.v2Error_CovScaled;
        m26Jacobian=other.m26Jacobian;
        isInlier=other.isInlier; age=other.age;
        return *this;
    }

    // Project point into image given certain pose and camera.
    // This can bail out at several stages if the point
    // will not be properly in the image.
    //Project 3d points onto right image of the stereo if baseline!=0
    void Project(const TooN::SE3<> &se3CFromW, const double calib_f, const double calib_cu,const double calib_cv, const double baseline=0);


    // Get the projection derivatives (depend only on the camera.)
    // This is called Unsafe because it depends on the camera caching
    // results from the previous projection:
    // Only do this right after the same point has been projected!
    static void GetDerivsUnsafe( double calib_f)
    {
        m2CamDerivs[0][0] = calib_f;
        m2CamDerivs[1][1] = 0;
        m2CamDerivs[0][1] = 0;
        m2CamDerivs[1][1] = calib_f;
    }

    void CalcJacobian();

    // Sometimes in tracker instead of reprojecting, just update the error linearly!
    inline void LinearUpdate(const TooN::Vector<6> &v6)
    {
        v2Image += m26Jacobian * v6;
    }
    TooN::Vector<3> v3WorldPos;        // Coords of the point in the world frame
    // Projection itermediates:
    TooN::Vector<3> v3Cam; // point position in current image frame
    TooN::Vector<2> v2Image;      // predicted Pixel coords in LEVEL0
    static TooN::Matrix<2> m2CamDerivs;  // Camera projection derivs d(u,v)/d(x/z, y/z)

    TooN::Vector<2> v2Found;      // measured Pixel coords of found patch (L0)

    // Stuff for pose update:
    TooN::Vector<2> v2Error_CovScaled;
    TooN::Matrix<2,6> m26Jacobian;   // Jacobian wrt camera position

    bool isInlier;
    int age; // used for integrated feature, age of feature in the last frame, age=0 in first frame
    static const double dSqrtInvNoise;   // Only depends on search level..
};
TooN::Vector<6> CalcPoseUpdate(const std::vector<TrackerData*> & vTD, double dOverrideSigma, bool bMarkOutliers);
}
#endif // TRACKERDATA_H
