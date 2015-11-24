#ifndef FIVE_POINT_HPP
#define FIVE_POINT_HPP
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>

// the following content are copied from opencv2/calib3d.hpp at github opencv
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/affine.hpp"

namespace cv{
Mat findEssentialMat( InputArray points1, InputArray points2,
                                 double focal = 1.0, Point2d pp = Point2d(0, 0),
                                 int method = RANSAC, double prob = 0.999,
                                 double threshold = 1.0, OutputArray mask = noArray() );

/** @brief Decompose an essential matrix to possible rotations and translation.
@param E The input essential matrix.
@param R1 One possible rotation matrix.
@param R2 Another possible rotation matrix.
@param t One possible translation.
This function decompose an essential matrix E using svd decomposition @cite HartleyZ00 . Generally 4
possible poses exists for a given E. They are \f$[R_1, t]\f$, \f$[R_1, -t]\f$, \f$[R_2, t]\f$, \f$[R_2, -t]\f$. By
decomposing E, you can only get the direction of the translation, so the function returns unit t.
 */
void decomposeEssentialMat( InputArray E, OutputArray R1, OutputArray R2, OutputArray t );

/** @brief Recover relative camera rotation and translation from an estimated essential matrix and the
corresponding points in two images, using cheirality check. Returns the number of inliers which pass
the check.
@param E The input essential matrix.
@param points1 Array of N 2D points from the first image. The point coordinates should be
floating-point (single or double precision).
@param points2 Array of the second image points of the same size and format as points1 .
@param R Recovered relative rotation.
@param t Recoverd relative translation.
@param focal Focal length of the camera. Note that this function assumes that points1 and points2
are feature points from cameras with same focal length and principle point.
@param pp Principle point of the camera.
@param mask Input/output mask for inliers in points1 and points2.
:   If it is not empty, then it marks inliers in points1 and points2 for then given essential
matrix E. Only these inliers will be used to recover pose. In the output mask only inliers
which pass the cheirality check.
This function decomposes an essential matrix using decomposeEssentialMat and then verifies possible
pose hypotheses by doing cheirality check. The cheirality check basically means that the
triangulated 3D points should have positive depth. Some details can be found in @cite Nister03 .
This function can be used to process output E and mask from findEssentialMat. In this scenario,
points1 and points2 are the same input for findEssentialMat. :
@code
    // Example. Estimation of fundamental matrix using the RANSAC algorithm
    int point_count = 100;
    vector<Point2f> points1(point_count);
    vector<Point2f> points2(point_count);
    // initialize the points here ...
    for( int i = 0; i < point_count; i++ )
    {
        points1[i] = ...;
        points2[i] = ...;
    }
    double focal = 1.0;
    cv::Point2d pp(0.0, 0.0);
    Mat E, R, t, mask;
    E = findEssentialMat(points1, points2, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points1, points2, R, t, focal, pp, mask);
@endcode
 */
int recoverPose( InputArray E, InputArray points1, InputArray points2,
                            OutputArray R, OutputArray t,
                            double focal = 1.0, Point2d pp = Point2d(0, 0),
                            InputOutputArray mask = noArray() );


/** @brief For points in an image of a stereo pair, computes the corresponding epilines in the other image.
@param points Input points. \f$N \times 1\f$ or \f$1 \times N\f$ matrix of type CV_32FC2 or
vector\<Point2f\> .
@param whichImage Index of the image (1 or 2) that contains the points .
@param F Fundamental matrix that can be estimated using findFundamentalMat or stereoRectify .
@param lines Output vector of the epipolar lines corresponding to the points in the other image.
Each line \f$ax + by + c=0\f$ is encoded by 3 numbers \f$(a, b, c)\f$ .
For every point in one of the two images of a stereo pair, the function finds the equation of the
corresponding epipolar line in the other image.
From the fundamental matrix definition (see findFundamentalMat ), line \f$l^{(2)}_i\f$ in the second
image for the point \f$p^{(1)}_i\f$ in the first image (when whichImage=1 ) is computed as:
\f[l^{(2)}_i = F p^{(1)}_i\f]
And vice versa, when whichImage=2, \f$l^{(1)}_i\f$ is computed from \f$p^{(2)}_i\f$ as:
\f[l^{(1)}_i = F^T p^{(2)}_i\f]
Line coefficients are defined up to a scale. They are normalized so that \f$a_i^2+b_i^2=1\f$ .
 */
}

//the following is written by Huai, 2014
// extract camera extrinsics from essential matrix which is estimated with matched points, points1 and points2
// points1 observed by camera 1, points2 by camera 2,
// output, Rc12c2 is the rotation from camera 1 frame to camera 2 frame,
// output, Tc1inc2 is the normalized coordinates of the origin of camera 1 frame in camera 2 frame,
// A is camera matrix K=[fx, 0, cx; 0, fy, cy, 0, 0, 1] in pixel units
// one drawback of this function is that it doesnot consider the camera distortion
//return number of inliers
int ExtractCameras(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                    cv::Mat &Rc12c2, cv::Mat& tc1inc2, const cv::Mat & A);
#endif // FIVE_POINT_HPP

