#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include "eigen_utils.h"

  Eigen::Vector3d rotro2eu(Eigen::Matrix3d R)
  {
      Eigen::Vector3d euler;
      euler[0] = atan2(R(2,1), R(2,2));
      euler[1] = -(atan2(R(2,0),  sqrt(1 - R(2,0) * R(2,0))));
      euler[2] = atan2(R(1,0), R(0,0));
      return euler;
  }

  //eul defined in "n": rotate "n" to obtain "b"
  //result: Cbn (from b to n)
  Eigen::Matrix3d roteu2ro(Eigen::Vector3d eul)
  {
      double cr = cos(eul[0]); double sr = sin(eul[0]);	//roll
      double cp = cos(eul[1]); double sp = sin(eul[1]);	//pitch
      double ch = cos(eul[2]); double sh = sin(eul[2]);	//heading
      Eigen::Matrix3d dcm;
      dcm(0,0) = cp * ch;
      dcm(0,1) = (sp * sr * ch) - (cr * sh);
      dcm(0,2) = (cr * sp * ch) + (sh * sr);

      dcm(1,0) = cp * sh;
      dcm(1,1) = (sr * sp * sh) + (cr * ch);
      dcm(1,2) = (cr * sp * sh) - (sr * ch);

      dcm(2,0) = -sp;
      dcm(2,1) = sr * cp;
      dcm(2,2) = cr * cp;
      return dcm;
  }
  //input: lat, long in radians, height is immaterial
  //output: Ce2n
  Eigen::Matrix3d llh2dcm( Eigen::Vector3d &llh)
  {
      double sL = sin(llh[0]);
      double cL = cos(llh[0]);
      double sl = sin(llh[1]);
      double cl = cos(llh[1]);

      Eigen::Matrix3d Ce2n;
      Ce2n<< -sL * cl, -sL * sl, cL ,  -sl, cl, 0 , -cL * cl, -cL * sl, -sL;
      return Ce2n;
  }

void TestEigenQuaternion()
{
using namespace std;
using namespace Eigen;
    vector<int> meas;
    for(int i=0; i<9; ++i)
        meas.push_back(i+1);
    meas.front()=meas.back();
    meas.resize(1);
    cout<<meas[0]<<endl;
    Quaterniond quat(0.2,0.5,-0.7,1.5);
    quat.normalize();
    cout<<"quat=["<<quat.coeffs()<<"];"<<endl;
    Vector3d rotvec(3,-2,1);

    //    cout<<quat._transformVector(rotvec)<<endl;
    Quaterniond quat2(3,-2,5,0.7);
    quat2.normalize();
    cout<<"quat2=["<<quat2.coeffs()<<"]"<<endl;
    //cout<<(quat*quat2).coeffs()<<endl;
    cout<<"quat2 rot:"<<endl<<quat2.toRotationMatrix()<<endl;
    Vector3d euler=rotro2eu(quat2.toRotationMatrix());
    cout<<"rot eu:"<<endl<< euler<<endl;
    cout<< "rot"<< endl<< roteu2ro(euler)<<endl;
}
