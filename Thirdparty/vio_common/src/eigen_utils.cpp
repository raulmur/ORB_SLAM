#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "vio/eigen_utils.h"

namespace vio{

using namespace Eigen;
using namespace std;
Eigen::Vector3d rotro2eu(Eigen::Matrix3d R)
{
    Eigen::Vector3d euler;
    euler[0] = atan2(R(2,1), R(2,2));
    euler[1] = -(atan2(R(2,0),  sqrt(1 - R(2,0) * R(2,0))));
    euler[2] = atan2(R(1,0), R(0,0));
    return euler;
}


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
Eigen::Matrix3d llh2dcm(const Eigen::Vector3d llh)
{
    double sL = sin(llh[0]);
    double cL = cos(llh[0]);
    double sl = sin(llh[1]);
    double cl = cos(llh[1]);

    Eigen::Matrix3d Ce2n;
    Ce2n<< -sL * cl, -sL * sl, cL ,  -sl, cl, 0 , -cL * cl, -cL * sl, -sL;
    return Ce2n;
}

Eigen::MatrixXd nullspace(const Eigen::MatrixXd& A)
{
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
   //ColPivHouseholderQR<MatrixXd> qr(A); //don't use column pivoting because in that case Q*R-A!=0
    Eigen::MatrixXd nullQ = qr.householderQ();

    int rows= A.rows(), cols= A.cols();
    assert( rows> cols); // "Rows should be greater than columns in computing nullspace"
    nullQ= nullQ.block(0,cols,rows,rows-cols).eval();
    return nullQ;
}

void leftNullspaceAndColumnSpace(const Eigen::MatrixXd &A,
                                        Eigen::MatrixXd *Q2,
                                        Eigen::MatrixXd *Q1)
{
    int rows= A.rows(), cols= A.cols();
    assert( rows> cols); // "Rows should be greater than columns in computing left nullspace"
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    //don't use column pivoting because in that case Q*R-A!=0
    Eigen::MatrixXd Q = qr.householderQ();

    Q2->resize(rows, rows-cols);
    *Q2 = Q.block(0,cols,rows,rows-cols);

    Q1->resize(rows, cols);
    *Q1 = Q.block(0,0,rows,cols);
}

Eigen::Matrix<double, Eigen::Dynamic, 1> superdiagonal(
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & M)
{
    const int numElements = std::min(M.rows(), M.cols()) - 1;
    Eigen::Matrix<double, Eigen::Dynamic, 1> r(numElements, 1);
    for(int jack = 0; jack< numElements; ++jack)
        r[jack] = M(jack, jack+1);
    return r;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> subdiagonal(
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> & M)
{
    const int numElements = std::min(M.rows(), M.cols()) - 1;
    Eigen::Matrix<double, Eigen::Dynamic, 1> r(numElements, 1);
    for(int jack = 0; jack< numElements; ++jack)
        r[jack] = M(jack+1, jack);
    return r;
}

void reparameterize_AIDP(const Eigen::Matrix3d &Ri, const Eigen::Matrix3d &Rj, const Eigen::Vector3d &abrhoi,
                         const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, Eigen::Vector3d& abrhoj,
                         Eigen::Matrix<double, 3, 9>* jacobian)
{
   Eigen::Matrix<double, 4,4> Tci2cj =Eigen::Matrix<double, 4,4>::Identity();
   Tci2cj.topLeftCorner<3,3>()= Rj.transpose()*Ri;
   Tci2cj.block<3,1>(0,3).noalias() = Rj.transpose()*(pi-pj);
   Eigen::Matrix<double, 4, 1> homogi;
   homogi<< abrhoi.head<2>(), 1, abrhoi[2];
   double rhoj_drhoi = 1/ (Tci2cj.row(2)* homogi); //\rho_j divided by \rho_i
   abrhoj.head<2>() = rhoj_drhoi* Tci2cj.topLeftCorner<2,4>()*homogi;
   abrhoj[2] = abrhoi[2]*rhoj_drhoi;
   if(jacobian)
   {
       Eigen::Matrix3d lhs;
       lhs.setIdentity();
       lhs.col(2)= - abrhoj;
       //{\alpha, \beta, \rho}_i
       Eigen::Matrix<double, 3, 3> subrhs;
       subrhs<< Tci2cj.topLeftCorner<3,2>(), Tci2cj.block<3,1>(0,3);
       jacobian->topLeftCorner<3,3>() = rhoj_drhoi*lhs*subrhs;
       (*jacobian)(2,2) = rhoj_drhoi*rhoj_drhoi*Tci2cj.block<1,3>(2,0)*homogi.head<3>();
       //{pi, pj}
       Eigen::Matrix<double, 3, 6> rhs;
       rhs.topLeftCorner<3,3>()= abrhoi[2]*Rj.transpose(),
       rhs.block<3,3>(0,3)= - rhs.topLeftCorner<3,3>();
       jacobian->block<3,6>(0,3) = rhoj_drhoi*lhs*rhs;

   }
}

void reparameterizeNumericalJacobian(const Eigen::Matrix3d &Ri, const Eigen::Matrix3d &Rj, const Eigen::Vector3d &abrhoi,
                                     const Eigen::Vector3d &pi, const Eigen::Vector3d &pj, Eigen::Vector3d& abrhoj,
                                     Eigen::Matrix<double, 3, 9>& jacobian){
   // numerical differentation
   Eigen::Vector3d abrhojp;
   reparameterize_AIDP(Ri, Rj, abrhoi, pi, pj, abrhoj);
   double h= 1e-8;
   for(int jack =0; jack<3; ++jack){
       Eigen::Vector3d abrhoip= abrhoi;
       abrhoip[jack]= abrhoi[jack] + h;
       reparameterize_AIDP(Ri, Rj, abrhoip, pi, pj, abrhojp);
       Eigen::Vector3d subJacobian = (abrhojp - abrhoj)/h;
       jacobian.col(jack) = subJacobian;
   }

   for(int jack =0; jack<3; ++jack){
       Eigen::Vector3d pip= pi; pip[jack]= pi[jack] + h;
       reparameterize_AIDP(Ri, Rj, abrhoi, pip, pj, abrhojp);
       Eigen::Vector3d subJacobian = (abrhojp - abrhoj)/h;
       jacobian.col(jack+3) = subJacobian;
   }

   for(int jack =0; jack<3; ++jack){
       Eigen::Vector3d pjp =pj; pjp[jack]= pj[jack] + h;
       reparameterize_AIDP(Ri, Rj, abrhoi, pi, pjp, abrhojp);
       Eigen::Vector3d subJacobian = (abrhojp - abrhoj)/h;
       jacobian.col(jack+6) = subJacobian;
   }
}

void testReparameterize()
{
    double distances[] ={3, 3e2, 3e4, 3e8}; //close to inifity
    for(size_t jack = 0; jack< sizeof(distances)/sizeof(distances[0]); ++jack){
        double dist = distances[jack];

        Eigen::Matrix3d Ri = Eigen::Matrix3d::Identity();
        Eigen::Vector3d ptini;
        ptini<< dist*cos(15*M_PI/180)*cos(45*M_PI/180), -dist*sin(15*M_PI/180),  dist*cos(15*M_PI/180)*sin(45*M_PI/180);
        Eigen::Matrix3d Rj = Eigen::AngleAxisd(30*M_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix();

        Eigen::Vector3d pi =Eigen::Vector3d::Zero();
        Eigen::Vector3d pj =Eigen::Vector3d::Random();

        Eigen::Vector3d ptinj = Rj.transpose()*(ptini - pj);

        Eigen::Vector3d abrhoi =Eigen::Vector3d(ptini[0],ptini[1],1)/ptini[2];
        Eigen::Vector3d abrhoj;
        Eigen::Matrix<double, 3, 9> jacobian;

        reparameterize_AIDP(Ri, Rj, abrhoi, pi, pj, abrhoj, &jacobian);
        Eigen::Matrix<double, 3, 9> jacobian3;
        reparameterizeNumericalJacobian(Ri, Rj, abrhoi, pi, pj, abrhoj, jacobian3);
        std::cout<<"analytic jacobian " << std::endl<< jacobian<<std::endl;
        std::cout <<"analytic jacobian - numerical jacobian "<< std::endl << (jacobian - jacobian3) <<std::endl;
        std::cout <<"abrhoi and j"<< abrhoi.transpose()<< " "<< abrhoj.transpose()<< std::endl;
    }
    //infinity
    Eigen::Matrix3d Ri = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ptiniRay;
    ptiniRay<< cos(15*M_PI/180)*cos(45*M_PI/180), -sin(15*M_PI/180),  cos(15*M_PI/180)*sin(45*M_PI/180);
    Eigen::Matrix3d Rj = Eigen::AngleAxisd(30*M_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Vector3d pi =Eigen::Vector3d::Zero();
    Eigen::Vector3d pj =Eigen::Vector3d::Random();

    Eigen::Vector3d ptinjRay = Rj.transpose()*ptiniRay;
    ptinjRay/=ptinjRay[2];
    ptinjRay[2] =0;

    Eigen::Vector3d abrhoi =Eigen::Vector3d(1, -tan(15*M_PI/180)/sin(45*M_PI/180),0);
    Eigen::Vector3d abrhoj;
    Eigen::Matrix<double, 3, 9> jacobian;

    reparameterize_AIDP(Ri, Rj, abrhoi, pi, pj, abrhoj, &jacobian);
    Eigen::Matrix<double, 3, 9> jacobian3;
    reparameterizeNumericalJacobian(Ri, Rj, abrhoi, pi, pj, abrhoj, jacobian3);
    std::cout <<"infinity point case "<< std::endl;
    std::cout<<"analytic jacobian " << std::endl<< jacobian<<std::endl;
    std::cout <<"analytic jacobian - numerical jacobian "<< std::endl << (jacobian - jacobian3) <<std::endl;
    std::cout <<"abrhoi and j"<< abrhoi.transpose()<< " "<< abrhoj.transpose()<< std::endl;
    std::cout <<"expected abrhoj "<< ptinjRay.transpose()<<std::endl;
}

void testExtractBlocks()
{
    Eigen::MatrixXd m(5,5);
    m<< 1,2,3,4,5,
        6,7,8,9,10,
        11,12,13,14,15,
        16,17,18,19,20,
        21,22,23,24,25;

    std::vector<std::pair<size_t, size_t> > vRowStartInterval;
    for(size_t jack=0; jack<5; ++jack)
        vRowStartInterval.push_back(std::make_pair(jack, 1));
    // test deleting none entry
    Eigen::MatrixXd res = extractBlocks(m, vRowStartInterval, vRowStartInterval);
    assert((res- m).lpNorm<Eigen::Infinity>()<1e-8);

    // test deleting odd indexed rows/cols
    vRowStartInterval.clear();
    for(size_t jack=0; jack<5; jack+=2)
        vRowStartInterval.push_back(std::make_pair(jack, 1));
    res = extractBlocks(m, vRowStartInterval, vRowStartInterval);
    Eigen::MatrixXd expected(3,3);
    expected<< 1,3,5,
            11,13,15,
            21,23,25;
    assert((res- expected).lpNorm<Eigen::Infinity>()<1e-8);

// test deleting even indexed rows/cols
    vRowStartInterval.clear();
    for(size_t jack=1; jack<5; jack+=2)
        vRowStartInterval.push_back(std::make_pair(jack, 1));
    res = extractBlocks(m, vRowStartInterval, vRowStartInterval);
    Eigen::MatrixXd expected2(2,2);
    expected2<< 7,9,17,19;
    assert((res- expected2).lpNorm<Eigen::Infinity>()<1e-8);

// test with keeping more than 1 rows/cols each time
    vRowStartInterval.clear();
    vRowStartInterval.push_back(std::make_pair(0, 2));
    vRowStartInterval.push_back(std::make_pair(3, 2));
    res = extractBlocks(m, vRowStartInterval, vRowStartInterval);
    Eigen::MatrixXd expected3(4,4);
    expected3<<1,2,4,5,
            6,7,9,10,
            16,17,19,20,
            21,22,24,25;;
    assert((res- expected3).lpNorm<Eigen::Infinity>()<1e-8);

// test with different rows and cols to keep
    vRowStartInterval.clear();
    vRowStartInterval.push_back(std::make_pair(0, 2));
    vRowStartInterval.push_back(std::make_pair(3, 2));
    std::vector<std::pair<size_t, size_t> > vColStartInterval;
    vColStartInterval.push_back(std::make_pair(0,2));
    vColStartInterval.push_back(std::make_pair(3,1));
    res = extractBlocks(m, vRowStartInterval, vColStartInterval);
    Eigen::MatrixXd expected4(4,3);
    expected4<< 1,2,4,
            6,7,9,
            16,17,19,
            21,22,24;
    assert((res- expected4).lpNorm<Eigen::Infinity>()<1e-8);
}

Eigen::Vector3d unskew3d(const Eigen::Matrix3d & Omega) {
   return 0.5 * Eigen::Vector3d(Omega(2,1) - Omega(1,2), Omega(0,2) - Omega(2,0), Omega(1,0) - Omega(0,1));
}

}
