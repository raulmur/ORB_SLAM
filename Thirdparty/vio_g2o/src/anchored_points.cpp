
#include "vio_g2o/anchored_points.h"
#include "vio_g2o/transformations.h" //jacobians w.r.t projections, etc
#include "vio/eigen_utils.h"
#include "vio/maths_utils.h" //project2d, unproject2d, etc.

using namespace Eigen;
using namespace std; // for debugging output

namespace vio{

g2o::OptimizableGraph::Vertex*  GET_MAP_ELEM(const int & key,
                                             const g2o::OptimizableGraph::VertexIDMap & m)
{
    g2o::OptimizableGraph::VertexIDMap::const_iterator it = m.find(key);
    assert(it!=m.end());
    return dynamic_cast<g2o::OptimizableGraph::Vertex*>(it->second);
}

G2oCameraParameters
::G2oCameraParameters()
    : principle_point_(Eigen::Vector2d(0., 0.)),
      focal_length_(Eigen::Vector2d(1., 1.))
{
}

Eigen::Vector2d  G2oCameraParameters
::cam_map(const Eigen::Vector3d & trans_xyz) const
{
    Eigen::Vector2d proj = vio::project2d(trans_xyz);
    Eigen::Vector2d res;
    res[0] = proj[0]*focal_length_[0] + principle_point_[0];
    res[1] = proj[1]*focal_length_[1] + principle_point_[1];
    return res;
}

Eigen::Vector2d G2oCameraParameters::
normalize                    (const Eigen::Vector2d & uv) const
{
    return (uv- principle_point_).cwiseQuotient(focal_length_);
}


G2oStereoCameraParameters
::G2oStereoCameraParameters()
  : principle_point_(Eigen::Vector2d(0., 0.)),
    focal_length_(1.),
    baseline_(0.5)
{
}

Eigen::Vector2d  G2oStereoCameraParameters
::cam_map(const Eigen::Vector3d & trans_xyz) const
{
  Eigen::Vector2d proj = project2d(trans_xyz);
  Eigen::Vector2d res;
  res[0] = proj[0]*focal_length_ + principle_point_[0];
  res[1] = proj[1]*focal_length_ + principle_point_[1];
  return res;
}

Eigen::Vector3d G2oStereoCameraParameters
::stereocam_uvu_map(const Eigen::Vector3d & trans_xyz) const
{
  Eigen::Vector2d uv_left = cam_map(trans_xyz);
  double proj_x_right = (trans_xyz[0]-baseline_)/trans_xyz[2];
  double u_right = proj_x_right*focal_length_ + principle_point_[0];
  return Eigen::Vector3d(uv_left[0],uv_left[1],u_right);
}

Eigen::Matrix<double,2,6>
G2oCameraParameters::frameJac(const Sophus::SE3d & se3,
                              const Eigen::Vector3d & xyz)const
{
    const Eigen::Vector3d & xyz_trans = se3*xyz;
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double fx = focal_length_[0];
    double fy = focal_length_[1];

    double one_b_z = 1./z;
    double one_b_z_sq = 1./(z*z);
    double A = -fx*one_b_z;
    double B = -fy*one_b_z;
    double C = fx*x*one_b_z_sq;
    double D = fy*y*one_b_z_sq;

    Eigen::Matrix<double, 2, 6> jac;
    jac <<  A, 0, C, y*C,     -fx-x*C, -y*A,
            0, B, D, fy+y*D, -x*D,     x*B;
    return jac;
}
Eigen::Matrix<double,2,3>
G2oCameraParameters::pointJac(const Sophus::SE3d & T_cw,
                              const Eigen::Vector3d & xyz_w) const
{
    Eigen::Vector3d y = T_cw*xyz_w;
    Eigen::Matrix<double,2,3> J1
            = d_proj_d_y(focal_length_[0],y);
    return -J1*T_cw.rotationMatrix();
}

void G2oVertexSE3
::oplusImpl(const double * update_p)
{
    Eigen::Map<const Eigen::Matrix<double, 6, 1> > update(update_p);
    setEstimate(Sophus::SE3d::exp(update)*estimate());
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSE3
::write(std::ostream& os) const
{
    os<<"estimate qxyzw, txyz:"<< _estimate.unit_quaternion().coeffs().transpose()<<
        " "<< _estimate.translation().transpose()<<std::endl;
    return os.good();
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSE3
::read(std::istream& is)
{
    // input format qxyzw, txyz
    for(int jack=0; jack<7; ++jack)
        is >> _estimate.data()[jack];
    return is.good();
}



void G2oVertexPointXYZ
::oplusImpl(const double * update_p)
{
    Eigen::Map<const Eigen::Vector3d> update(update_p);
    _estimate += update;
}

bool G2oVertexPointXYZ
::write (std::ostream & os) const
{
    const Eigen::Vector3d & lv = estimate();
    for (int i=0; i<3; i++)
    {
        os << lv[i] << " ";
    }
    return true;
}

bool G2oVertexPointXYZ
::read(std::istream& is)
{
    Eigen::Vector3d lv;
    for (int i=0; i<3; i++)
    {
        is >> lv[i];
    }
    setEstimate(lv);
    return true;
}


bool G2oEdgeProjectPSI2UVU
::write(std::ostream& os) const
{
  for (int i=0; i<3; i++)
  {
    os  << measurement()[i] << " ";
    {
      for (int i=0; i<3; i++)
        for (int j=i; j<3; j++)
        {
          os << " " <<  information()(i,j);
        }
    }
  }
  return true;
}

bool G2oEdgeProjectPSI2UVU
::read(std::istream& is)
{
  assert(false);
  //  for (int i=0; i<3; i++)
  //  {
  //    is  >> measurement()[i];
  //  }
  //  inverseMeasurement()[0] = -measurement()[0];
  //  inverseMeasurement()[1] = -measurement()[1];
  //  inverseMeasurement()[2] = -measurement()[2];

  //  for (int i=0; i<3; i++)
  //    for (int j=i; j<3; j++)
  //    {
  //      is >> information()(i,j);
  //      if (i!=j)
  //        information()(j,i)=information()(i,j);
  //    }
  return true;
}

void G2oEdgeProjectPSI2UVU
::computeError()
{
  const G2oVertexPointXYZ * psi
      = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
  const G2oVertexSE3 * T_p_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[1]);
  const G2oVertexSE3 * T_anchor_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[2]);

  const G2oStereoCameraParameters * cam
      = static_cast<const G2oStereoCameraParameters *>(parameter(0));

  Eigen::Vector3d obs(_measurement);
  _error = obs - cam->stereocam_uvu_map(
        T_p_from_world->estimate()
        *T_anchor_from_world->estimate().inverse()
        *invert_depth(psi->estimate()));
}

void G2oEdgeProjectPSI2UVU::linearizeOplus()
{
  G2oVertexPointXYZ* vpoint = static_cast<G2oVertexPointXYZ*>(_vertices[0]);
  Eigen::Vector3d psi_a = vpoint->estimate();
  G2oVertexSE3 * vpose = static_cast<G2oVertexSE3 *>(_vertices[1]);
  Sophus::SE3d T_cw = vpose->estimate();
  G2oVertexSE3 * vanchor = static_cast<G2oVertexSE3 *>(_vertices[2]);
  const G2oStereoCameraParameters * cam
      = static_cast<const G2oStereoCameraParameters *>(parameter(0));

  Sophus::SE3d A_aw = vanchor->estimate();
  Sophus::SE3d T_ca = T_cw*A_aw.inverse();
  Eigen::Vector3d x_a = invert_depth(psi_a);
  Eigen::Vector3d y = T_ca*x_a;
  Eigen::Matrix3d Jcam
      = d_stereoproj_d_y(cam->focal_length_,
                         cam->baseline_,
                         y);
  _jacobianOplus[0] = -Jcam*d_Tinvpsi_d_psi(T_ca, psi_a);
  _jacobianOplus[1] = -Jcam*d_expy_d_y(y);
  _jacobianOplus[2] = Jcam*T_ca.rotationMatrix()*d_expy_d_y(x_a);
}

bool G2oEdgeSE3
::read(std::istream& is)
{
    //input qxyzw, txyz
    for(int jack=0; jack<7;++jack)
        is>> _measurement.data()[jack];
    return is.good();
}

bool G2oEdgeSE3
::write(std::ostream& os) const
{
    os<<"measurement qxyzw txyz:"<< _measurement.unit_quaternion().coeffs().transpose()<<
        " "<< _measurement.translation().transpose()<<std::endl;
    return os.good();
}

typedef Eigen::Matrix<double, 6, 6, Eigen::ColMajor> Matrix6d;

Matrix6d third(const Sophus::SE3d & A, const Eigen::Matrix<double, 6, 1> & d)
{
    const Matrix6d & AdjA = A.Adj();

    Matrix6d d_lie = Sophus::SE3d::d_lieBracketab_by_d_a(d);
    //cerr << d_lie << endl;
    return AdjA + 0.5*d_lie*AdjA + (1./12.)*d_lie*d_lie*AdjA;
}

void G2oEdgeSE3
::computeError()
{
    const G2oVertexSE3 * v1 = static_cast<const G2oVertexSE3 *>(_vertices[0]);
    const G2oVertexSE3 * v2 = static_cast<const G2oVertexSE3 *>(_vertices[1]);
    Sophus::SE3d T_21(_measurement);
    _error = (T_21*v1->estimate()*v2->estimate().inverse()).log();
}

void G2oEdgeSE3::
linearizeOplus()
{
    const Sophus::SE3d & T_21 = _measurement;
    Sophus::SE3d I;
    const Eigen::Matrix<double, 6, 1> & d = _error;
    _jacobianOplusXi = third(T_21, d);
    _jacobianOplusXj = -third(I, -d);

}
G2oEdgeProjectXYZ2XYZ::G2oEdgeProjectXYZ2XYZ() : g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
   
}
bool G2oEdgeProjectXYZ2XYZ::read(std::istream& is){
 
    for (int i=0; i<3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}
bool G2oEdgeProjectXYZ2XYZ::write(std::ostream& os) const {
   
    for (int i=0; i<3; i++){
        os << measurement()[i] << " ";
    }
    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
            os << " " << information()(i,j);
        }
    return os.good();
}

void G2oEdgeProjectXYZ2XYZ::linearizeOplus() {
    g2o::VertexSE3Expmap* vj = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
	g2o::SE3Quat T(vj->estimate());
 
    g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz_trans(T*vi->estimate());
 
	_jacobianOplusXi = - T.rotation().toRotationMatrix();
    Eigen::Matrix<double,3,6,Eigen::ColMajor> dX_to_dT;
	dX_to_dT.block(0,0,3,3)= g2o::skew(xyz_trans);
    dX_to_dT.block(0,3,3,3)= - Eigen::Matrix3d::Identity();// put identity here because g2o se3 is different from sophus se3
    _jacobianOplusXj =  dX_to_dT;
}
bool G2oEdgeProjectPSI2XYZ
::write(std::ostream& os) const
{
  for (int i=0; i<3; i++)
  {
    os  << measurement()[i] << " ";
    {
      for (int i=0; i<3; i++)
        for (int j=i; j<3; j++)
        {
          os << " " <<  information()(i,j);
        }
    }
  }
  return true;
}

bool G2oEdgeProjectPSI2XYZ
::read(std::istream& is)
{
  assert(false);
  //  for (int i=0; i<3; i++)
  //  {
  //    is  >> measurement()[i];
  //  }
  //  inverseMeasurement()[0] = -measurement()[0];
  //  inverseMeasurement()[1] = -measurement()[1];
  //  inverseMeasurement()[2] = -measurement()[2];

  //  for (int i=0; i<3; i++)
  //    for (int j=i; j<3; j++)
  //    {
  //      is >> information()(i,j);
  //      if (i!=j)
  //        information()(j,i)=information()(i,j);
  //    }
  return true;
}

void G2oEdgeProjectPSI2XYZ
::computeError()
{
  const G2oVertexPointXYZ * psi
      = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
  const G2oVertexSE3 * T_p_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[1]);
  const G2oVertexSE3 * T_anchor_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[2]);

  Vector3d obs(_measurement);
  _error = obs - T_p_from_world->estimate()
        *T_anchor_from_world->estimate().inverse()
        *invert_depth(psi->estimate());
}

void G2oEdgeProjectPSI2XYZ::linearizeOplus()
{
  G2oVertexPointXYZ* vpoint = static_cast<G2oVertexPointXYZ*>(_vertices[0]);
  Vector3d psi_a = vpoint->estimate();
  G2oVertexSE3 * vpose = static_cast<G2oVertexSE3 *>(_vertices[1]);
  Sophus::SE3d T_cw = vpose->estimate();
  G2oVertexSE3 * vanchor = static_cast<G2oVertexSE3 *>(_vertices[2]);
 
  Sophus::SE3d A_aw = vanchor->estimate();
  Sophus::SE3d T_ca = T_cw*A_aw.inverse();
  Vector3d x_a = invert_depth(psi_a);
  Vector3d y = T_ca*x_a;
  
  _jacobianOplus[0] = -d_Tinvpsi_d_psi(T_ca, psi_a);
  _jacobianOplus[1] = -d_expy_d_y(y);
  _jacobianOplus[2] = T_ca.rotationMatrix()*d_expy_d_y(x_a);
}

G2oEdgeProjectXYZ2UV::G2oEdgeProjectXYZ2UV() : BaseBinaryEdge<2, Eigen::Vector2d, G2oVertexPointXYZ, G2oVertexSE3>() {
    _cam = 0;
    resizeParameters(1);
    installParameter(_cam, 0);
}
bool G2oEdgeProjectXYZ2UV::read(std::istream& is){
    int paramId;
    is >> paramId;
    setParameterId(0, paramId);
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}
bool G2oEdgeProjectXYZ2UV::write(std::ostream& os) const {
    os << _cam->id() << " ";
    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " << information()(i,j);
        }
    return os.good();
}

void G2oEdgeProjectXYZ2UV::linearizeOplus() {
    G2oVertexSE3* vj = static_cast<G2oVertexSE3*>(_vertices[1]);
    Sophus::SE3d T(vj->estimate());
    if(vj->first_estimate!=NULL) T=*(vj->first_estimate);
    G2oVertexPointXYZ* vi = static_cast<G2oVertexPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz_trans((vi->first_estimate==NULL)?T*vi->estimate(): T*(*(vi->first_estimate)));

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    const G2oCameraParameters * cam = static_cast<const G2oCameraParameters *>(parameter(0));
    Eigen::Matrix<double,2,3,Eigen::ColMajor> tmp;
    tmp(0,0) = cam->focal_length_[0];
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*cam->focal_length_[0];
    tmp(1,0) = 0;
    tmp(1,1) = cam->focal_length_[1];
    tmp(1,2) = -y/z*cam->focal_length_[1];

    _jacobianOplusXi = -1./z * tmp * T.rotationMatrix();

//    std::cout<<"J1 original: "<< endl<< _jacobianOplusXi<<std::endl;
//    cout<<"J1 pointJac : "<< endl<< cam->pointJac(T, vi->estimate())<<endl;
    Eigen::Matrix<double,3,6,Eigen::ColMajor> dX_to_dT;
    dX_to_dT.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
    dX_to_dT.block<3,3>(0,3)= - vio::skew3d(xyz_trans);
    _jacobianOplusXj = -1./z * tmp * dX_to_dT;

//    cout<<"J2 original: "<< endl<< _jacobianOplusXj<<endl;
//    cout<<"J2 frameJac : "<< endl<< cam->frameJac(T, vi->estimate())<<endl;
}
void G2oExEdgeProjectXYZ2UV::linearizeOplus() {
    G2oVertexSE3* vj = static_cast<G2oVertexSE3*>(_vertices[1]);
    Sophus::SE3d vjestimate=(vj->first_estimate==NULL? vj->estimate(): *(vj->first_estimate));
    Sophus::SE3d T(Ts2c*vjestimate);
    G2oVertexPointXYZ* vi = static_cast<G2oVertexPointXYZ*>(_vertices[0]);
    Eigen::Vector3d viestimate= (vi->first_estimate==NULL? vi->estimate(): *(vi->first_estimate));
    Eigen::Vector3d xyz_trans = T*viestimate;
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    const G2oCameraParameters * cam = static_cast<const G2oCameraParameters *>(parameter(0));
    Eigen::Matrix<double,2,3,Eigen::ColMajor> tmp;
    tmp(0,0) = cam->focal_length_[0];
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*cam->focal_length_[0];
    tmp(1,0) = 0;
    tmp(1,1) = cam->focal_length_[1];
    tmp(1,2) = -y/z*cam->focal_length_[1];
    _jacobianOplusXi = -1./z * tmp * T.rotationMatrix();
    Eigen::Matrix<double,3,6,Eigen::ColMajor> dX_to_dT;
    dX_to_dT.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
    dX_to_dT.block<3,3>(0,3)= - vio::skew3d(vjestimate*viestimate);
    _jacobianOplusXj = -1./z * tmp *Ts2c.rotationMatrix()* dX_to_dT;
}

G2oEdgeProjectXYZ2UV2::G2oEdgeProjectXYZ2UV2(G2oCameraParameters * cam) : BaseBinaryEdge<2, Eigen::Vector2d, G2oVertexPointXYZ, G2oVertexSE3>() {
    _cam = cam==NULL? NULL: new G2oCameraParameters(*cam);
}

bool G2oEdgeProjectXYZ2UV2::read(std::istream& is){
    int paramId;
    is >> paramId;
    setParameterId(0, paramId);
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool G2oEdgeProjectXYZ2UV2::write(std::ostream& os) const {
    os << _cam->id() << " ";
    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " << information()(i,j);
        }
    return os.good();
}

void G2oEdgeProjectXYZ2UV2::linearizeOplus() {
    G2oVertexSE3* vj = static_cast<G2oVertexSE3*>(_vertices[1]);
    Sophus::SE3d T(vj->estimate());
    if(vj->first_estimate!=NULL) T=*(vj->first_estimate);
    G2oVertexPointXYZ* vi = static_cast<G2oVertexPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz_trans((vi->first_estimate==NULL)?T*vi->estimate(): T*(*(vi->first_estimate)));

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    const G2oCameraParameters * cam = _cam;
    Eigen::Matrix<double,2,3,Eigen::ColMajor> tmp;
    tmp(0,0) = cam->focal_length_[0];
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*cam->focal_length_[0];
    tmp(1,0) = 0;
    tmp(1,1) = cam->focal_length_[1];
    tmp(1,2) = -y/z*cam->focal_length_[1];

    _jacobianOplusXi = -1./z * tmp * T.rotationMatrix();

    Eigen::Matrix<double,3,6,Eigen::ColMajor> dX_to_dT;
    dX_to_dT.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
    dX_to_dT.block<3,3>(0,3)= - vio::skew3d(xyz_trans);
    _jacobianOplusXj = -1./z * tmp * dX_to_dT;

}

void G2oEdgeProjectXYZ2UV2::setCameraParameters(G2oCameraParameters & cam){
    if(_cam)
        delete _cam;
    _cam= new G2oCameraParameters(cam);
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d xyz = vi->estimate();
    Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    Matrix<double,2,3> tmp;
    tmp(0,0) = fx;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*fx;

    tmp(1,0) = 0;
    tmp(1,1) = fy;
    tmp(1,2) = -y/z*fy;

    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
    _jacobianOplusXj(0,2) = y/z *fx;
    _jacobianOplusXj(0,3) = -1./z *fx;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2 *fx;

    _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
    _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
    _jacobianOplusXj(1,2) = -x/z *fy;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *fy;
    _jacobianOplusXj(1,5) = y/z_2 *fy;
}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = vio::project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0]*fx + cx;
    res[1] = proj[1]*fy + cy;
    return res;
}
// copied from Raul ORB_SLAM /Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.cpp
// This function was commented both in ORB_SLAM and github g2o for no clear reason
// note that g2o::Sim3 tangential space [omega, upsilon, sigma] determining [rotation, translation, scale]
// in contrast Sophus sim3 tangential space is [upsilon, omega, sigma]
// The exact Jacobian expression can be found in Strasdat's dissertation
/* $J_0=\frac{\partial \left\{ \begin{bmatrix} u
\\ v \end{bmatrix}- \textup{proj}\left[\mathbf{K}_1 \textup{proj}(\mathbf{S}_w^c \dot{p}^w) \right ]\right\}}{\partial [p^w]} = -\begin{bmatrix}
\frac{-f_x}{z} & 0 & \frac{f_xx}{z^2}\\
0 & \frac{-f_y}{z} & \frac{f_yy}{z^2}\end{bmatrix}s\mathbf{R}$*/
/*$J_1=\frac{\partial \left\{ \begin{bmatrix} u
\\ v \end{bmatrix}- \textup{proj}\left[\mathbf{K}_1 \textup{proj}(\mathbf{S}_w^c \dot{p}^w) \right ]\right\}}{\partial [\upsilon, \omega, \sigma]]} = -\begin{bmatrix}
\frac{-f_x}{z} & 0 & \frac{f_xx}{z^2}\\
0 & \frac{-f_y}{z} & \frac{f_yy}{z^2}\end{bmatrix}\begin{bmatrix}
\mathbf{I}_3 & -[\mathbf{x}]_\times & \mathbf{x}
\end{bmatrix}$ where $\mathbf{x}= s\mathbf{R}p^w+\mathbf{t}$*/

//  void EdgeSim3ProjectXYZ::linearizeOplus()
//  {
//    VertexSim3Expmap * vj = static_cast<VertexSim3Expmap *>(_vertices[1]);
//    Sim3 T = vj->estimate();

//    VertexPointXYZ* vi = static_cast<VertexPointXYZ*>(_vertices[0]);
//    Vector3d xyz = vi->estimate();
//    Vector3d xyz_trans = T.map(xyz);

//    double x = xyz_trans[0];
//    double y = xyz_trans[1];
//    double z = xyz_trans[2];
//    double z_2 = z*z;

//    Matrix<double,2,3> tmp;
//    tmp(0,0) = _focal_length(0);
//    tmp(0,1) = 0;
//    tmp(0,2) = -x/z*_focal_length(0);

//    tmp(1,0) = 0;
//    tmp(1,1) = _focal_length(1);
//    tmp(1,2) = -y/z*_focal_length(1);

//    _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();// scale is ignored

//    _jacobianOplusXj(0,0) =  x*y/z_2 * _focal_length(0);
//    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *_focal_length(0);
//    _jacobianOplusXj(0,2) = y/z *_focal_length(0);
//    _jacobianOplusXj(0,3) = -1./z *_focal_length(0);
//    _jacobianOplusXj(0,4) = 0;
//    _jacobianOplusXj(0,5) = x/z_2 *_focal_length(0);
//    _jacobianOplusXj(0,6) = 0; // scale is ignored


//    _jacobianOplusXj(1,0) = (1+y*y/z_2) *_focal_length(1);
//    _jacobianOplusXj(1,1) = -x*y/z_2 *_focal_length(1);
//    _jacobianOplusXj(1,2) = -x/z *_focal_length(1);
//    _jacobianOplusXj(1,3) = 0;
//    _jacobianOplusXj(1,4) = -1./z *_focal_length(1);
//    _jacobianOplusXj(1,5) = y/z_2 *_focal_length(1);
//    _jacobianOplusXj(1,6) = 0; // scale is ignored
//  }

#ifdef MONO

G2oVertexSim3
::G2oVertexSim3()
  : principle_point(Vector2d(0., 0.)),
    focal_length(1.)
{
}

void G2oVertexSim3
::oplusImpl(const double * update_p)
{
  Eigen::Map<Eigen::Matrix<double,7,1> > update(const_cast<double*>(update_p));
  _estimate = Sophus::Sim3d::exp(update)*estimate();
}

Vector2d  G2oVertexSim3
::cam_map(const Vector2d & v) const
{
  Vector2d res;
  res[0] = v[0]*focal_length + principle_point[0];
  res[1] = v[1]*focal_length + principle_point[1];
  return res;
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSim3
::write(std::ostream& os) const
{
  assert(false);
  return true;
}

//TODO: implement, but first remove camera parameters from vertex state
bool G2oVertexSim3
::read(std::istream& is)
{
  assert(false);
  return true;
}

void G2oEdgeSim3ProjectUVQ::
computeError()
{
  const G2oVertexPointXYZ* psi
      = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
  const G2oVertexSE3* T_p_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[1]);
  const G2oVertexSE3* T_anchor_from_world
      = static_cast<const G2oVertexSE3*>(_vertices[2]);

  // Huai: Strasdat's implementation has an error and I revise it this way
  // Note an alternative correction is alluded by the fact that G2oVertexSE3 has a cam_map function
  const G2oCameraParameters * cam
      = static_cast<const G2oCameraParameters *>(parameter(0));

  Vector2d obs(_measurement);
  _error = obs-cam->cam_map(
        T_p_from_world->estimate()
        *T_anchor_from_world->estimate().inverse()
        *invert_depth(psi->estimate()));
}

bool G2oEdgeSim3ProjectUVQ
::read(std::istream & is)
{
  for (int i=0; i<2; i++)
  {
    is >> _measurement[i];
  }
//  inverseMeasurement()[0] = -measurement()[0];
//  inverseMeasurement()[1] = -measurement()[1];
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++)
    {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i) = information()(i,j);
    }
  return true;
}

bool  G2oEdgeSim3ProjectUVQ
::write(std::ostream& os) const
{
  for (int i=0; i<2; i++)
  {
    os  << measurement()[i] << " ";
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++)
    {
      os << " " <<  information()(i,j);
    }
  return true;
}

bool G2oEdgeSim3
::read(std::istream& is)
{
  Eigen::Matrix<double, 7, 1>  v7;
  for (int i=0; i<7; i++)
  {
    is >> v7[i];
  }
  Sophus::Sim3d cam2world  = Sophus::Sim3d::exp(v7);
  _measurement = cam2world.inverse();
//  inverseMeasurement() = cam2world;
  for (int i=0; i<7; i++)
    for (int j=i; j<7; j++)
    {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i) = information()(i,j);
    }
  return is.good();
}

bool G2oEdgeSim3
::write(std::ostream& os) const
{
  Sophus::Sim3d cam2world(measurement().inverse());
  Eigen::Matrix<double, 7, 1>  v7 = cam2world.log();
  for (int i=0; i<7; i++)
  {
    os  << v7[i] << " ";
  }
  for (int i=0; i<7; i++)
    for (int j=i; j<7; j++){
      os << " " <<  information()(i,j);
    }
  return true;
}


void G2oEdgeSim3
::computeError()
{
  const G2oVertexSim3* v1 = static_cast<const G2oVertexSim3*>(_vertices[0]);
  const G2oVertexSim3* v2 = static_cast<const G2oVertexSim3*>(_vertices[1]);
  Sophus::Sim3d C(_measurement);
  Sophus::Sim3d error_= v2->estimate().inverse()*C*v1->estimate();
  _error = error_.log().head<6>();

}

#endif

}
