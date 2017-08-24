
#ifndef G2O_ANCHORED_POINTS_H
#define G2O_ANCHORED_POINTS_H

#include "vio/maths_utils.h" //for project2d

#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#ifdef USE_SLIM_G2O //this only works with the slim g2o in ORB_SLAM2/Thirdparty
#include <g2o/types/types_sba.h> // EdgeSE3ProjectXYZ
#include <g2o/types/types_six_dof_expmap.h> //EdgeSE3ProjectXYZ
#include <g2o/types/types_seven_dof_expmap.h> //EdgeInverseSim3ProjectXYZ
#else //this works with the a full g2o, e.g., the Kummerle's g2o on github
#include <g2o/types/sba/types_sba.h> // EdgeSE3ProjectXYZ
#include <g2o/types/sba/types_six_dof_expmap.h> //EdgeSE3ProjectXYZ
#include <g2o/types/sim3/types_seven_dof_expmap.h> //EdgeInverseSim3ProjectXYZ
#endif

#include<Eigen/Core>

namespace vio{

Eigen::Matrix<double, 6,6> third(const Sophus::SE3d & A, const Eigen::Matrix<double,6,1> & d);

g2o::OptimizableGraph::Vertex*  GET_MAP_ELEM(const int & key,
                                             const g2o::OptimizableGraph::VertexIDMap & m);

class G2oCameraParameters : public g2o::Parameter
{
public:
    G2oCameraParameters();

    G2oCameraParameters        (const Eigen::Vector2d & principle_point,
                                const Eigen::Vector2d & focal_length)
        : principle_point_(principle_point),
          focal_length_(focal_length)
    {}

    Eigen::Vector2d
    cam_map                    (const Eigen::Vector3d & trans_xyz) const;

    Eigen::Vector2d
    normalize                    (const Eigen::Vector2d & uv) const;

    virtual bool
    read                       (std::istream& is)
    {
        is>>focal_length_[0] >> focal_length_[1] >> principle_point_[0] >> principle_point_[1];
        return is.good();
    }

    virtual bool
    write                      (std::ostream& os) const
    {
        os<<"focal length xy:"<<focal_length_[0] <<" "<< focal_length_[1] <<
            " principle point xy:"<< principle_point_[0] <<" "<< principle_point_[1]<<std::endl;
        return os.good();

    }

    Eigen::Matrix<double,2,6>
    frameJac(const Sophus::SE3d & se3,
             const Eigen::Vector3d & xyz)const;
    Eigen::Matrix<double,2,3>
    pointJac(const Sophus::SE3d & T_cw,
             const Eigen::Vector3d & xyz_w) const;

    Eigen::Vector2d principle_point_;
    Eigen::Vector2d focal_length_;

};

// The below definition of G2oStereoCameraParameters is exact the G2oCameraParameters
// that is used in ScaViSLAM, and perhaps used in huai's test program SE3Optimization and sparseba.
// It is renamed by adding a prefix "stereo" compared to the above definition which is used in
// other later programs developed by Huai. In case these old codebase
// surface, this definition here is for reference.
//Note g2o implements g2o::CameraParameters class which is almost identical to this stereo one

class G2oStereoCameraParameters : public g2o::Parameter
{
public:
  G2oStereoCameraParameters();

  G2oStereoCameraParameters(const Eigen::Vector2d & principle_point,
                              double focal_length,
                              double baseline)
    : principle_point_(principle_point),
      focal_length_(focal_length),
      baseline_(baseline){}

  Eigen::Vector2d
  cam_map                    (const Eigen::Vector3d & trans_xyz) const;

  Eigen::Vector3d
  stereocam_uvu_map          (const Eigen::Vector3d & trans_xyz) const;

  virtual bool read (std::istream& is){
    is >> focal_length_;
    is >> principle_point_[0];
    is >> principle_point_[1];
    is >> baseline_;
    return true;
  }

  virtual bool write (std::ostream& os) const {
    os << focal_length_ << " ";
    os << principle_point_.x() << " ";
    os << principle_point_.y() << " ";
    os << baseline_ << " ";
    return true;
  }

  Eigen::Vector2d principle_point_;
  double focal_length_;
  double baseline_;
};

class G2oVertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexSE3               ():first_estimate(NULL){}
    ~G2oVertexSE3               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);

    virtual bool
    write                      (std::ostream& os) const;

    virtual void
    oplusImpl                  (const double * update);

    virtual void
    setToOriginImpl            () {_estimate= Sophus::SE3d();}
    void setFirstEstimate(const Sophus::SE3d& fe){
        first_estimate=new Sophus::SE3d(fe);
    }
    //for first estimate Jacobians
    Sophus::SE3d* first_estimate;
};
//This vertex can be used to represent both Euclidean coordinates XYZ and inverse depth coordinates (X/Z, Y/Z, 1/Z)
//Please check its related G2oEdges to tell which representation is used.
class G2oVertexPointXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexPointXYZ          ():first_estimate(NULL){}
    ~G2oVertexPointXYZ               (){
        if(first_estimate)
            delete first_estimate;
        first_estimate=NULL;
    }
    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;
    virtual void
    oplusImpl                  (const double * update);

    virtual void
    setToOriginImpl            ()
    {
        _estimate.setZero();
    }
    void setFirstEstimate(const Eigen::Vector3d& fe){
        first_estimate=new Eigen::Vector3d(fe);
    }
    Eigen::Vector3d* first_estimate;
};

class G2oEdgeProjectPSI2UVU : public  g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  G2oEdgeProjectPSI2UVU()
  {
    resizeParameters(1);
    installParameter(g2o_cam, 0);
  }

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  void
  computeError               ();
  virtual
  void linearizeOplus        ();

  G2oStereoCameraParameters * g2o_cam;
};

// This edge uses a universal camera for all such edges in a LS problem, the camera is configured at setting up the problem by addParameter
// and in construciing such an edge, make sure calling resizeParameters, installParameter, and, setParameterId, so that
// the _cam can be resolved in addEdge through resolveParameters
class G2oEdgeProjectXYZ2UV : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, G2oVertexPointXYZ, G2oVertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    G2oEdgeProjectXYZ2UV();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    virtual void computeError() {
        const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
        const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
        const G2oCameraParameters* cam
                = static_cast<const G2oCameraParameters *>(parameter(0));
        _error = _measurement-cam->cam_map(v1->estimate()*(v2->estimate()));
    }
    virtual bool isDepthPositive() const {
      const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
      const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
      return (v1->estimate()*v2->estimate())(2)>0.0;
    }
    virtual void linearizeOplus();
    G2oCameraParameters * _cam;
};

// another version of G2oEdgeProjectXYZ2UV, the nuance is this version can use a different camera for each edge,
// By design, the _cam is created for each edge, so it is unnecessary to addParameter to the g2o graph or installParameter for this edge
class G2oEdgeProjectXYZ2UV2 : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, G2oVertexPointXYZ, G2oVertexSE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //if the input argument cam is NULL, you will want to setCameraParameters later on
    G2oEdgeProjectXYZ2UV2(G2oCameraParameters * cam=NULL); 
    ~G2oEdgeProjectXYZ2UV2(){delete _cam;}
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    virtual void computeError() {
        const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
        const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);      
        _error = _measurement- _cam->cam_map(v1->estimate()*(v2->estimate()));
    }
    virtual bool isDepthPositive() const {
        const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
        const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
        return (v1->estimate()*v2->estimate())(2)>0.0;
    }
    virtual void linearizeOplus();
    void setCameraParameters(G2oCameraParameters & cam);

    G2oCameraParameters * _cam;
};

// this derived class can deal with projections of a camera w.r.t a reference camera
class G2oExEdgeProjectXYZ2UV : public G2oEdgeProjectXYZ2UV{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    G2oExEdgeProjectXYZ2UV(const Sophus::SE3d & Tcs): G2oEdgeProjectXYZ2UV(), Ts2c(Tcs){}
    void computeError() {
        const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
        const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
        const G2oCameraParameters* cam
                = static_cast<const G2oCameraParameters *>(parameter(0));
        _error = _measurement-cam->cam_map(Ts2c*v1->estimate()*v2->estimate());
    }
    bool isDepthPositive() const {
      const G2oVertexSE3* v1 = static_cast<const G2oVertexSE3*>(_vertices[1]);
      const G2oVertexPointXYZ* v2 = static_cast<const G2oVertexPointXYZ*>(_vertices[0]);
      return (Ts2c*v1->estimate()*v2->estimate())(2)>0.0;
    }
    void linearizeOplus();
    Sophus::SE3d Ts2c; // transformation from a reference sensor frame (e.g., the left camera)
    //to this camera (e.g., the right camera)frame into which the point is projected
};


class G2oEdgeSim3ProjectUVQ : public  g2o::BaseMultiEdge<2, Eigen::Vector2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeSim3ProjectUVQ      ()
    {
        resizeParameters(1);
        installParameter(g2o_cam, 0);
    }

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;

    void
    computeError               ();

    G2oCameraParameters * g2o_cam;
};

class G2oEdgeSE3
        : public g2o::BaseBinaryEdge<6, Sophus::SE3d, G2oVertexSE3, G2oVertexSE3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeSE3                 () {}

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;
    void
    computeError               ();
    virtual void
    linearizeOplus             ();
};

//copied from Raul ORB_SLAM /Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h
class EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs-cam_project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate()))(2)>0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    double fx, fy, cx, cy;
};

//copied from Raul ORB_SLAM /Thirdparty/g2o/g2o/types/sim3/types_seven_dof_expmap.h
class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSim3Expmap(): g2o::BaseVertex<7, g2o::Sim3>(), _principle_point1(Eigen::Vector2d(0., 0.)),
        _principle_point2(Eigen::Vector2d(0., 0.)),_focal_length1(Eigen::Vector2d(1., 1.)),
        _focal_length2(Eigen::Vector2d(1., 1.))
    {
        _marginalized=false;
        _fix_scale = false;
    }

    bool read(std::istream& is)
    {
        Eigen::Matrix<double,7,1> cam2world;
        for (int i=0; i<6; i++){
            is >> cam2world[i];
        }
        is >> cam2world[6];

        for (int i=0; i<2; i++)
        {
            is >> _focal_length1[i];
        }
        for (int i=0; i<2; i++)
        {
            is >> _principle_point1[i];
        }

        setEstimate(g2o::Sim3(cam2world).inverse());
        return true;
    }

    bool write(std::ostream& os) const
    {
        g2o::Sim3 cam2world(estimate().inverse());
        Eigen::Matrix<double,7,1> lv=cam2world.log();
        for (int i=0; i<7; i++){
            os << lv[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _focal_length1[i] << " ";
        }
        for (int i=0; i<2; i++)
        {
            os << _principle_point1[i] << " ";
        }
        return os.good();
    }

    virtual void setToOriginImpl() {
        _estimate = g2o::Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
        Eigen::Map<Eigen::Matrix<double,7,1> > update(const_cast<double*>(update_));

        if (_fix_scale)
            update[6] = 0;

        g2o::Sim3 s(update);
        setEstimate(s*estimate());
    }

    Eigen::Vector2d cam_map1(const Eigen::Vector2d & v) const
    {
        Eigen::Vector2d res;
        res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
        res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
        return res;
    }

    Eigen::Vector2d cam_map2(const Eigen::Vector2d & v) const
    {
        Eigen::Vector2d res;
        res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
        res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
        return res;
    }

    Eigen::Vector2d _principle_point1, _principle_point2;
    Eigen::Vector2d _focal_length1, _focal_length2;

    bool _fix_scale;

protected:
};
/**
* \brief 7D edge between two Vertex7
*/
class EdgeSim3 : public g2o::BaseBinaryEdge<7, g2o::Sim3, VertexSim3Expmap, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3() : BaseBinaryEdge<7, g2o::Sim3, VertexSim3Expmap, VertexSim3Expmap>()
    {
    }
    bool read(std::istream& is)
    {
        Eigen::Matrix<double,7,1> v7;
        for (int i=0; i<7; i++){
            is >> v7[i];
        }

        g2o::Sim3 cam2world(v7);
        setMeasurement(cam2world.inverse());

        for (int i=0; i<7; i++)
            for (int j=i; j<7; j++)
            {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool write(std::ostream& os) const
    {
        g2o::Sim3 cam2world(measurement().inverse());
        Eigen::Matrix<double,7,1> v7 = cam2world.log();
        for (int i=0; i<7; i++)
        {
            os  << v7[i] << " ";
        }
        for (int i=0; i<7; i++)
            for (int j=i; j<7; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }
    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

        g2o::Sim3 C(_measurement);
        g2o::Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
        _error = error_.log();
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* /*to*/)
    {
        VertexSim3Expmap* v1 = static_cast<VertexSim3Expmap*>(_vertices[0]);
        VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
        if (from.count(v1) > 0)
            v2->setEstimate(measurement()*v1->estimate());
        else
            v1->setEstimate(measurement().inverse()*v2->estimate());
    }
};
/**/
class EdgeSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSim3ProjectXYZ() :  g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
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

    bool write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->cam_map1(vio::project2d(v1->estimate().map(v2->estimate())));
    }
    // virtual void linearizeOplus(); //if not implemented, g2o does numeric differentiation

};
class EdgeInverseSim3ProjectXYZ : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInverseSim3ProjectXYZ() :
        g2o::BaseBinaryEdge<2, Eigen::Vector2d,  g2o::VertexSBAPointXYZ, VertexSim3Expmap>()
    {
    }

    bool read(std::istream& is)
    {
        for (int i=0; i<2; i++)
        {
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

    bool write(std::ostream& os) const
    {
        for (int i=0; i<2; i++){
            os  << _measurement[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void computeError()
    {
        const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

        Eigen::Vector2d obs(_measurement);
        _error = obs-v1->cam_map2(vio::project2d(v1->estimate().inverse().map(v2->estimate())));
    }
    // virtual void linearizeOplus(); //if not implemented, g2o does numeric differentiation

};

class G2oEdgeProjectXYZ2XYZ : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    G2oEdgeProjectXYZ2XYZ();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    virtual void computeError() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
       
        _error = _measurement-v1->estimate()*(v2->estimate());
    }
    virtual bool isDepthPositive() const {
      const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
      const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
      return (v1->estimate()*v2->estimate())(2)>0.0;
    }
    virtual void linearizeOplus();
  
};

class G2oEdgeProjectPSI2XYZ : public g2o::BaseMultiEdge<3, Eigen::Vector3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	G2oEdgeProjectPSI2XYZ(){}
    bool read(std::istream& is);
    bool write(std::ostream& os) const;
    virtual void computeError();    
    virtual void linearizeOplus();  
};

#ifndef USE_SLIM_G2O //g2o::CameraParameters not defined in ORBSLAM G2O
//Stereo Observations
// U: left u
// V: left v
// U: right u
class G2oEdgeProjectXYZ2UVU : public  g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
    G2oEdgeProjectXYZ2UVU()
    {}

    bool read(std::istream& is){
      for (int i=0; i<3; i++){
        is  >> _measurement[i];
      }
      for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
          is >> information()(i,j);
          if (i!=j)
            information()(j,i)=information()(i,j);
        }
      return true;
    }

    bool write(std::ostream& os) const {
      for (int i=0; i<3; i++){
        os  << measurement()[i] << " ";
      }

      for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
          os << " " << information()(i,j);
        }
      return os.good();
    }

    void computeError(){
      const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
      const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
  
      Eigen::Vector3d obs(_measurement);
      _error = obs-cam_->stereocam_uvu_map(v1->estimate().map(v2->estimate()));
    }
    //  virtual void linearizeOplus();
    g2o::CameraParameters * cam_;
};
#endif

#ifdef MONO
class G2oVertexSim3 : public g2o::BaseVertex<6, Sophus::Sim3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  G2oVertexSim3              ();

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  virtual void
  oplusImpl                      (const double * update);
  Eigen::Vector2d
  cam_map                    (const Eigen::Vector2d & v) const;

  virtual void
  setToOriginImpl            () {_estimate= Sophus::Sim3d();}

  Eigen::Vector2d principle_point;
  double focal_length;
};

class G2oEdgeSim3
    : public g2o::BaseBinaryEdge<6, Sophus::Sim3d, G2oVertexSim3, G2oVertexSim3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  G2oEdgeSim3                (){}

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  void computeError          ();

};
#endif
typedef G2oVertexSE3 g2oFrameSE3;
typedef G2oVertexPointXYZ g2oPoint;
//typedef g2o::VertexSE3Expmap g2oFrameSE3;
//typedef g2o::VertexSBAPointXYZ g2oPoint;
}

#endif
