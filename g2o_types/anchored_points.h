#ifndef SCAVISLAM_G2O_ANCHORED_POINTS_H
#define SCAVISLAM_G2O_ANCHORED_POINTS_H

#include "maths_utils.h" //for project2d

#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_sba.h> // EdgeSE3ProjectXYZ
#include <g2o/types/sba/types_six_dof_expmap.h> //EdgeSE3ProjectXYZ
#include <g2o/types/sim3/types_seven_dof_expmap.h> //EdgeInverseSim3ProjectXYZ

#include<Eigen/Core>

namespace ScaViSLAM{

Eigen::Matrix<double, 6,6> third(const Sophus::SE3d & A, const Eigen::Matrix<double,6,1> & d);

//$\frac{\partial Kproj(X)}{\partial X}$
inline Eigen::Matrix<double,2,3>
d_proj_d_y(const Eigen::Vector2d & f, const Eigen::Vector3d & xyz)
{
  double z_sq = xyz[2]*xyz[2];
  Eigen::Matrix<double,2,3> J;
  J << f[0]/xyz[2], 0,           -(f[0]*xyz[0])/z_sq,
      0,           f[1]/xyz[2], -(f[1]*xyz[1])/z_sq;
  return J;
}
//$\frac{\partial Kproj(X)}{\partial X}$
inline Eigen::Matrix<double,2,3>
d_proj_d_y(const double & f, const Eigen::Vector3d & xyz)
{
  double z_sq = xyz[2]*xyz[2];
  Eigen::Matrix<double,2,3> J;
  J << f/xyz[2], 0,           -(f*xyz[0])/z_sq,
      0,           f/xyz[2], -(f*xyz[1])/z_sq;
  return J;
}

// this d_expy_d_y definition follows Strasdat's derivation in his dissertation
// It is also confirmed in github g2o: g2o/types/sba/types_six_dof_expmap.cpp
// though N.B. g2o used SE3Quat::exp([\omega, \upsilon]), Strasdat used SE3d::exp([\upsilon, \omega])
// compute $\frac{\partial proj(exp(\epsilon)y)}{\partial \epsilon}$
inline Eigen::Matrix<double,3,6>
d_expy_d_y(const Eigen::Vector3d & y)
{
  Eigen::Matrix<double,3,6> J;
  J.topLeftCorner<3,3>().setIdentity();
  J.bottomRightCorner<3,3>() = -Sophus::SO3::hat(y);
  return J;
}
// $\frac{\partial proj(T \psi^{-1})}{\partial \psi}$
inline Eigen::Matrix3d
d_Tinvpsi_d_psi(const Sophus::SE3d & T, const Eigen::Vector3d & psi)
{
  Eigen::Matrix3d R = T.rotationMatrix();
  Eigen::Vector3d x = invert_depth(psi);
  Eigen::Vector3d r1 = R.col(0);
  Eigen::Vector3d r2 = R.col(1);
  Eigen::Matrix3d J;
  J.col(0) = r1;
  J.col(1) = r2;
  J.col(2) = -R*x;
  J*=1./psi.z();
  return J;
}
//$\frac{\partial \tilde{u}- Kproj(RX+t)}{\partial X}$
inline void
point_jac_xyz2uv(const Eigen::Vector3d & xyz,
                 const Eigen::Matrix3d & R,
                 const double & focal_length,
                 Eigen::Matrix<double,2,3> & point_jac)
{
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  Eigen::Matrix<double,2,3> tmp;
  tmp(0,0) = focal_length;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*focal_length;
  tmp(1,0) = 0;
  tmp(1,1) = focal_length;
  tmp(1,2) = -y/z*focal_length;
  point_jac =  -1./z * tmp * R;
}

//$\frac{\partial \tilde{u}- Kproj(exp(\hat{\epsilon}))X)}{\partial \epsilon}$
inline void
frame_jac_xyz2uv(const Eigen::Vector3d & xyz,
                 const double & focal_length,
                 Eigen::Matrix<double,2,6> & frame_jac)
{
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  double z_2 = z*z;

  frame_jac(0,0) = -1./z *focal_length;
  frame_jac(0,1) = 0;
  frame_jac(0,2) = x/z_2 *focal_length;
  frame_jac(0,3) =  x*y/z_2 * focal_length;
  frame_jac(0,4) = -(1+(x*x/z_2)) *focal_length;
  frame_jac(0,5) = y/z *focal_length;

  frame_jac(1,0) = 0;
  frame_jac(1,1) = -1./z *focal_length;
  frame_jac(1,2) = y/z_2 *focal_length;
  frame_jac(1,3) = (1+y*y/z_2) *focal_length;
  frame_jac(1,4) = -x*y/z_2 *focal_length;
  frame_jac(1,5) = -x/z *focal_length;
}

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
  virtual bool//SCAVISLAM
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
 //    if (! is) {
 //      // if the scale is not specified we set it to 1;
 //      std::cerr << "!s";
 //      cam2world[6]=0.;
 //    }

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
       _error = obs-v1->cam_map1(project2d(v1->estimate().map(v2->estimate())));
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
      _error = obs-v1->cam_map2(project2d(v1->estimate().inverse().map(v2->estimate())));
    }
   // virtual void linearizeOplus(); //if not implemented, g2o does numeric differentiation

};

}

#endif
