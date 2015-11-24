// This file is part of ScaViSLAM.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// ScaViSLAM is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// ScaViSLAM is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with ScaViSLAM.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SCAVISLAM_G2O_ANCHORED_POINTS_H
#define SCAVISLAM_G2O_ANCHORED_POINTS_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_sba.h> // EdgeSE3ProjectXYZ
#include <g2o/types/sba/types_six_dof_expmap.h> //EdgeSE3ProjectXYZ
#include <g2o/types/sim3/types_seven_dof_expmap.h> //EdgeInverseSim3ProjectXYZ

#include <sophus/se3.hpp>
#ifdef MONO
#include <sophus/sim3.hpp>
#endif
#include "global.h"
#include "maths_utils.h" //for project2d
namespace ScaViSLAM{

Matrix6d third(const Sophus::SE3d & A, const Vector6d & d);
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

  virtual bool
  read                       (std::istream& is)
  {
    assert(false);
    return false;
  }

  virtual bool
  write                      (std::ostream& os) const
  {
    assert(false);
    return false;
  }


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
    Sophus::SE3d Ts2c; // transformation from reference sensor frame to this camera frame into which the point is projected
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
     Vector7d cam2world;
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
     Vector7d lv=cam2world.log();
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
     Eigen::Map<Vector7d> update(const_cast<double*>(update_));

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
      Vector7d v7;
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
      Vector7d v7 = cam2world.log();
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
