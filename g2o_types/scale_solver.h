#ifndef SCALE_SOLVER_H_
#define SCALE_SOLVER_H_

#include <sophus/so3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

#include <vector>

namespace ScaViSLAM{

class G2oVertexScale : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexScale               ():first_estimate(NULL){}
    ~G2oVertexScale               (){
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
    setToOriginImpl            () {_estimate= 0.0;}
    void setFirstEstimate(const double& fe){
        first_estimate=new double(fe);
    }
    //for first estimate Jacobians
    double * first_estimate;
};
class G2oEdgeScale
        : public g2o::BaseBinaryEdge<1, double, G2oVertexScale, G2oVertexScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeScale                () {}

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;
    void
    computeError               ();
    virtual void
    linearizeOplus             ();
};

//scale + translation
class G2oVertexScaleTrans : public g2o::BaseVertex<4, Eigen::Matrix<double,4,1> >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oVertexScaleTrans               (Sophus::SO3d _Rw2i= Sophus::SO3d()):first_estimate(NULL), Rw2i(_Rw2i){}
    ~G2oVertexScaleTrans               (){
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
    setToOriginImpl            () {_estimate.setZero();}
    void setFirstEstimate(const Eigen::Matrix<double,4,1>& fe){
        first_estimate=new Eigen::Matrix<double,4,1>(fe);
    }

    //for first estimate Jacobians
    Eigen::Matrix<double, 4,1> * first_estimate;
    Sophus::SO3d  Rw2i;
};
class G2oEdgeScaleTrans
        : public g2o::BaseBinaryEdge<4, Eigen::Matrix<double,4,1>, G2oVertexScaleTrans, G2oVertexScaleTrans>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    G2oEdgeScaleTrans                () {}

    virtual bool
    read                       (std::istream& is);
    virtual bool
    write                      (std::ostream& os) const;
    void
    computeError               ();
    virtual void
    linearizeOplus             ();
};
class TestScaleTransOptimizer{
public:
    const int num_poses;
    static const double scale;
    std::vector<double> sw2i;
    std::vector<double> sw2i_est;

    std::vector<Eigen::Vector3d> twini;
    std::vector<Eigen::Vector3d> twini_est;

    TestScaleTransOptimizer(const int _num_poses=50): num_poses(_num_poses),
        sw2i(num_poses), sw2i_est(num_poses),
        twini(num_poses),twini_est(num_poses)
    {}
    void optimizeScale();
    void optimizeScaleTrans();
};
}
#endif
