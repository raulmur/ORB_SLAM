#include "vio_g2o/scale_solver.h"
#include "vio/rand_sampler.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"

#ifdef USE_SLIM_G2O
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/solvers/linear_solver_dense.h"
#else
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#endif

using namespace Sophus;
using namespace Eigen;
using namespace std;

namespace vio{

scalei2j::scalei2j(int _i, int _j, double _si2j, double _weight):
    i(_i), j(_j), si2j(_si2j), rootWeight(sqrt(_weight))
{}

void G2oVertexScale
::oplusImpl(const double * update_p)
{   
    setEstimate(*update_p+estimate());
}


bool G2oVertexScale
::write(std::ostream& os) const
{
    os<<"estimated scale:"<<_estimate<<endl;
    return os.good();
}


bool G2oVertexScale
::read(std::istream& is)
{   
    is>> _estimate;
    return is.good();
}

bool G2oEdgeScale
::read(std::istream& is)
{   
    is>> _measurement;
    return is.good();
}

bool G2oEdgeScale
::write(std::ostream& os) const
{
    os<<"measurement Si2j:"<< _measurement<<" ";
    return os.good();
}

void G2oEdgeScale
::computeError()
{
    const G2oVertexScale * vi = static_cast<const G2oVertexScale *>(_vertices[0]);
    const G2oVertexScale * vj = static_cast<const G2oVertexScale *>(_vertices[1]);
    // \tilde{S}_i^j - S_w^j/S_w^i
    _error[0] = _measurement-vj->estimate()/(vi->estimate());
}


void G2oEdgeScale::
linearizeOplus()
{   
    const G2oVertexScale * vi = static_cast<const G2oVertexScale *>(_vertices[0]);
    const G2oVertexScale * vj = static_cast<const G2oVertexScale *>(_vertices[1]);

    _jacobianOplusXi[0] = vj->estimate()/(vi->estimate()*vi->estimate());
    _jacobianOplusXj[0] = -1.0/vi->estimate();
}

void G2oVertexScaleTrans
::oplusImpl(const double * update_p)
{   
    Eigen::Map<const Vector4d> update(update_p);
    setEstimate(update+estimate());
}

bool G2oVertexScaleTrans
::write(std::ostream& os) const
{
    os<<"estimated sw2i and twini:"<<_estimate.transpose()<<endl;
    return os.good();
}


bool G2oVertexScaleTrans
::read(std::istream& is)
{   
    is>> _estimate[0]>> _estimate[1]>> _estimate[2]>> _estimate[3];
    return is.good();
}

bool G2oEdgeScaleTrans
::read(std::istream& is)
{   
    is>> _measurement[0]>> _measurement[1]>> _measurement[2]>>  _measurement[3];
    return is.good();
}

bool G2oEdgeScaleTrans
::write(std::ostream& os) const
{
    os<<"measurement \\delta{s}_i^j biinj:"<< _measurement.transpose()<<endl;
    return os.good();
}

void G2oEdgeScaleTrans
::computeError()
{
    const G2oVertexScaleTrans * vi = static_cast<const G2oVertexScaleTrans *>(_vertices[0]);
    const G2oVertexScaleTrans * vj = static_cast<const G2oVertexScaleTrans *>(_vertices[1]);
    Matrix<double, 4,1> stw2i= vi->estimate();
    Matrix<double, 4,1> stw2j= vj->estimate();
    Sophus::SO3d Ri2j= vj->Rw2i*vi->Rw2i.inverse();
    // \tilde{S}_i^j - S_w^j/S_w^i
    _error[0] = _measurement[0]-stw2j[0]/stw2i[0];
    // \tilde{b}_i^j - (t_w^j- s_w^j/s_w^i*R_i^j*t_w^i)
    _error.tail<3>() = _measurement.tail<3>()- stw2j.tail<3>() + Ri2j*stw2i.tail<3>()*stw2j[0]/stw2i[0];
}

void G2oEdgeScaleTrans::
linearizeOplus()
{   
    const G2oVertexScaleTrans * vi = static_cast<const G2oVertexScaleTrans *>(_vertices[0]);
    const G2oVertexScaleTrans * vj = static_cast<const G2oVertexScaleTrans *>(_vertices[1]);
    Matrix<double, 4,1> stw2i= vi->estimate();
    Matrix<double, 4,1> stw2j= vj->estimate();
    Sophus::SO3d Ri2j= vj->Rw2i*vi->Rw2i.inverse();

    _jacobianOplusXi.setZero();
    _jacobianOplusXj.setZero();
    _jacobianOplusXi(0,0) = stw2j[0]/(stw2i[0]*stw2i[0]);
    _jacobianOplusXj(0,0) = -1.0/stw2i[0];

    _jacobianOplusXi.bottomLeftCorner<3,1>() = Ri2j*stw2i.tail<3>()*(-stw2j[0]/(stw2i[0]*stw2i[0]));
    _jacobianOplusXi.bottomRightCorner<3,3>() = (stw2j[0]/stw2i[0])*Ri2j.unit_quaternion().toRotationMatrix();

    _jacobianOplusXj.bottomLeftCorner<3,1>() = Ri2j*stw2i.tail<3>()/stw2i[0];
    _jacobianOplusXj.bottomRightCorner<3,3>() = - Matrix3d::Identity();
}


const double TestScaleTransOptimizer::scale =1.1;
void TestScaleTransOptimizer::optimizeScale(){
    Vector3d dist(1.0, 1.0, 1.0);

    for(int jack=0; jack<num_poses; ++jack)
    {
        sw2i[jack]= pow(scale, jack);
        if(jack==0)
            twini[jack].setZero();
        else{
            twini[jack]= twini[jack-1] + dist*sw2i[jack];
            twini_est[jack]= twini[jack] + dist*sw2i[jack]*Sample::gaussian(0.1);

        }
        cout<< "twini:"<< jack<<" "<< twini[jack].transpose()<<endl;
    }
    //nonlinear optimization with g2o
    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType> >();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
       g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);

    // SET KEYFRAME VERTICES
    G2oVertexScale * vS = new G2oVertexScale();
    vS->setEstimate(1.0);
    vS->setId(0);
    vS->setFixed(true);
    optimizer.addVertex(vS);

    for(int i=1, iend=num_poses; i<iend; ++i)
    {
        G2oVertexScale * vS = new G2oVertexScale();
        vS->setEstimate( 1.0 );
        vS->setId(i);
        vS->setFixed(false);
        optimizer.addVertex(vS);
    }

    //SET EDGES
    int zinc=0;
    double thHuber = 0.3;
    int nIterations=200;
    double sigmaScale=0.1;
    for (; zinc< num_poses-1; ++zinc)
    {
        G2oEdgeScale* e = new G2oEdgeScale();
        e->setVertex(0, optimizer.vertex(zinc));
        e->setVertex(1, optimizer.vertex(zinc +1));
        e->setMeasurement(1.0);

        double invSigma2 = 1/(sigmaScale*sigmaScale);
        Eigen::Matrix<double,1,1> dummy; dummy[0]=invSigma2;
        e->setInformation(dummy);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber);

        optimizer.addEdge(e);
    }
    //last edge: loop constraint
    G2oEdgeScale* e = new G2oEdgeScale();
    e->setVertex(0, optimizer.vertex(0));
    e->setVertex(1, optimizer.vertex(zinc));
    e->setMeasurement(pow(scale, num_poses-1));

    double invSigma2 = 1/(sigmaScale*sigmaScale);
    Eigen::Matrix<double,1,1> dummy; dummy[0]=invSigma2;
    e->setInformation(dummy);

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber);
    optimizer.addEdge(e);

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data
    cout<<"NLSQ estimates of scales:"<<endl;
    for(int i=0, iend= num_poses; i<iend; ++i)
    {
        G2oVertexScale* vS = static_cast<G2oVertexScale*>(optimizer.vertex(i));
        sw2i_est[i] = vS->estimate();

        cout<<"sw2i hat and true:"<< i<< " "<<sw2i_est[i]<< " "<< sw2i[i]<<endl;
    }
}

void TestScaleTransOptimizer::optimizeScaleTrans(){
    Vector3d dist(1.0, 1.0, 1.0);
    for(int jack=0; jack<num_poses; ++jack)
    {
        sw2i[jack]= 1.0;
        sw2i_est[jack]= sw2i[jack] + Sample::gaussian(0.05);
        if(jack==0)
            twini[jack].setZero();
        else{
            twini[jack]= twini[jack-1] + dist*sw2i[jack];
            twini_est[jack]= twini[jack] + dist*sw2i[jack]*Sample::gaussian(0.05);
        }
        cout<< "swini twini:"<< jack<<" "<< sw2i[jack]<<" "<<twini[jack].transpose()<<endl;
   //     cout<< "init swini twini:"<< sw2i_est[jack]<<" "<<twini_est[jack].transpose()<<endl;
    }
    //nonlinear optimization with g2o
    g2o::SparseOptimizer optimizer;
    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType> >();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
       g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
    optimizer.setAlgorithm(solver);

    // SET KEYFRAME VERTICES
    G2oVertexScaleTrans * vS = new G2oVertexScaleTrans();
    vS->setEstimate(Eigen::Vector4d(1.0, 0,0,0));
    vS->Rw2i= SO3d();
    vS->setId(0);
    vS->setFixed(true);
    optimizer.addVertex(vS);

    for(int i=1, iend=num_poses; i<iend; ++i)
    {
        G2oVertexScaleTrans * vS = new G2oVertexScaleTrans();
        Eigen::Vector4d v4;
        v4[0]=sw2i_est[i];
        v4.tail<3>()= twini_est[i];
        vS->setEstimate( v4 );
        vS->Rw2i= SO3d();
        vS->setId(i);
        vS->setFixed(false);
        optimizer.addVertex(vS);
    }

    //SET EDGES
    int zinc=0;
    double thHuber = 0.3;
    int nIterations=200;
    double sigmaScale=0.05;
    double sigmaTrans =0.02;
    for (; zinc< num_poses-1; ++zinc)
    {
        G2oEdgeScaleTrans* e = new G2oEdgeScaleTrans();
        e->setVertex(0, optimizer.vertex(zinc));
        e->setVertex(1, optimizer.vertex(zinc +1));
        Eigen::Vector4d v4;
        v4[0]=1.0 + Sample::gaussian(0.01);
        v4.tail<3>()= dist*sw2i[zinc];
        e->setMeasurement(v4);

        double invSigma2 = 1/(sigmaScale*sigmaScale);
        double invSigmaTrans2 =1/(sigmaTrans*sigmaTrans);
        Eigen::Matrix4d infoMat;
        infoMat.setIdentity();
        infoMat.topLeftCorner<1,1>()*= invSigma2;
        infoMat.bottomRightCorner<3,3>()*= invSigmaTrans2;
        e->setInformation(infoMat);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber);
        optimizer.addEdge(e);
    }
    assert(zinc==num_poses-1);
    //last edge: loop constraint
    G2oEdgeScaleTrans* e = new G2oEdgeScaleTrans();
    e->setVertex(0, optimizer.vertex(0));
    e->setVertex(1, optimizer.vertex(zinc));
    Eigen::Vector4d v4;
    v4[0]=1.0 + Sample::gaussian(0.01);
    v4.tail<3>()= twini[zinc] - twini[0] + dist*Sample::gaussian(0.01);
    e->setMeasurement(v4);

    Eigen::Matrix4d infoMat;
    infoMat.setIdentity();
    double invSigma2 = 1/(sigmaScale*sigmaScale);
    double invSigmaTrans2 =1/(sigmaTrans*sigmaTrans);
    infoMat.topLeftCorner<1,1>()*= invSigma2;
    infoMat.bottomRightCorner<3,3>()*= invSigmaTrans2;
    e->setInformation(infoMat);

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(thHuber);
    optimizer.addEdge(e);

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data
    cout<<"NLSQ estimates of scales:"<<endl;
    for(int i=0, iend= num_poses; i<iend; ++i)
    {
        G2oVertexScaleTrans* vS = static_cast<G2oVertexScaleTrans*>(optimizer.vertex(i));

        cout<<"sw2i tw2i:"<< i<< " "<<vS->estimate().transpose()<<endl;
    }
}

}
