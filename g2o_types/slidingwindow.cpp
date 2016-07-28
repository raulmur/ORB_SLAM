#include "slidingwindow.h"

// Opencv and geographiclib are only used in constructor the sliding window
//#include <GeographicLib/GravityModel.hpp>
#include <GeographicLib/Geocentric.hpp> //transform between NED nav frame and ECEF frame
#include <opencv2/opencv.hpp> //filestorage
#include <opencv2/core/eigen.hpp> //cv2eigen

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h> //for cholmod



using namespace ScaViSLAM;
using namespace GeographicLib;
using namespace Sophus;
using namespace Eigen;
using namespace std;
const double SlidingWindow::qla_= 1.25e-6;

SlidingWindow::SlidingWindow(const string settingFile):nWindowSize(30)
{
    cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr<<"Wrong path to settings file, failed to initialize sliding window filter!"<<endl;
        return;
    }
    ///initialize IMU related parameters
    ScaViSLAM::G2oIMUParameters imu;
    cv::Mat na, nw, acc_bias_var, gyro_bias_var;
    fSettings["na"]>>na;
    fSettings["nw"]>>nw;
    fSettings["acc_bias_var"]>>acc_bias_var;
    fSettings["gyro_bias_var"]>>gyro_bias_var;

    double acc_bias_Tc=fSettings["acc_bias_Tc"];        //half correlation time
    double gyro_bias_Tc=fSettings["gyro_bias_Tc"];

    Eigen::Vector3d q_noise_acc;
    cv::cv2eigen(na, q_noise_acc);
    q_noise_acc=q_noise_acc.cwiseAbs2();
    Eigen::Vector3d q_noise_gyr;
    cv::cv2eigen(nw, q_noise_gyr);
    q_noise_gyr*=(M_PI/180);
    q_noise_gyr=q_noise_gyr.cwiseAbs2();
    Eigen::Vector3d q_noise_accbias;
    cv::cv2eigen(acc_bias_var, q_noise_accbias);
    q_noise_accbias=q_noise_accbias.cwiseAbs2();
    q_noise_accbias*=(2/acc_bias_Tc);
    Eigen::Vector3d q_noise_gyrbias;
    cv::cv2eigen(gyro_bias_var, q_noise_gyrbias);
    q_noise_gyrbias*=(M_PI/180);
    q_noise_gyrbias=q_noise_gyrbias.cwiseAbs2();
    q_noise_gyrbias*=(2/gyro_bias_Tc);

    imu.q_n_aw_babw.head<3>()=q_noise_acc;
    imu.q_n_aw_babw.segment<3>(3)=q_noise_gyr;
    imu.q_n_aw_babw.segment<3>(6)=q_noise_accbias;
    imu.q_n_aw_babw.tail<3>()=q_noise_gyrbias;

    //the relative rotation and translation between the c frame(reference sensor frame), and the IMU sensor frame
    cv::Mat Rs2c, tsinc;
    fSettings["Rs2c"]>>Rs2c; fSettings["tsinc"]>>tsinc;
    Eigen::Matrix3d tempRs2c;
    Eigen::Vector3d tempVec3d;
    cv::cv2eigen(Rs2c, tempRs2c);
    cv::cv2eigen(tsinc, tempVec3d);
    SE3d T_s_2_c(tempRs2c, tempVec3d);
    imu.T_imu_from_cam=T_s_2_c.inverse();

    imu.sampling_interval= fSettings["sample_interval"];

    cv::Mat ned_ref_ecef;//the antenna position represented in ECEF frame
    fSettings["ned_ref_ecef"]>>ned_ref_ecef;

    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    double lat, lon, h;
    earth.Reverse(ned_ref_ecef.at<double>(0), ned_ref_ecef.at<double>(1), ned_ref_ecef.at<double>(2), lat, lon, h);
    Vector3d ref_llh;
    ref_llh<<lat*M_PI/180, lon*M_PI/180, h;
    Matrix3d Re2w_ant=llh2dcm(ref_llh);

    constTw2e= SE3d(Re2w_ant.transpose(), Vector3d(ned_ref_ecef.at<double>(0), ned_ref_ecef.at<double>(1), ned_ref_ecef.at<double>(2)));
    constTe2w = constTw2e.inverse();


    cv::Mat gw;
    fSettings["gw"]>>gw;
    imu.gwomegaw.setZero();
    cv::cv2eigen(gw, tempVec3d);
    imu.gwomegaw.head<3>()=tempVec3d;
    double WIE_E=7292115e-11;    //earth's rotaion rate
    tempVec3d<<0,0,WIE_E;
    imu.gwomegaw.tail<3>()=Re2w_ant*tempVec3d;
    imu_=imu;

    //initialize 4 priors: position, speed bias, etc
    cv::Mat tantinmic;
    fSettings["t_ant_in_imu"]>>tantinmic;
    Vector3d t_ant_in_mic;
    cv::cv2eigen(tantinmic, t_ant_in_mic);
    priorla = T_s_2_c* t_ant_in_mic;

    Sags_= Matrix<double, 27,1>::Zero();
    priorSags = Sags_;

    priorSB = Matrix<double, 9, 1>::Zero();
    cv::Mat initVel, initba, initbg;
    fSettings["init_vel_imu_in_ned"]>>initVel;
    fSettings["init_ba"]>>initba;
    fSettings["init_bg"]>>initbg;
    cv2eigen(initVel, tempVec3d);
    priorSB.head<3>()= tempVec3d;
    cv2eigen(initba, tempVec3d);
    priorSB.segment<3>(3)= tempVec3d;
    cv2eigen(initbg, tempVec3d);
    priorSB.tail<3>()= tempVec3d*(M_PI/180);

    cv::Mat Rc2w;
    fSettings["Rc2w"]>>Rc2w;
    Eigen::Matrix3d tempRc2w;
    cv2eigen(Rc2w, tempRc2w);
    priorTc2w=SE3d(tempRc2w, -tempRc2w*priorla); //assume the world frame is roughly at the IMU sensor at start

    //initialize 4 prior weight matrices
    Eigen::Matrix<double, 9, 9> Wsb=Eigen::Matrix<double, 9, 9>::Identity();
    cv::Mat tempMat;
    Matrix3d tempMat3d;

    fSettings["std_init_vsinw"]>>tempMat;
    cv2eigen(tempMat, tempVec3d);
    tempVec3d= tempVec3d.cwiseAbs2();
    tempMat3d=  tempVec3d.asDiagonal();
    Wsb.block<3,3>(0,0)= tempMat3d.inverse();

    double std_temp=fSettings["std_init_ba"];
    Wsb.block<3,3>(3,3)/=(std_temp*std_temp);
    std_temp=fSettings["std_init_bg"];
    std_temp*=(M_PI/180);
    Wsb.block<3,3>(6,6)/=(std_temp*std_temp);
    invCovPriorSB = Wsb;

    Eigen::Matrix<double, 6, 6> Wpose=Eigen::Matrix<double, 6, 6>::Identity();
    fSettings["std_init_tcinw"]>>tempMat;
    cv2eigen(tempMat, tempVec3d);
    tempVec3d= tempVec3d.cwiseAbs2();
    tempMat3d=  tempVec3d.asDiagonal();

    Wpose.block<3,3>(0,0)= tempMat3d.inverse();

    fSettings["std_init_rc2w"]>>tempMat;
    cv2eigen(tempMat, tempVec3d);
    tempVec3d*=(M_PI/180);
    tempVec3d= tempVec3d.cwiseAbs2();
    tempMat3d= tempVec3d.asDiagonal();

    Wpose.block<3,3>(3,3)= tempMat3d.inverse();
    invCovPriorPose = Wpose;

    Eigen::Matrix<double, 27, 27> WSags=Eigen::Matrix<double, 27, 27>::Identity();
    std_temp=fSettings["std_init_Sa"];
    WSags.block<9,9>(0,0)/=(std_temp*std_temp);
    std_temp=fSettings["std_init_Sg"];
    WSags.block<9,9>(9,9)/=(std_temp*std_temp);
    std_temp=fSettings["std_init_Ts"];
    WSags.block<9,9>(18,18)/=(std_temp*std_temp);
    invCovSags= WSags;

    Eigen::Matrix<double, 3, 3> Wla=Eigen::Matrix<double, 3, 3>::Identity();
    std_temp=fSettings["std_init_la"];
    Wla/=(std_temp*std_temp);
    invCovla= Wla;
}
SlidingWindow::~SlidingWindow()
{

}
void SlidingWindow::AddVertex(const Sophus::SE3d predTskp12w, const double timestamp,
                              const Eigen::Matrix<double,9,1> predSBkp1,
                              const Eigen::Vector3d vGPSObs, const Eigen::Matrix3d covGPSMeas,
                              const std::vector<Eigen::Matrix<double, 7,1> > vIMUObs)
{
    vTimes.push_back(timestamp);
    vTw2cj.push_back((predTskp12w*imu_.T_imu_from_cam).inverse());
    vSpeedBias.push_back(predSBkp1);
    if(vla_.empty())
        vla_.push_back(priorla);
    else
        vla_.push_back(vla_.back());

    vGPSMeas.push_back(vGPSObs);
    if(!vIMUObs.empty())
        vIMUMeas.push_back(vIMUObs);
    vInvCovGPSMeas.push_back(covGPSMeas.inverse());
}

void SlidingWindow::RemoveVertex(ofstream & out_stream_optimized)
{
    if(vTw2cj.size()>= nWindowSize)
    {
        // print output
        auto itSB= vSpeedBias.begin();
        auto itTime = vTimes.begin();
        auto it = vTw2cj.begin();
        SE3d Tcj2w= it->inverse();
        out_stream_optimized<<setprecision(10)<< *itTime <<setprecision(6)<<" "<< Tcj2w.translation().transpose()<<" ";
        out_stream_optimized<< Tcj2w.unit_quaternion().coeffs().transpose()<<" "<<(*itSB).transpose()<<" ";
        out_stream_optimized<< Sags_.transpose()<<" "<< vla_.front().transpose()<<endl;
        // remove head vertices
        vTimes.pop_front();
        vTw2cj.pop_front();
        vSpeedBias.pop_front();
        vla_.pop_front();

        vGPSMeas.pop_front();
        vIMUMeas.pop_front();
        vInvCovGPSMeas.pop_front();
    }
}
void SlidingWindow::DumpVertices(ofstream & out_stream_optimized){
    auto itSB= vSpeedBias.begin();
    auto itTime = vTimes.begin();
    auto itla= vla_.begin();
    for(auto it = vTw2cj.begin(), ite= vTw2cj.end() ;it!=ite; ++it, ++itSB, ++itTime, ++itla)
    {
        SE3d Tcj2w= it->inverse();
        out_stream_optimized<<setprecision(10)<< *itTime <<setprecision(6)<<" "<< Tcj2w.translation().transpose()<<" ";
        out_stream_optimized<< Tcj2w.unit_quaternion().coeffs().transpose()<<" "<<(*itSB).transpose()<<" ";
        out_stream_optimized<< Sags_.transpose()<<" "<< itla->transpose()<<endl;
    }
}
void SlidingWindow::Optimize(int num_iters, char bOptimizeSagsAndArm)
{
    if(vTw2cj.size()< nWindowSize/2)
    {
        return;
    }
    bool bOptimizeSags= bOptimizeSagsAndArm & 0x10;
    bool bOptimizeLa= bOptimizeSagsAndArm & 0x01;
    bool bSoftLeverArm= bOptimizeSagsAndArm & 0x02;
    g2o::SparseOptimizer optimizer;
    ScaViSLAM::G2oIMUParameters  * g2o_imu  = new ScaViSLAM::G2oIMUParameters(imu_);
    g2o_imu->setId(0);
    //  setupG2o
    optimizer.setVerbose(true);
    g2o::BlockSolverX::LinearSolverType * linearSolver=
            new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * block_solver =
            new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* lm =
            new g2o::OptimizationAlgorithmLevenberg(block_solver);
    lm->setMaxTrialsAfterFailure(5);
    optimizer.setAlgorithm(lm);
    if (!optimizer.addParameter(g2o_imu))
    {
        assert(false);
    }
    ///construct the optimizer on the sliding window
    if(bOptimizeSags){
        //add shape matrices and lever arm
        ScaViSLAM::G2oVertexShapeMatrices * v_sm = new ScaViSLAM::G2oVertexShapeMatrices();
        v_sm->setId(0);
        v_sm->setEstimate(Sags_);
        v_sm->setFixed(false);
        optimizer.addVertex(v_sm);
        pSags= v_sm;
    }else
        pSags=NULL;

    int num_vertices= vTw2cj.size();

    vpSE3.clear(); vpSE3.reserve(num_vertices);
    vpSB.clear(); vpSB.reserve(num_vertices);
    vpla.clear(); vpla.reserve(num_vertices);

    if(bOptimizeLa && !bSoftLeverArm){
        ScaViSLAM::G2oVertexLeverArm * v_la = new ScaViSLAM::G2oVertexLeverArm();
        v_la->setId(1);
        v_la->setEstimate(vla_.front());
        v_la->setFixed(false);
        optimizer.addVertex(v_la);
        vpla.push_back( v_la);
    }
    const int offset= 2; // 0 for shape matrices, 1 for lever arm

    int jack=0;
    auto it2= vSpeedBias.begin();
    auto itla= vla_.begin();
    for(auto it= vTw2cj.begin(),  ite= vTw2cj.end();
        it!= ite; ++jack, ++it, ++it2, ++itla)
    {
        //add pose to g2o
        ScaViSLAM::G2oVertexSE3 * v_se3 = new ScaViSLAM::G2oVertexSE3();
        v_se3->setId(jack*MAGIC2 + offset);
        v_se3->setEstimate(*it);
        v_se3->setFixed(false);
        optimizer.addVertex(v_se3);
        vpSE3.push_back(v_se3);
        //addSpeedBiasToG2o
        ScaViSLAM::G2oVertexSpeedBias * v_sb = new ScaViSLAM::G2oVertexSpeedBias();
        v_sb->setId(jack*MAGIC2+ 1 + offset);
        v_sb->setEstimate(*it2);
        v_sb->setFixed(false);
        optimizer.addVertex(v_sb);
        vpSB.push_back(v_sb);

        if(bOptimizeLa && bSoftLeverArm){
            ScaViSLAM::G2oVertexLeverArm * v_la = new ScaViSLAM::G2oVertexLeverArm();
            v_la->setId(jack*MAGIC2 + 2+ offset);
            v_la->setEstimate(*itla);
            v_la->setFixed(false);
            optimizer.addVertex(v_la);
            vpla.push_back( v_la);
        }
    }
    ///add priors
    if(vTw2cj.size()< nWindowSize)
    {
        // add prior constraint on vertices at first epoch
        ScaViSLAM::G2oSpeedBiasObs9DEdge *e9=new ScaViSLAM::G2oSpeedBiasObs9DEdge();

        e9->vertices()[0] = vpSB[0];
        e9->setMeasurement(priorSB);
        e9->information() = invCovPriorSB;
        optimizer.addEdge(e9);

        ScaViSLAM::G2oSE3Observation6DEdge *e6=new ScaViSLAM::G2oSE3Observation6DEdge();
        e6->vertices()[0] = vpSE3[0];
        e6->setMeasurement(priorTc2w);
        e6->information() = invCovPriorPose;
        optimizer.addEdge(e6);
        if(bOptimizeSags){
            ScaViSLAM::G2oEdgeShapeMatrices *e27=new ScaViSLAM::G2oEdgeShapeMatrices();

            e27->vertices()[0] = pSags;
            e27->setMeasurement(priorSags);
            e27->information() = invCovSags;
            optimizer.addEdge(e27);
        }
        if(bOptimizeLa){
            ScaViSLAM::G2oUnaryEdgeLeverArm *e3=new ScaViSLAM::G2oUnaryEdgeLeverArm();
            e3->vertices()[0] = vpla[0];
            e3->setMeasurement(priorla);
            e3->information() = invCovla;
            optimizer.addEdge(e3);
        }
    }
    // add GPS observations
    auto itInvCovGPS= vInvCovGPSMeas.begin();
    auto itTimes = vTimes.begin();
    double last_time = *itTimes;
    int kudos =0;
    for(auto it= vGPSMeas.begin(), ite= vGPSMeas.end(); it!=ite; ++it, ++itInvCovGPS, ++itTimes)
    {
        if(bOptimizeLa){
            ScaViSLAM::G2oEdgeGPSObservation * e = new ScaViSLAM::G2oEdgeGPSObservation();
            e->vertices()[0] = vpSE3[kudos];
            if(bSoftLeverArm)
                e->vertices()[1] = vpla[kudos];
            else
                e->vertices()[1] = vpla[0];
            e->setMeasurement(constTe2w*(*it));
            e->information() =constTe2w.rotationMatrix()*(*itInvCovGPS)*constTe2w.rotationMatrix().transpose();
            optimizer.addEdge(e);
            if(kudos && bSoftLeverArm)// lever arm constraint
            {
                G2oEdgeLeverArm * e3= new G2oEdgeLeverArm();
                e3->vertices()[0]= vpla[kudos-1];
                e3->vertices()[1]= vpla[kudos];
                e3->setMeasurement(Vector3d::Zero());
                Matrix3d info_la= Matrix3d::Identity()/((*itTimes - last_time)*qla_);
                e3->setInformation(info_la);
                optimizer.addEdge(e3);

                ScaViSLAM::G2oUnaryEdgeLeverArm *e1=new ScaViSLAM::G2oUnaryEdgeLeverArm();
                e1->vertices()[0] = vpla[kudos];
                e1->setMeasurement(priorla);
                e1->information() = invCovla;
                optimizer.addEdge(e1);
            }
        }
        else
        {
            ScaViSLAM::GPSObservationPosition3DEdge * e = new ScaViSLAM::GPSObservationPosition3DEdge();
            e->vertices()[0] = vpSE3[kudos];
            Vector3d tcinw=constTe2w*(*it) - vpSE3[kudos]->estimate().rotationMatrix().transpose()*priorla;
            e->setMeasurement(tcinw);
            e->information() =constTe2w.rotationMatrix()*(*itInvCovGPS)*constTe2w.rotationMatrix().transpose();
            optimizer.addEdge(e);
        }
        ++kudos;
    }
    // add constraint from IMU observations
    kudos=1;
    itTimes = vTimes.begin();
    last_time = *itTimes;
    ++itTimes;
    for(auto it = vIMUMeas.begin(), ite= vIMUMeas.end(); it!=ite; ++it, ++itTimes){
        if(bOptimizeSags){
            ScaViSLAM::G2oEdgeIMUConstraintEx * e = new ScaViSLAM::G2oEdgeIMUConstraintEx();
            e->resize(5);
            e->vertices()[4]= pSags;

            e->setParameterId(0,0);
            e->vertices()[0]
                    = vpSE3[kudos-1];
            e->vertices()[1]
                    = vpSB[kudos-1];
            e->vertices()[2]=vpSE3[kudos];
            e->vertices()[3]=vpSB[kudos];

            e->time_frames[0]= last_time;
            e->time_frames[1]= *itTimes;
            e->setMeasurement(*it);
            e->calcAndSetInformation(*g2o_imu);
            optimizer.addEdge(e);
        }
        else{
            ScaViSLAM::G2oEdgeIMUConstraint * e = new ScaViSLAM::G2oEdgeIMUConstraint();
            e->resize(4);

            e->setParameterId(0,0);
            e->vertices()[0]
                    = vpSE3[kudos-1];
            e->vertices()[1]
                    = vpSB[kudos-1];
            e->vertices()[2]=vpSE3[kudos];
            e->vertices()[3]=vpSB[kudos];
            e->time_frames[0]= last_time;
            e->time_frames[1]= *itTimes;
            e->setMeasurement(*it);
            //            Matrix<double, 15, 15> info=Matrix<double, 15, 15>::Identity();
            //            const double factor=1;
            //            info.block(0,0,3,3)*=(1e-7*factor);
            //            info.block(3,3,3,3)*=(5e-6*factor);
            //            info.block(6,6,3,3)*=8e-6*factor;
            //            info.block(9,9,3,3)*=4e-4*factor;
            //            info.block(12,12,3,3)*=1e-6*factor;
            //            info=info.cwiseAbs2();
            //            e->setInformation(info.inverse());
            e->calcAndSetInformation(*g2o_imu);
            optimizer.addEdge(e);
        }
        ///add pseudo gravity observations
        const double std_gravity=2.0;
        const vector<Eigen::Matrix<double, 7,1> >& imu_meas = *it;
        if((last_time- imu_meas[0][0])<imu_.sampling_interval/2.0 && (imu_meas[0].segment<3>(1).norm()-9.8)<std_gravity)
        {
            ScaViSLAM::IMURollPitchEdge * e = new ScaViSLAM::IMURollPitchEdge();
            e->setParameterId(0,0);

            e->vertices()[0]
                    = vpSE3[kudos-1];
            e->vertices()[1]
                    = vpSB[kudos-1];
            e->setMeasurement(imu_meas[0].segment<3>(1));
            Vector3d tempVec3d;
            tempVec3d<<1/std_gravity, 1/std_gravity, 1/std_gravity;
            tempVec3d=tempVec3d.cwiseAbs2();
            e->setInformation(tempVec3d.asDiagonal());
            optimizer.addEdge(e);
        }
        else if((imu_meas[1][0]-last_time)<imu_.sampling_interval/2.0 && (imu_meas[1].segment<3>(1).norm()-9.8)<std_gravity)
        {
            ScaViSLAM::IMURollPitchEdge * e = new ScaViSLAM::IMURollPitchEdge();
            e->setParameterId(0,0);

            e->vertices()[0]= vpSE3[kudos-1];
            e->vertices()[1]= vpSB[kudos-1];
            e->setMeasurement(imu_meas[1].segment<3>(1));
            Vector3d tempVec3d; tempVec3d<<1/std_gravity, 1/std_gravity, 1/std_gravity;
            tempVec3d=tempVec3d.cwiseAbs2();
            e->setInformation(tempVec3d.asDiagonal());
            optimizer.addEdge(e);
        }

        ++kudos;
        last_time =*itTimes;
    }

    ///TODO: add zero velocity observations

    //optimize
    optimizer.initializeOptimization();
    optimizer.optimize(num_iters);
    //  restoreDataFromG2o
    auto itTw2c= vTw2cj.begin();
    for (auto it= vpSE3.begin(), ite= vpSE3.end(); it!=ite;++it ,++itTw2c)
    {
        (*itTw2c) = (*it)->estimate();
    }

    auto itSB = vSpeedBias.begin();
    for (auto it= vpSB.begin(), ite= vpSB.end(); it!=ite; ++it, ++itSB)
        (*itSB)= (*it)->estimate();
    if(bOptimizeSags)
        Sags_=pSags->estimate();
    if(bOptimizeLa){
        auto itla= vla_.begin();
        if(bSoftLeverArm){
            for(auto it= vpla.begin(), ite= vpla.end(); it!=ite; ++it, ++itla){
                (*itla)= (*it)->estimate();
            }
        }
        else
        {
            assert(vpla.size()==1);
            for(auto ite= vla_.end(); itla!=ite; ++itla){
                (*itla)= vpla.front()->estimate();
            }
        }
    }
}
void SlidingWindow::SetInitialShapeMatrices(Eigen::Matrix<double, 6,1> vSMDiagonals)
{
    for (int i=0; i<2; ++i){
        for (int j=0; j<3; ++j){
            Sags_(i*9+4*j)=vSMDiagonals(i*3+j);
        }
    }
}
// observations: without optimizing lever arm gives good results, optimizing rigid constant lever arm worse results,
// optimizing soft lever arm seemingly better result.
// conclusion: the lever arm between antenna and the IMU sensor is unobservable as the car is moving on a plane
void SlidingWindow::GlobalOptimize(int num_iters, const Eigen::Matrix<double, 6,1> vSMDiagonals)
{
    nWindowSize = vTw2cj.size()+1; //enable prior
    char option= 0x03;

    if( option & 0x10)
        SetInitialShapeMatrices(vSMDiagonals);
    Optimize(num_iters, option);

}
