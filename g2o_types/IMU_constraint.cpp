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

#include "IMU_constraint.h"

#include "EXTERNAL/ceres/autodiff.h" //autodiff
namespace ScaViSLAM{
using namespace Eigen;
using namespace Sophus;
using namespace std;
bool G2oVertexSpeedBias::read(std::istream& is) {
    Matrix<double, 9,1> est;
    for (int i=0; i<9; i++)
        is >> est[i];
    setEstimate(est);
    return true;
}
bool G2oVertexSpeedBias::write(std::ostream& os) const {
    for (int i=0; i<9; i++)
        os << estimate()[i] << " ";
    return os.good();
}

bool G2oVertexSpeedBiasEx::read(std::istream& is) {
    Matrix<double, 36,1> est;
    for (int i=0; i<36; i++)
        is >> est[i];
    setEstimate(est);
    return true;
}
bool G2oVertexSpeedBiasEx::write(std::ostream& os) const {
    for (int i=0; i<36; i++)
        os << estimate()[i] << " ";
    return os.good();
}

bool G2oEdgeIMUConstraint
::write(std::ostream& os) const {
    os <<g2o_IMU->id() << endl;
    for (unsigned int i=0; i<measurement().size(); i++){
        for(int j=0; j<7; ++j)
            os  << measurement()[i][j] << " ";
        os<<endl;
    }
    for (int i=0; i<15; i++){
        for (int j=i; j<15; j++){
            os << information()(i,j)<<" ";
        }
        os<<endl;
    }
    return os.good();
}
bool G2oEdgeIMUConstraint::read(std::istream& is) {
    int paramId;
    is >> paramId;
    setParameterId(0, paramId);
    int num_measurements;
    is >>num_measurements;
    _measurement.resize(num_measurements);
    for (int i=0; i<num_measurements; ++i){
        for(int j=0; j<7; ++j)
            is  >> _measurement[i][j];
    }

    for (int i=0; i<15; i++)
        for (int j=i; j<15; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}
/**
* \brief Functor used to compute the Jacobian via AD
* $J=\frac{\partial e(T_w^{c(k)}, \epsilon^k, \mathbf{x}_{sb}^k, \mathbf{x}_R^{k+1}, z_s^k)}
* {\partial (\epsilon^k, \mathbf{x}_{sb}^k)}$, visually,
*    J0,J1 =                                                  \epsilon^k      v_{s(k)}^w      b_a(k)  b_g(k)
*                                                       ---------------------------------------------------
*   log(\hat{T}_{s(k+1)}^w T_c^s T_w^{c(k+1)})^\vee     |       J00             J01             J02     J03
*   \hat{v}_{s(k)}^w-v_{s(k)}^w                         |       J10             J11             J12     J13
*   \hat{b}_a(k+1)-b_a(k+1)                             |       0               0               I       0
*   \hat{b}_g(k+1)-b_g(k+1)                             |       0               0               0       I
*                                                       ---------------------------------------------------
* $e=[log(\hat{T}_{s(k+1)}^w T_c^s T_w^{c(k+1)})^\vee; \hat{v}_{s(k)}^w-v_{s(k)}^w]$ is the cost function
*/

template <typename T>
bool G2oEdgeIMUConstraint
::operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1, const T* pXsbkp1, T* error) const
//must use const because AutoDiff::Differentiate's first argument is const *this
{
    //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)
    const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2ck(pTw2ck); //qxyzw txyz
    typename Eigen::Matrix<T, 6, 1, Eigen::ColMajor>::ConstMapType epsk(epsilonk);
    typename Eigen::Matrix<T, 9, 1, Eigen::ColMajor>::ConstMapType Xsbk(pXsbk);
    Eigen::Matrix<T,9,1> cast_Xsbk=Xsbk;
    const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2ckp1(pTw2ckp1); //qxyzw txyz
    typename Eigen::Matrix<T, 9, 1, Eigen::ColMajor>::ConstMapType Xsbkp1(pXsbkp1);
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));
    Eigen::Map<Eigen::Matrix<T,9,1> > value(error);//delta eps and vel

    //SE3Group<Scalar> Tw2ck=se3Tw2ck.template cast<double>();//this template keyword trick follows https://forum.kde.org/viewtopic.php?f=74&t=62606
    SE3Group<T> perturbed_Tw2ck=Sophus::SE3Group<T>::exp(epsk)*se3Tw2ck;
    Sophus::SE3Group<T>  T_s1_to_w=(params_imu->T_imu_from_cam.cast<T>()*perturbed_Tw2ck).inverse();

    std::vector<Eigen::Matrix<T,7,1> > cast_measurement(_measurement.size());
    for(unsigned int jack=0; jack<_measurement.size(); ++jack)
        cast_measurement[jack]=_measurement[jack].cast<T>();
    T cast_time_frames[2]={T(time_frames[0]), T(time_frames[1])};
    Eigen::Matrix<T, 6, 1> cast_gwomegaw=params_imu->gwomegaw.cast<T>();
    Eigen::Matrix<T,12,1> cast_q=params_imu->q_n_aw_babw.cast<T>();
    Sophus::SE3Group<T>  pred_T_s2_to_w;
    Matrix<T,3,1> pred_speed_2;
    Eigen::Matrix<T, 15,15> *holder=NULL;
    predictStates(T_s1_to_w, cast_Xsbk, cast_time_frames,
                  cast_measurement, cast_gwomegaw, cast_q,
                  &pred_T_s2_to_w, &pred_speed_2, holder);
    SE3Group<T> predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam.cast<T>();
    value.head(6)=SE3Group<T>::log(predTckp12w*se3Tw2ckp1);
    value.tail(3)=pred_speed_2-Xsbkp1.head(3);
    return true;
}

// this implementation refers to ScaViSLAM:g2o_types/anchored_points.cpp and g2o:g2o/types/sba/types_six_dof_expmap.cpp
// though N.B. g2o used SE3Quat::exp([\omega, \upsilon]), Strasdat used SE3d::exp([\upsilon, \omega])
//here we follow Strasdat's notation
void G2oEdgeIMUConstraint
::computeError()
{
    //first two nodes
    const G2oVertexSE3 * T_c1_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexSpeedBias * speed_bias_1
            = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);
    //second two nodes
    const G2oVertexSE3 * T_c2_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[2]);
    const G2oVertexSpeedBias * speed_bias_2
            = static_cast<const G2oVertexSpeedBias*>(_vertices[3]);

    Eigen::Matrix<double, 9,1 > error_posvel= Eigen::Matrix<double, 9,1 >::Zero();
    Matrix<double, 6, 1> zero_delta= Matrix<double, 6, 1>::Zero();
    (*this)(T_c1_from_world->estimate().data(), zero_delta.data(), speed_bias_1->estimate().data(),
            T_c2_from_world->estimate().data(), speed_bias_2->estimate().data(), error_posvel.data() );
    _error.head(9)=error_posvel;
    _error.tail(6)=(speed_bias_1->estimate()).tail(6)-(speed_bias_2->estimate()).tail(6);
}
/**
* \brief Functor used to compute the Jacobian via AD
* $J=\frac{\partial log(\tilde{T}_{s(k+1)}^w\boxplus (\delta x_s^w, \psi^w) T_c^s T_w^{c(k+1)})^\vee}
* {\partial (\delta x_s^w, \psi^w)}$
* $log(\tilde{T}_{s(k+1)}^w\boxplus (\delta x_s^w, \psi^w) T_c^s T_w^{c(k+1)})^\vee$ is the cost function of
* $\tilde{T}_{s(k+1)}^w$, $(\delta x_s^w, \psi^w)$
*/

struct LogDeltaSE3Vee
{
    template<typename T>
    bool operator()(const T* predTskp12w, const T* pDeltaxpsi, const T* pTw2skp1, T* value) const
    {
        Eigen::Map<const Sophus::SE3Group<T> > se3Tskp12wConst(predTskp12w); //qxyzw txyz
        Sophus::SE3Group<T> se3Tskp12w=se3Tskp12wConst;
        typename Eigen::Matrix<T, 6, 1, Eigen::ColMajor>::ConstMapType deltaxpsi(pDeltaxpsi);
        se3Tskp12w.translation() =se3Tskp12wConst.translation() - deltaxpsi.head(3);
        Matrix<T,3,1> rvec= deltaxpsi.tail(3);
        se3Tskp12w.setQuaternion(quaternionFromSmallAngle(rvec)*se3Tskp12wConst.unit_quaternion());

        const Eigen::Map<const Sophus::SE3Group<T> > se3Tw2skp1(pTw2skp1); //qxyzw txyz
        Eigen::Map<Eigen::Matrix<T,6,1> > tang(value);
        tang= (se3Tskp12w*se3Tw2skp1).log();
        return true;
    }
};
// compute information matrix
void G2oEdgeIMUConstraint
::calcAndSetInformation(const G2oIMUParameters & params_imu)
{
    // predict T s(k+1) to w and covariance
    //first two nodes
    const G2oVertexSE3 * T_c1_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexSpeedBias * speed_bias_1
            = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);
    //second two nodes
    const G2oVertexSE3 * T_c2_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[2]);

    //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)
    SE3d T_s1_to_w=(params_imu.T_imu_from_cam*T_c1_from_world->estimate()).inverse();
    SE3d pred_T_s2_to_w;
    Vector3d pred_speed_2;
    Matrix<double, 15,15> P= Matrix<double, 15,15>::Identity()*1e-10; // covariance of first states are 0 in this case

    predictStates(T_s1_to_w, speed_bias_1->estimate(), time_frames,
                  _measurement, params_imu.gwomegaw, params_imu.q_n_aw_babw,
                  &pred_T_s2_to_w, &pred_speed_2, &P);
    Matrix<double,15,15> J_pred
            = Matrix<double,15,15>::Zero();
#if 0
    const double h = 0.00000001; //was 1e-12 in Strasdat's numerical differentiation
    Matrix<double, 6,1> orig_pose_error=SE3d::log(pred_T_s2_to_w*params_imu.T_imu_from_cam*T_c2_from_world->estimate());
    Matrix<double, 6,1> diff;
    //J=(perturbed value -original value)/ (perturbed variable - original variable), however $\delta x$ is defined
    //take derivative w.r.t perturbation in predicted position
    for (unsigned int i=0; i<3; ++i)
    {
        SE3d temp_T=pred_T_s2_to_w;
        temp_T.translation()[i]-=h;
        diff=SE3d::log(temp_T*params_imu.T_imu_from_cam*T_c2_from_world->estimate());
        diff-=orig_pose_error;
        J_pred.col(i).head(6) = diff/h;
    }
    //take derivative w.r.t perturbation in predicted rotation
    for (unsigned int i=0; i<3; ++i)
    {
        SE3d temp_T=pred_T_s2_to_w;
        Vector3d rvec=Vector3d::Zero();
        rvec[i]=h;
        temp_T.setQuaternion(quaternionFromSmallAngle(rvec)*temp_T.unit_quaternion());
        diff=SE3d::log(temp_T*params_imu.T_imu_from_cam*T_c2_from_world->estimate());
        diff-=orig_pose_error;
        J_pred.col(i+6).head(6) = diff/h; // +6 because $\psi^w$ corresponds to 6-8 rows/cols of P
    }
    cout<<"ND J_pred.block(0,0,6,9):"<<endl<< J_pred.block(0,0,6,9)<<endl;

#else
    // compute the Jacobian using AD
    const int kGlobalSize=7, kLocalSize=6, num_outputs=6;
    Matrix<double, kLocalSize, 1> zero_delta= Matrix<double, kLocalSize, 1>::Zero();
    double value[num_outputs];
    Matrix<double, num_outputs, kLocalSize, Eigen::RowMajor> dTinv_de_AD;
    const double *parameters[3] = { pred_T_s2_to_w.data(), zero_delta.data(),
                                    (params_imu.T_imu_from_cam*T_c2_from_world->estimate()).data()};
    double *jacobians[3] = {NULL, dTinv_de_AD.data(), NULL };
    LogDeltaSE3Vee deltaTvee;
    ceres::internal::AutoDiff<LogDeltaSE3Vee, double, kGlobalSize, kLocalSize, kGlobalSize>
            ::Differentiate(deltaTvee, parameters, num_outputs, value, jacobians);
    J_pred.block(0,0,6,3)= dTinv_de_AD.block(0,0,6,3);
    J_pred.block(0,6,6,3)= dTinv_de_AD.block(0,3,6,3);
    //cout<<"AD J_pred.block(0,0,6,9):"<<endl<< J_pred.block(0,0,6,9)<<endl;
#endif
    J_pred.block(6,3,3,3)=Matrix3d::Identity();//delta velocity
    J_pred.block(9,9,3,3)=Matrix3d::Identity();//b_a
    J_pred.block(12,12,3,3)=Matrix3d::Identity();//b_g
    //TODO: verify that numerical Jacobian satisfy $J_pred\approx diag(third(_error.head(6), SE3d()),I)$
    _information=(J_pred*P*J_pred.transpose()).inverse();

    //    cout<<"new P(9,9)"<<P.topLeftCorner(9,9)<<endl;
    //    cout<<_information.diagonal().transpose()<<endl;
}

//following $g^2o$: A general framework for graph optimization
//$\mathbf{J}_{ij}=\frac{\partial \mathbf{e}_{ij}(\breve{\mathbf{x}}\boxplus \Delta\mathbf{x})}{\partial \mathbf{\Delta x}}\vert_{\mathbf{\Delta x}=0}$
void G2oEdgeIMUConstraint::linearizeOplus()
{
    G2oVertexSE3 * vpose_1 = static_cast<G2oVertexSE3 *>(_vertices[0]);
    SE3d T_1w =  (vpose_1->first_estimate==NULL? vpose_1->estimate(): *(vpose_1->first_estimate)); //T world to 1 frame
    G2oVertexSpeedBias* vsb_1 = static_cast<G2oVertexSpeedBias*>(_vertices[1]);
    Matrix<double, 9,1> sb_1 = (vsb_1->first_estimate==NULL? vsb_1->estimate(): *(vsb_1->first_estimate));
    G2oVertexSE3 * vpose_2 = static_cast<G2oVertexSE3 *>(_vertices[2]);
    SE3d T_2w =(vpose_2->first_estimate==NULL? vpose_2->estimate(): *(vpose_2->first_estimate));
    G2oVertexSpeedBias* vsb_2 = static_cast<G2oVertexSpeedBias*>(_vertices[3]);
    Matrix<double, 9,1> sb_2 = (vsb_2->first_estimate==NULL? vsb_2->estimate(): *(vsb_2->first_estimate));
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));
#if 0
    const double h = 0.00000001; //was 1e-12 in Strasdat's numerical differentiation
    const Matrix<double, 6,1> orig_pose_error=_error.head(6);
    Matrix<double, 6,1> diff;

    _jacobianOplus[0]=Matrix<double,15, 6>::Zero(); //15x6
    //numerical differentiation
    for (unsigned int i=0; i<6; ++i)
    {
        Matrix<double,6,1> eps
                = Matrix<double,6,1>::Zero();
        eps[i] = h;
        SE3d temp_T=SE3d::exp(eps)*T_1w;
        //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)
        SE3d T_s1_to_w=(params_imu->T_imu_from_cam*temp_T).inverse();
        SE3d pred_T_s2_to_w;
        Vector3d pred_speed_2;
        Eigen::Matrix<double, 15,15> *holder=NULL;
        predictStates(T_s1_to_w, sb_1, time_frames,
                      _measurement, params_imu->gwomegaw, params_imu->q_n_aw_babw,
                      &pred_T_s2_to_w, &pred_speed_2, holder);

        diff=SE3d::log(pred_T_s2_to_w*params_imu->T_imu_from_cam*T_2w);
        diff-=orig_pose_error;
        _jacobianOplus[0].col(i).head(6) = diff/h;
        _jacobianOplus[0].col(i).segment(6,3) = ((pred_speed_2- sb_2.head(3))-_error.segment(6,3))/h;

    }

    _jacobianOplus[1]=Matrix<double,15, 9>::Zero(); //15x9
    //numerical differentiation
    const SE3d T_s1_to_w=(params_imu->T_imu_from_cam*T_1w).inverse();
    for (unsigned int i=0; i<9; ++i)
    {
        //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)
        Matrix<double, 9,1> temp_sb=sb_1;
        temp_sb[i]+=h;
        SE3d pred_T_s2_to_w;
        Vector3d pred_speed_2;
        Eigen::Matrix<double, 15,15> *holder=NULL;
        predictStates(T_s1_to_w, temp_sb, time_frames,
                      _measurement, params_imu->gwomegaw, params_imu->q_n_aw_babw,
                      &pred_T_s2_to_w, &pred_speed_2, holder);

        diff=SE3d::log(pred_T_s2_to_w*params_imu->T_imu_from_cam*T_2w);
        diff-=orig_pose_error;
        _jacobianOplus[1].col(i).head(6) = diff/h;
        _jacobianOplus[1].col(i).segment(6,3) = ((pred_speed_2- sb_2.head(3))-_error.segment(6,3))/h;
    }
    //cout<<"ND Jac _jacobianOplus[0].topLeftCorner(9,6):"<<endl<< _jacobianOplus[0].topLeftCorner(9,6)<<endl;
    //cout<<"ND Jac _jacobianOplus[1].topLeftCorner(9,9):"<<endl<< _jacobianOplus[1].topLeftCorner(9,9)<<endl;
#else
    _jacobianOplus[0]=Matrix<double,15, 6>::Zero(); //15x6
    _jacobianOplus[1]=Matrix<double,15, 9>::Zero(); //15x9
    const int kGlobalSize=7, kLocalSize=6, num_outputs=9, sbDim=9;
    Matrix<double, kLocalSize, 1> zero_delta= Matrix<double, kLocalSize, 1>::Zero();

    typedef ceres::internal::AutoDiff<G2oEdgeIMUConstraint, double, kGlobalSize,
            kLocalSize, sbDim, kGlobalSize, sbDim > IMUConstraintAutoDiff;
    Matrix<double, num_outputs, kLocalSize, Eigen::RowMajor> dError_dTw2ck;
    Matrix<double, num_outputs, sbDim, Eigen::RowMajor> dError_dsb;
    const double *parameters[] = { T_1w.data(), zero_delta.data(), sb_1.data(), T_2w.data(), sb_2.data()};
    double value[num_outputs];
    double *jacobians[] = { NULL, dError_dTw2ck.data(), dError_dsb.data(), NULL, NULL };
    bool diffState = IMUConstraintAutoDiff::Differentiate(*this, parameters, num_outputs, value, jacobians);
    // copy over the Jacobians (convert row-major -> column-major)
    if (diffState) {
        _jacobianOplus[0].topLeftCorner(9,6) = dError_dTw2ck;
        _jacobianOplus[1].topLeftCorner(9,9) = dError_dsb;
    } else {
        assert(0 && "Error while AD differentiating");
    }
    //    cout<<"AD Jac dError_dTw2ck:"<<endl<< dError_dTw2ck <<endl;
    //    cout<<"AD Jac dError_dsb:"<<endl<< dError_dsb <<endl;
#endif
    _jacobianOplus[1].bottomRightCorner(6,6).setIdentity();

    //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)

    SE3d Ts1tow=(params_imu->T_imu_from_cam*T_1w).inverse();
    SE3d pred_T_s2_to_w;
    Vector3d pred_speed_2;
    Eigen::Matrix<double, 15,15> *holder=NULL;
    predictStates(Ts1tow, sb_1, time_frames,
                  _measurement, params_imu->gwomegaw, params_imu->q_n_aw_babw,
                  &pred_T_s2_to_w, &pred_speed_2, holder);
    SE3d predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam;
    Eigen::Matrix<double, 6,1> error_fe= SE3d::log(predTckp12w*T_2w);
    _jacobianOplus[2]=Matrix<double, 15, 6>::Zero(); //15x6
    _jacobianOplus[2].topLeftCorner(6,6)= third(predTckp12w,  error_fe);

    _jacobianOplus[3]=Matrix<double, 15, 9>::Zero(); //15x9
    _jacobianOplus[3].bottomRightCorner(9,9)= -Matrix<double, 9,9>::Identity();
}
void IMURollPitchEdge::computeError()
{
    const G2oVertexSE3* v_se3 = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexSpeedBias* v_sb = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));

    Eigen::Quaterniond qw2s=(params_imu->T_imu_from_cam*v_se3->estimate()).unit_quaternion();
    _error = params_imu->gwomegaw.head(3)+qw2s.conjugate()._transformVector(_measurement-v_sb->estimate().segment(3,3));
}
void IMURollPitchEdge::
linearizeOplus()
{
    G2oVertexSE3 * vpose = static_cast<G2oVertexSE3 *>(_vertices[0]);
    SE3d Tw2c = vpose->estimate();//T world to cam frame
    G2oVertexSpeedBias * v_sb = static_cast<G2oVertexSpeedBias *>(_vertices[1]);
    Matrix<double, 9,1> v9sb = v_sb->estimate();
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));
    const Eigen::Quaterniond qw2s=(params_imu->T_imu_from_cam*Tw2c).unit_quaternion();
    _jacobianOplusXi.setZero(); // 3x6
#if 0
    const double h=1e-8;
    for (unsigned int i=3; i<6; ++i) //eps=[trans, rot]
    {
        Matrix<double,6,1> eps
                = Matrix<double,6,1>::Zero();
        eps[i] = h;
        Quaterniond pred_qw2s=(params_imu->T_imu_from_cam*SE3d::exp(eps)*Tw2c).unit_quaternion();
        Vector3d pred_err=params_imu->gwomegaw.head(3)+pred_qw2s.conjugate()._transformVector(_measurement-v9sb.segment(3,3));
        _jacobianOplusXi.col(i)= (pred_err-_error)/h;
    }
#else
    _jacobianOplusXi.block(0,3,3,3) = Tw2c.rotationMatrix().transpose()*
            skew(params_imu->T_imu_from_cam.unit_quaternion().conjugate()._transformVector(_measurement-v9sb.segment(3,3)));
#endif
    _jacobianOplusXj.setZero(); //3x9
    _jacobianOplusXj.block(0,3,3,3) = - qw2s.conjugate().toRotationMatrix();
}

void GPSObservationPosition3DEdge::linearizeOplus()
{
    const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
    SE3d Tw2c= v->estimate();
#if 0
    //numerical differentiation
    const double h=1e-8;
    for (unsigned int i=0; i<6; ++i) //eps=[trans, rot]
    {
        Matrix<double,6,1> eps
                = Matrix<double,6,1>::Zero();
        eps[i] = h;
        SE3d temp_T=SE3d::exp(eps)*Tw2c;
        _jacobianOplusXi.col(i)= (temp_T.inverse().translation()-_measurement - _error)/h;
    }
#else
    _jacobianOplusXi.block(0,0,3,3)=Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block(0,3,3,3)=Eigen::Matrix3d::Zero();
    _jacobianOplusXi= - Tw2c.rotationMatrix().transpose()*_jacobianOplusXi;
#endif
}
void G2oEdgeGPSObservation::linearizeOplus()
{
    const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexLeverArm* vl = static_cast<const G2oVertexLeverArm*>(_vertices[1]);
    SE3d Tw2c= v->estimate();
    _jacobianOplusXi.block(0,0,3,3)=Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block(0,3,3,3)= -skew(Tw2c* _measurement);
    _jacobianOplusXj= - Eigen::Matrix3d::Identity();
}
bool G2oVertexGSCamera
::write (std::ostream & os) const
{
  const Eigen::Matrix<double, 8,1> & lv = estimate();
  for (int i=0; i<8; ++i)
  {
    os << lv[i] << " ";
  }
  return true;
}

bool G2oVertexGSCamera
::read(std::istream& is)
{
  Eigen::Matrix<double, 8,1> lv;
  for (int i=0; i<8; ++i)
  {
    is >> lv[i];
  }
  setEstimate(lv);
  return true;
}
void testG2OVertexGSCamera()
{
    ScaViSLAM::G2oVertexGSCamera gscam;
    gscam.setToOriginImpl();
    gscam.write(std::cout);
    cout<<endl;
    double update[7]= {1,2,3,4,5,6, 7};
    gscam.oplusImpl(update);
    gscam.write(std::cout);
    cout<<endl;
    double update2[7]= {8,9,10,11,12,13, 14};
    gscam.oplusImpl(update2);
    gscam.write(std::cout);
    cout<<endl;
    Eigen::Matrix<double, 6,1> x;
    x<< update[0],update[1],update[2],update[3],update[4],update[5];
    Eigen::Matrix<double, 6,1> x2;
    x2<< update2[0],update2[1],update2[2],update2[3],update2[4],update2[5];
    Sophus::SE3d truth = Sophus::SE3d::exp(x2)*Sophus::SE3d::exp(x);
    for(int jack=0; jack<7;++jack)
    cout<<truth.data()[jack]<<" ";
    cout<<endl;
}
}
