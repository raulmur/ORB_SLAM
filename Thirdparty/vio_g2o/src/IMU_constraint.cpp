
#include "vio_g2o/IMU_constraint.h"
#include "vio/timegrabber.h" //imugrabber

#include "ceres/autodiff.h" //autodiff included in Thirdparty/g2o_External
namespace vio{
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
    value.template head<6>()=SE3Group<T>::log(predTckp12w*se3Tw2ckp1);
    value.template tail<3>()=pred_speed_2-Xsbkp1.template head<3>();
    return true;
}

//added shape matrices compared to G2oEdgeIMUConstraint operator()
// pSags is the data pointer to S_a, S_g, T_s with elements in row major order
template <typename T>
bool G2oEdgeIMUConstraintEx
::operator ()( const  T* pTw2ck, const T* epsilonk, const T* pXsbk, const T* pTw2ckp1,
               const T* pXsbkp1, const T* pSags, T* error) const
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
    Eigen::Matrix<T, 27,1 > Sags= Eigen::Map<const Eigen::Matrix<T, 27,1 > >(pSags);
    predictStates(T_s1_to_w, cast_Xsbk, cast_time_frames,
                  cast_measurement, cast_gwomegaw, cast_q,
                  &pred_T_s2_to_w, &pred_speed_2, holder,Sags);
    SE3Group<T> predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam.cast<T>();
    value.template head<6>()=SE3Group<T>::log(predTckp12w*se3Tw2ckp1);
    value.template tail<3>()=pred_speed_2-Xsbkp1.template head<3>();
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
    _error.head<9>()=error_posvel;
    _error.tail<6>()=(speed_bias_1->estimate()).tail<6>()-(speed_bias_2->estimate()).tail<6>();
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
        se3Tskp12w.translation() =se3Tskp12wConst.translation() - deltaxpsi.template head<3>();
        Matrix<T,3,1> rvec= deltaxpsi.template tail<3>();
        se3Tskp12w.setQuaternion(vio::quaternionFromSmallAngle(rvec)*se3Tskp12wConst.unit_quaternion());

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
        J_pred.col(i).head<6>() = diff/h;
    }
    //take derivative w.r.t perturbation in predicted rotation
    for (unsigned int i=0; i<3; ++i)
    {
        SE3d temp_T=pred_T_s2_to_w;
        Vector3d rvec=Vector3d::Zero();
        rvec[i]=h;
        temp_T.setQuaternion(vio::quaternionFromSmallAngle(rvec)*temp_T.unit_quaternion());
        diff=SE3d::log(temp_T*params_imu.T_imu_from_cam*T_c2_from_world->estimate());
        diff-=orig_pose_error;
        J_pred.col(i+6).head<6>() = diff/h; // +6 because $\psi^w$ corresponds to 6-8 rows/cols of P
    }
    cout<<"ND J_pred.block<6,9>(0,0):"<<endl<< J_pred.block<6,9>(0,0)<<endl;

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
    J_pred.block<6,3>(0,0)= dTinv_de_AD.block<6,3>(0,0);
    J_pred.block<6,3>(0,6)= dTinv_de_AD.block<6,3>(0,3);
    //cout<<"AD J_pred.block<6,9>(0,0):"<<endl<< J_pred.block<6,9>(0,0)<<endl;
#endif
    J_pred.block<3,3>(6,3)=Matrix3d::Identity();//delta velocity
    J_pred.block<3,3>(9,9)=Matrix3d::Identity();//b_a
    J_pred.block<3,3>(12,12)=Matrix3d::Identity();//b_g
    //TODO: verify that numerical Jacobian satisfy $J_pred\approx diag(third(_error.head<6>(), SE3d()),I)$
    Eigen::Matrix<double, 15, 15> Atemp = J_pred*(0.5*P + 0.5*P.transpose())*J_pred.transpose();
    _information=Atemp.llt().solve(Eigen::Matrix<double,15,15>::Identity());
    //    _information=Atemp.inverse(); // this is less efficient and prone to numerical issues
    _information= 0.5*_information+ 0.5*_information.transpose().eval();

    //    cout<<"new P(9,9)"<<P.topLeftCorner<9,9>()<<endl;
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
    const Matrix<double, 6,1> orig_pose_error=_error.head<6>();
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
        _jacobianOplus[0].col(i).head<6>() = diff/h;
        _jacobianOplus[0].col(i).segment<3>(6) = ((pred_speed_2- sb_2.head<3>())-_error.segment<3>(6))/h;
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
        _jacobianOplus[1].col(i).head<6>() = diff/h;
        _jacobianOplus[1].col(i).segment<3>(6) = ((pred_speed_2- sb_2.head<3>())-_error.segment<3>(6))/h;
    }
    cout<<"ND Jac _jacobianOplus[0].topLeftCorner<9,6>():"<<endl<< _jacobianOplus[0].topLeftCorner<9,6>()<<endl;
    cout<<"ND Jac _jacobianOplus[1].topLeftCorner<9,9>():"<<endl<< _jacobianOplus[1].topLeftCorner<9,9>()<<endl;
#endif
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
        _jacobianOplus[0].topLeftCorner<9,6>() = dError_dTw2ck;       
        _jacobianOplus[1].topLeftCorner<9,9>() = dError_dsb;
    } else {
        assert(0 && "Error while AD differentiating");
    }
#if 0
    cout<<"AD Jac dError_dTw2ck:"<<endl<< dError_dTw2ck <<endl;
    cout<<"AD Jac dError_dsb:"<<endl<< dError_dsb <<endl;
#endif
    _jacobianOplus[1].bottomRightCorner<6,6>().setIdentity();

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
    _jacobianOplus[2].topLeftCorner<6,6>()= third(predTckp12w,  error_fe);

    _jacobianOplus[3]=Matrix<double, 15, 9>::Zero(); //15x9
    _jacobianOplus[3].bottomRightCorner<9,9>()= -Matrix<double, 9,9>::Identity();
}
void IMURollPitchEdge::computeError()
{
    const G2oVertexSE3* v_se3 = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexSpeedBias* v_sb = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));

    Eigen::Quaterniond qw2s=(params_imu->T_imu_from_cam*v_se3->estimate()).unit_quaternion();
    _error = params_imu->gwomegaw.head<3>()+qw2s.conjugate()._transformVector(_measurement-v_sb->estimate().segment<3>(3));
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
        Vector3d pred_err=params_imu->gwomegaw.head<3>()+pred_qw2s.conjugate()._transformVector(_measurement-v9sb.segment<3>(3));
        _jacobianOplusXi.col(i)= (pred_err-_error)/h;
    }
#else
    _jacobianOplusXi.block<3,3>(0,3) = Tw2c.rotationMatrix().transpose()*
            skew3d(params_imu->T_imu_from_cam.unit_quaternion().conjugate()._transformVector(_measurement-v9sb.segment<3>(3)));
#endif
    _jacobianOplusXj.setZero(); //3x9
    _jacobianOplusXj.block<3,3>(0,3) = - qw2s.conjugate().toRotationMatrix();
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
    _jacobianOplusXi.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(0,3)=Eigen::Matrix3d::Zero();
    _jacobianOplusXi= - Tw2c.rotationMatrix().transpose()*_jacobianOplusXi;
#endif
}
void G2oEdgeGPSObservation::linearizeOplus()
{
    const G2oVertexSE3* v = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexLeverArm* vl = static_cast<const G2oVertexLeverArm*>(_vertices[1]);
    SE3d Tw2c= v->estimate();
    _jacobianOplusXi.block<3,3>(0,0)= Tw2c.rotationMatrix().transpose();
    _jacobianOplusXi.block<3,3>(0,3)= -Tw2c.rotationMatrix().transpose()*skew3d(vl->estimate());
    _jacobianOplusXj= - Tw2c.rotationMatrix().transpose();
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
    G2oVertexGSCamera gscam;
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
bool G2oEdgeIMUConstraintEx
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
bool G2oEdgeIMUConstraintEx::read(std::istream& is) {
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
void G2oEdgeIMUConstraintEx
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
    const G2oVertexShapeMatrices * shape_matrices
            = static_cast<const G2oVertexShapeMatrices*>(_vertices[4]);
    Eigen::Matrix<double, 9,1 > error_posvel= Eigen::Matrix<double, 9,1 >::Zero();
    Matrix<double, 6, 1> zero_delta= Matrix<double, 6, 1>::Zero();
    (*this)(T_c1_from_world->estimate().data(), zero_delta.data(), speed_bias_1->estimate().data(),
            T_c2_from_world->estimate().data(), speed_bias_2->estimate().data(),
            shape_matrices->estimate().data(), error_posvel.data() );
    _error.head<9>()=error_posvel;
    _error.tail<6>()=(speed_bias_1->estimate()).tail<6>()-(speed_bias_2->estimate()).tail<6>();
}

//following $g^2o$: A general framework for graph optimization
//$\mathbf{J}_{ij}=\frac{\partial \mathbf{e}_{ij}(\breve{\mathbf{x}}\boxplus \Delta\mathbf{x})}{\partial \mathbf{\Delta x}}\vert_{\mathbf{\Delta x}=0}$
void G2oEdgeIMUConstraintEx::linearizeOplus()
{
    G2oVertexSE3 * vpose_1 = static_cast<G2oVertexSE3 *>(_vertices[0]);
    SE3d T_1w =  (vpose_1->first_estimate==NULL? vpose_1->estimate(): *(vpose_1->first_estimate)); //T world to 1 frame
    G2oVertexSpeedBias* vsb_1 = static_cast<G2oVertexSpeedBias*>(_vertices[1]);
    Matrix<double, 9,1> sb_1 = (vsb_1->first_estimate==NULL? vsb_1->estimate(): *(vsb_1->first_estimate));
    G2oVertexSE3 * vpose_2 = static_cast<G2oVertexSE3 *>(_vertices[2]);
    SE3d T_2w =(vpose_2->first_estimate==NULL? vpose_2->estimate(): *(vpose_2->first_estimate));
    G2oVertexSpeedBias* vsb_2 = static_cast<G2oVertexSpeedBias*>(_vertices[3]);
    Matrix<double, 9,1> sb_2 = (vsb_2->first_estimate==NULL? vsb_2->estimate(): *(vsb_2->first_estimate));
    G2oVertexShapeMatrices* vsm = static_cast<G2oVertexShapeMatrices*>(_vertices[4]);
    Matrix<double, 27,1> sm = (vsm->first_estimate==NULL? vsm->estimate(): *(vsm->first_estimate));
    const G2oIMUParameters * params_imu
            = static_cast<const G2oIMUParameters *>(parameter(0));

    _jacobianOplus[0]=Matrix<double,15, 6>::Zero(); //15x6
    _jacobianOplus[1]=Matrix<double,15, 9>::Zero(); //15x9
    _jacobianOplus[4]=Matrix<double,15, 27>::Zero(); //15x27
    const int kGlobalSize=7, kLocalSize=6, num_outputs=9, sbDim=9, smDim=27;
    Matrix<double, kLocalSize, 1> zero_delta= Matrix<double, kLocalSize, 1>::Zero();

    typedef ceres::internal::AutoDiff<G2oEdgeIMUConstraintEx, double, kGlobalSize,
            kLocalSize, sbDim, kGlobalSize, sbDim, smDim > IMUConstraintAutoDiff;
    Matrix<double, num_outputs, kLocalSize, Eigen::RowMajor> dError_dTw2ck;
    Matrix<double, num_outputs, sbDim, Eigen::RowMajor> dError_dsb;
    Matrix<double, num_outputs, smDim, Eigen::RowMajor> dError_dsm;
    const double *parameters[] = { T_1w.data(), zero_delta.data(), sb_1.data(), T_2w.data(), sb_2.data(), sm.data()};
    double value[num_outputs];
    double *jacobians[] = { NULL, dError_dTw2ck.data(), dError_dsb.data(), NULL, NULL, dError_dsm.data() };
    bool diffState = IMUConstraintAutoDiff::Differentiate(*this, parameters, num_outputs, value, jacobians);
    // copy over the Jacobians (convert row-major -> column-major)
    if (diffState) {
        _jacobianOplus[0].topLeftCorner<9,6>() = dError_dTw2ck;      
        _jacobianOplus[1].topLeftCorner<9,9>() = dError_dsb;
        _jacobianOplus[4].topLeftCorner<9,27>() = dError_dsm;
    } else {
        assert(0 && "Error while AD differentiating");
    }
    //    cout<<"AD Jac dError_dTw2ck:"<<endl<< dError_dTw2ck <<endl;
    //    cout<<"AD Jac dError_dsb:"<<endl<< dError_dsb <<endl;

    _jacobianOplus[1].bottomRightCorner<6,6>().setIdentity();

    //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)

    SE3d Ts1tow=(params_imu->T_imu_from_cam*T_1w).inverse();
    SE3d pred_T_s2_to_w;
    Vector3d pred_speed_2;
    Eigen::Matrix<double, 15,15> *holder=NULL;
    predictStates(Ts1tow, sb_1, time_frames,
                  _measurement, params_imu->gwomegaw, params_imu->q_n_aw_babw,
                  &pred_T_s2_to_w, &pred_speed_2, holder, sm );
    SE3d predTckp12w=  pred_T_s2_to_w*params_imu->T_imu_from_cam;
    Eigen::Matrix<double, 6,1> error_fe= SE3d::log(predTckp12w*T_2w);
    _jacobianOplus[2]=Matrix<double, 15, 6>::Zero(); //15x6
    _jacobianOplus[2].topLeftCorner<6,6>()= third(predTckp12w,  error_fe);

    _jacobianOplus[3]=Matrix<double, 15, 9>::Zero(); //15x9
    _jacobianOplus[3].bottomRightCorner<9,9>()= -Matrix<double, 9,9>::Identity();
}
// compute information matrix
void G2oEdgeIMUConstraintEx
::calcAndSetInformation(const G2oIMUParameters & params_imu)
{
    //first two nodes
    const G2oVertexSE3 * T_c1_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[0]);
    const G2oVertexSpeedBias * speed_bias_1
            = static_cast<const G2oVertexSpeedBias*>(_vertices[1]);
    //second two nodes
    const G2oVertexSE3 * T_c2_from_world
            = static_cast<const G2oVertexSE3*>(_vertices[2]);
    const G2oVertexShapeMatrices * Sags
            = static_cast<const G2oVertexShapeMatrices*>(_vertices[4]);
    //given IMU measurements, and states at time 1,i.e., t(k), predict states at time 2, i.e., t(k+1)
    SE3d T_s1_to_w=(params_imu.T_imu_from_cam*T_c1_from_world->estimate()).inverse();
    SE3d pred_T_s2_to_w;
    Vector3d pred_speed_2;
    Matrix<double, 15,15> P= Matrix<double, 15,15>::Identity()*1e-10; // covariance of first states are 0 in this case

    predictStates(T_s1_to_w, speed_bias_1->estimate(), time_frames,
                  _measurement, params_imu.gwomegaw, params_imu.q_n_aw_babw,
                  &pred_T_s2_to_w, &pred_speed_2, &P, Sags->estimate());
    Matrix<double,15,15> J_pred
            = Matrix<double,15,15>::Zero();

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
    J_pred.block<6,3>(0,0)= dTinv_de_AD.block<6,3>(0,0);
    J_pred.block<6,3>(0,6)= dTinv_de_AD.block<6,3>(0,3);
    //cout<<"AD J_pred.block<6,9>(0,0):"<<endl<< J_pred.block<6,9>(0,0)<<endl;

    J_pred.block<3,3>(6,3)=Matrix3d::Identity();//delta velocity
    J_pred.block<3,3>(9,9)=Matrix3d::Identity();//b_a
    J_pred.block<3,3>(12,12)=Matrix3d::Identity();//b_g

    _information=(J_pred*P*J_pred.transpose()).inverse();

    //    cout<<"new P(9,9)"<<P.topLeftCorner<9,9>()<<endl;
    //    cout<<_information.diagonal().transpose()<<endl;
}

IMUProcessor::IMUProcessor(const G2oIMUParameters &imu):
    speed_bias_1(Eigen::Matrix<double, 9,1>::Zero()), pred_speed_bias_2(Eigen::Matrix<double, 9,1>::Zero()),
    imu_(imu), bStatesInitialized(false),
    P_(Eigen::Matrix<double,15,15>::Identity()),bPredictCov(false)
{
    time_pair[0]=-1;
    time_pair[1]=-1;
}

Sophus::SE3d IMUProcessor::propagate(const double time_frame, const
                                     std::vector<Eigen::Matrix<double, 7, 1> > & imuMeas)
{
  
    time_pair[0]=time_pair[1];
    time_pair[1]=time_frame;

    Eigen::Vector3d tempVs0inw;

    Eigen::Matrix<double, 15,15>* holder=NULL;
    if(bPredictCov)
        holder= &P_;

    predictStates(T_s1_to_w, speed_bias_1, time_pair,
                             imuMeas, imu_.gwomegaw, imu_.q_n_aw_babw,
                             &pred_T_s2_to_w, &tempVs0inw, holder);
    pred_speed_bias_2.head<3>()=tempVs0inw;
    pred_speed_bias_2.tail<6>()=speed_bias_1.tail<6>();     //biases do not change in propagation
    Sophus::SE3d pred_Tr_delta=pred_T_s2_to_w*imu_.T_imu_from_cam;
#ifdef ONLY_USE_IMU     //for IMU debug
    out_stream<<i<<" "<< pred_Tr_delta.matrix().row(0)<<" "<<pred_Tr_delta.matrix().row(1)<<" ";
    out_stream<<pred_Tr_delta.matrix().row(2)<<" "<<pred_speed_bias_2.transpose()<<endl;
#endif
    pred_Tr_delta=pred_Tr_delta.inverse()*(T_s1_to_w*imu_.T_imu_from_cam);
    //                        pred_Tr_delta.translation().setZero();  // remove the translation part
    T_s1_to_w=pred_T_s2_to_w;
    speed_bias_1=pred_speed_bias_2;
    return pred_Tr_delta;
}

void IMUProcessor::printStateAndCov(std::ofstream &output, double time)const {
    output<<setprecision(10)<< time<<" "<<pred_T_s2_to_w.translation().transpose()<<" "<< pred_speed_bias_2.head<3>().transpose() <<" ";
    output<< pred_T_s2_to_w.unit_quaternion().coeffs().transpose()<<" "<<pred_speed_bias_2.tail<6>().transpose()<<" ";
    Eigen::Matrix<double, 15,1> Pdiag=P_.diagonal();
    Eigen::Matrix<double, 15,1> stdDiag;
    for(int jack=0; jack<15;++jack)
    {
        if(Pdiag[jack]<0)
            cout<<"Warning P diagonal elements negative at ("<<jack<<","<<jack<<")"<<endl;
        stdDiag[jack]= sqrt(abs(Pdiag[jack]));
    }
    output<<stdDiag.transpose()<<endl;
}

void IMUProcessor::freeInertial(vio::IMUGrabber& ig, std::string output_file, double finish_time)
{
    int every_n_reading=3;// update covariance every n IMU readings
    imu_traj_stream.open(output_file.c_str(), std::ios::out);
    imu_traj_stream<<"% time/sec, pos of sensor IMU in the local world NED frame, vsinw, qs2w(xyzw), ba, bg, std of [delta rsinw, delta vsinw, psi_w, ba, bg]"<<endl;

    bool is_meas_good=ig.getObservation(finish_time);

    if(!is_meas_good)
    {
        if(ig.measurement.size()){
            std::cout <<"IMU measurement start time "<<ig.measurement.front()[0]<<std::endl;
            std::cout <<"IMU measurement finish time "<<ig.measurement.back()[0]<<std::endl;
        }
        std::cerr<<"Error getting valid inertial observations!"<<std::endl;
        imu_traj_stream.close();
        return;
    }
    time_pair[0]=time_pair[1];
    time_pair[1]=ig.measurement.back()[0] - 0.001;

    pred_T_s2_to_w= T_s1_to_w;
    pred_speed_bias_2= speed_bias_1;
    printStateAndCov(imu_traj_stream, time_pair[0]);
#if 0 //the following two implementation gives almost identical results
    Eigen::Matrix<double, 3,1> pred_speed_2;
    predictStates(T_s1_to_w, speed_bias_1, time_pair,
                             ig.measurement, imu_.gwomegaw, imu_.q_n_aw_babw,
                             &pred_T_s2_to_w, &pred_speed_2, &P_);
    cout<<"Final states and sqrt(P) diagonal elements:"<<endl;
    cout<<pred_T_s2_to_w.translation().transpose()<<" "<< pred_speed_2.transpose() <<endl;
    cout<< pred_T_s2_to_w.unit_quaternion().coeffs().transpose()<<endl<<speed_bias_1.tail<6>().transpose()<<endl;
    Eigen::Matrix<double, 15,1> Pdiag=P_.diagonal();
    Eigen::Matrix<double, 15,1> stdDiag;
    for(int jack=0; jack<15;++jack)
    {
        if(Pdiag[jack]<0)
            cout<<"Warning P diagonal elements negative at ("<<jack<<","<<jack<<")"<<endl;
        stdDiag[jack]= sqrt(abs(Pdiag[jack]));
    }
    cout<<endl<<stdDiag.transpose()<<endl;
#else// identical to case 1 except verbose output
    Vector3d r_new, r_old(T_s1_to_w.translation()), v_new, v_old(speed_bias_1.head<3>());
    Quaterniond q_new, q_old(T_s1_to_w.unit_quaternion().conjugate());

    vector<Eigen::Matrix<double, 7,1> > measurements=ig.measurement;
    double dt=measurements[1][0]-time_pair[0];
    double covupt_time(time_pair[0]);
    assert(dt>0);
    Eigen::Matrix<double,6,1> b_ga;
    b_ga.head<3>()= speed_bias_1.tail<3>();
    b_ga.tail<3>()= speed_bias_1.segment<3>(3);
    IMUErrorModel<double> iem(b_ga);
    iem.estimate(measurements[0].tail<3>(), measurements[0].segment<3>(1));

    Eigen::Matrix<double, 3,1> qna=imu_.q_n_aw_babw.head(3);
    Eigen::Matrix<double, 3,1> qnw=imu_.q_n_aw_babw.segment(3,3);
    Eigen::Matrix<double, 3,1> qnba=imu_.q_n_aw_babw.segment(6,3);
    Eigen::Matrix<double, 3,1> qnbw=imu_.q_n_aw_babw.tail(3);

    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                                          dt, imu_.gwomegaw, &r_new, &v_new, &q_new);
    if(bPredictCov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old, iem.a_est, iem.w_est,
                                      measurements[1][0]-covupt_time,qna, qnw, qnba, qnbw,&P_);
        covupt_time=measurements[1][0];
    }
    pred_T_s2_to_w.setQuaternion(q_new.conjugate());
    pred_T_s2_to_w.translation()=r_new;
    pred_speed_bias_2.head<3>()=v_new;
    printStateAndCov(imu_traj_stream, measurements[1][0]);

    r_old=r_new;
    v_old=v_new;
    q_old=q_new;
    int unsigned jack=1;
    for (; jack<measurements.size()-1;++jack){
        dt=measurements[jack+1][0]-measurements[jack][0];
        iem.estimate(measurements[jack].tail<3>(), measurements[jack].segment<3>(1));

        strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                                              dt, imu_.gwomegaw, &r_new, &v_new, &q_new);
        if(bPredictCov &&(jack%every_n_reading==0))
        {
            sys_local_dcm_bias(r_old, v_old, q_old,iem.a_est, iem.w_est,
                                          measurements[jack+1][0]-covupt_time,qna, qnw, qnba, qnbw,&P_);
            covupt_time=measurements[jack+1][0];
        }
        pred_T_s2_to_w.setQuaternion(q_new.conjugate());
        pred_T_s2_to_w.translation()=r_new;
        pred_speed_bias_2.head<3>()=v_new;
        printStateAndCov(imu_traj_stream, measurements[jack+1][0]);

        r_old=r_new;
        v_old=v_new;
        q_old=q_new;
    }
    assert(jack==measurements.size()-1);
    dt=time_pair[1]-measurements[jack][0];//the last measurement
    assert(dt>=0);

    iem.estimate(measurements[jack].tail<3>(), measurements[jack].segment<3>(1));

    strapdown_local_quat_bias( r_old, v_old, q_old, iem.a_est, iem.w_est,
                                          dt, imu_.gwomegaw, &r_new, &v_new, &q_new);
    if(bPredictCov)
    {
        sys_local_dcm_bias(r_old, v_old, q_old,iem.a_est, iem.w_est,
                                      time_pair[1]-covupt_time,qna, qnw, qnba, qnbw,&P_);
        covupt_time=time_pair[1];
    }
    pred_T_s2_to_w.setQuaternion(q_new.conjugate());
    pred_T_s2_to_w.translation()=r_new;
    pred_speed_bias_2.head<3>()=v_new;
    printStateAndCov(imu_traj_stream, time_pair[1]);
#endif
    imu_traj_stream.close();
}

bool IMUProcessor::initStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1, const double timestamp, Eigen::Matrix<double, 15, 15> *pCov)
{

    bStatesInitialized=true;

    time_pair[0]=time_pair[1];
    time_pair[1]=timestamp;
    resetStates(Ts1tow, sb1);

    if(pCov!=NULL){
        P_= *pCov;
        bPredictCov=true;
    }
    return true;
}

void IMUProcessor::resetStates(const Sophus::SE3d &Ts1tow, const Eigen::Matrix<double, 9,1> & sb1)
{
    T_s1_to_w=Ts1tow;
    speed_bias_1=sb1;
}

}
