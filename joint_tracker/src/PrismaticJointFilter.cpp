#include "joint_tracker/PrismaticJointFilter.h"

#include <Eigen/Geometry>

#include <fstream>

#include <sstream>

#include <Eigen/Eigenvalues>

#include <boost/math/distributions/chi_squared.hpp>

#include <omip_common/OMIPUtils.h>

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

// Dimensions of the system state of the filter that tracks a prismatic joint: orientation (2 values), joint variable, and joint velocity
#define PRISM_STATE_DIM 5
#define MEAS_DIM 6

/**
 * EKF internal state:
 *
 * x(1) =  PrismJointOrientation_x
 * x(2) =  PrismJointOrientation_y
 * x(3) =  PrismJointOrientation_z
 * PrismJointOrientation is represented in cartesian coords
 * x(4) = PrismJointVariable
 * x(5) = PrismJointVariable_d
 *
 * EKF measurement:
 *
 * m(1) = TwistLinearPart_x
 * m(2) = TwistLinearPart_y
 * m(3) = TwistLinearPart_z
 * m(4) = TwistAngularPart_x
 * m(5) = TwistAngularPart_y
 * m(6) = TwistAngularPart_z
 */

PrismaticJointFilter::PrismaticJointFilter():
    JointFilter(),
    _sys_PDF(NULL),
    _sys_MODEL(NULL),
    _meas_PDF(NULL),
    _meas_MODEL(NULL),
    _ekf(NULL),
    _sigma_delta_meas_uncertainty_linear(-1)
{

}

void PrismaticJointFilter::setCovarianceDeltaMeasurementLinear(double sigma_delta_meas_uncertainty_linear)
{
    this->_sigma_delta_meas_uncertainty_linear = sigma_delta_meas_uncertainty_linear;
}



void PrismaticJointFilter::setInitialMeasurement(const joint_measurement_t &initial_measurement,
                                        const Eigen::Twistd& rrb_pose_at_srb_birth_in_sf,
                                        const Eigen::Twistd& srb_pose_at_srb_birth_in_sf)
{
    // Extract current pose of the reference RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // PoseCoord({rrbf}|RRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.first.pose_wc.twist, this->_rrb_pose_in_sf);
    this->_rrb_previous_pose_in_sf = this->_rrb_pose_in_sf;

    // Extract current velocity of the reference RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({rrbf}|RRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.first.velocity_wc.twist, this->_rrb_vel_in_sf);

    // Extract current pose of the second RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({sfbf}|SRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.second.pose_wc.twist, this->_srb_pose_in_sf);

    // Extract current velocity of the second RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({sfbf}|SRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.second.velocity_wc.twist, this->_srb_vel_in_sf);

    // Extract covariance of the reference and the second RB wrt sensor frame
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            this->_rrb_pose_cov_in_sf(i, j) = initial_measurement.first.pose_wc.covariance[6 * i + j];
            this->_rrb_vel_cov_in_sf(i, j) = initial_measurement.first.velocity_wc.covariance[6 * i + j];
            this->_srb_pose_cov_in_sf(i, j) = initial_measurement.second.pose_wc.covariance[6 * i + j];
        }
    }

    // Estimate the initial pose of the second RB frame relative to the initial reference RB frame in initial reference RB frame coordinates
    // TwistCoord({srbf}|SRB,{rrbf}|RRB,[rrbf])(0)
    Eigen::Displacementd T_sf_srbf_t0 = srb_pose_at_srb_birth_in_sf.exp(1e-20);
    Eigen::Displacementd T_sf_rrbf_t0 = rrb_pose_at_srb_birth_in_sf.exp(1e-20);
    Eigen::Displacementd T_rrbf_sf_t0 = T_sf_rrbf_t0.inverse();
    Eigen::Displacementd T_rrbf_srbf_t0 = T_rrbf_sf_t0 * T_sf_srbf_t0;
    this->_srb_initial_pose_in_rrbf = T_rrbf_srbf_t0.log(1.0e-20);

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> rrb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf_t0, _rrb_pose_cov_in_sf, rrb_pose_cov_in_rrbf);
    Eigen::Matrix<double, 6, 6> srb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf_t0, _srb_pose_cov_in_sf, srb_pose_cov_in_rrbf);
    this->_srb_initial_pose_cov_in_rrbf = rrb_pose_cov_in_rrbf + srb_pose_cov_in_rrbf;

    // Estimate the current pose of the second RB frame relative to the current reference RB frame in current reference RB frame coordinates
    // TwistCoord({srbf}|SRB,{rrbf}|RRB,[rrbf])(t)
    Eigen::Displacementd T_sf_rrbf_t = this->_rrb_pose_in_sf.exp(1e-20);
    Eigen::Displacementd T_sf_srbf_t = this->_srb_pose_in_sf.exp(1e-20);
    Eigen::Displacementd T_rrbf_sf_t = T_sf_rrbf_t.inverse();
    Eigen::Displacementd T_rrbf_srbf_t = T_rrbf_sf_t * T_sf_srbf_t;
    this->_srb_current_pose_in_rrbf = T_rrbf_srbf_t.log(1.0e-20);
    this->_srb_previous_pose_in_rrbf = this->_srb_current_pose_in_rrbf;

    // Estimate the transformation between the initial and the current pose of the second RB expressed in the coordinates of the reference RB frame:

    //This lines change the coordinate frame in which the change in relative pose is measured
    //There is no BEST selection: while using the second rigid body usually leads to more accurate and stable joint estimations
    //it fails if the reference rigid body moves, e.g. if it rotates, because this rotation of the reference rigid body is seen as a translation
    //of the second rigid body when we use the second rigid body as reference and the radius is large
    //WRT second rigid body
    //Eigen::Displacementd delta_displ = (T_rrbf_srbf_t0.inverse()) * T_rrbf_srbf_t;
    //this->_current_delta_pose_in_rrbf = delta_displ.log(1.0e-20);
    this->_current_delta_pose_in_rrbf = (_srb_current_pose_in_rrbf.exp(1e-12)*(_srb_initial_pose_in_rrbf.exp(1e-12).inverse())).log(1e-12);

    Eigen::Matrix4d pose_first_ht;
    Twist2TransformMatrix(_srb_initial_pose_in_rrbf, pose_first_ht);

    Eigen::Matrix4d pose_now_ht;
    Twist2TransformMatrix(_srb_current_pose_in_rrbf, pose_now_ht);

    Eigen::Vector3d displacement_between_frame_centers = Eigen::Vector3d(pose_now_ht(0,3), pose_now_ht(1,3), pose_now_ht(2,3)) -
            Eigen::Vector3d(pose_first_ht(0,3), pose_first_ht(1,3), pose_first_ht(2,3));

    this->_joint_orientation = displacement_between_frame_centers;
    this->_joint_state = this->_joint_orientation.norm();

    this->_previous_delta_pose_in_rrbf = this->_current_delta_pose_in_rrbf;
    this->_changes_in_relative_pose_in_rrbf.push_back(this->_current_delta_pose_in_rrbf);

    _rrb_id = initial_measurement.first.rb_id;
    // Extract centroid of the reference RB in sensor frame
    if(_rrb_id == 0)
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d(0.,0.,0.);
    }
    else
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d( initial_measurement.first.centroid.x,
                                                     initial_measurement.first.centroid.y,
                                                     initial_measurement.first.centroid.z);
    }

    // Extract centroid of the second RB in sensor frame
    this->_srb_centroid_in_sf = Eigen::Vector3d( initial_measurement.second.centroid.x,
                                                 initial_measurement.second.centroid.y,
                                                 initial_measurement.second.centroid.z);
}

void PrismaticJointFilter::initialize()
{
    JointFilter::initialize();

    Eigen::Matrix4d  _current_delta_pose_in_rrbf_matrix;
    Twist2TransformMatrix( _current_delta_pose_in_rrbf, _current_delta_pose_in_rrbf_matrix);

    //Then set the rotation to the identity
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
                _current_delta_pose_in_rrbf_matrix(i,j) = 1;
            }else{
                _current_delta_pose_in_rrbf_matrix(i,j) = 0;
            }
        }
    }

    Eigen::Twistd  _current_delta_pose_in_rrbf_no_rot;
    TransformMatrix2Twist(_current_delta_pose_in_rrbf_matrix, _current_delta_pose_in_rrbf_no_rot);
    _current_delta_pose_in_rrbf = _current_delta_pose_in_rrbf_no_rot;

    this->_joint_states_all.push_back(this->_joint_state);
    this->_joint_velocity = 0.0;
    this->_joint_orientation.normalize();

    // The position of the prismatic joint is the centroid of the features that belong to the second rigid body (in RBF)
    Eigen::Displacementd current_ref_pose_displ = this->_rrb_pose_in_sf.exp(1e-12);
    Eigen::Affine3d current_ref_pose;
    current_ref_pose.matrix() = current_ref_pose_displ.toHomogeneousMatrix();
    this->_joint_position = current_ref_pose.inverse() * this->_srb_centroid_in_sf;

    this->_initializeSystemModel();
    this->_initializeMeasurementModel();
    this->_initializeEKF();
}

void PrismaticJointFilter::_initializeSystemModel()
{
    // create SYSTEM MODEL
    Matrix A(PRISM_STATE_DIM, PRISM_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(4, 5) = this->_loop_period_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    ColumnVector sys_noise_MU(PRISM_STATE_DIM);
    sys_noise_MU = 0;

    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = 0.001;
    sys_noise_COV(2, 2) = 0.001;
    sys_noise_COV(3, 3) = 0.001;
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jv* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(5, 5) = this->_sigma_sys_noise_jvd*(this->_loop_period_ns/1e9);

    // Initialize System Model
    Gaussian system_uncertainty_PDF(sys_noise_MU, sys_noise_COV);
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( A, system_uncertainty_PDF);
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( this->_sys_PDF);
}

void PrismaticJointFilter::_initializeMeasurementModel()
{
    // create MEASUREMENT MODEL
    ColumnVector meas_noise_MU(MEAS_DIM);
    meas_noise_MU = 0.0;
    SymmetricMatrix meas_noise_COV(MEAS_DIM);
    meas_noise_COV = 0.0;
    for (unsigned int i = 1; i <= MEAS_DIM; i++)
        meas_noise_COV(i, i) = this->_sigma_meas_noise;

    Gaussian meas_uncertainty_PDF(meas_noise_MU, meas_noise_COV);

    this->_meas_PDF = new NonLinearPrismaticMeasurementPdf(meas_uncertainty_PDF);
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty(this->_meas_PDF);
}

void PrismaticJointFilter::_initializeEKF()
{
    ColumnVector prior_MU(PRISM_STATE_DIM);
    prior_MU = 0.0;

    SymmetricMatrix prior_COV(PRISM_STATE_DIM);
    prior_COV = 0.0;

    prior_MU(1) = _joint_orientation.x();
    prior_MU(2) = _joint_orientation.y();
    prior_MU(3) = _joint_orientation.z();

    prior_MU(4) = this->_joint_state;
    prior_MU(5) = this->_joint_velocity;

    for (int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        prior_COV(i, i) = this->_prior_cov_vel;
    }
    this->_uncertainty_joint_orientation_xyz = this->_prior_cov_vel*Eigen::Matrix3d::Identity();
    this->_uncertainty_joint_orientation_phitheta = this->_prior_cov_vel*Eigen::Matrix2d::Identity();

    Gaussian prior_PDF(prior_MU, prior_COV);

    this->_ekf = new ExtendedKalmanFilter(&prior_PDF);
}

PrismaticJointFilter::~PrismaticJointFilter()
{
    if (this->_sys_PDF)
    {
        delete this->_sys_PDF;
        this->_sys_PDF = NULL;
    }
    if (this->_sys_MODEL)
    {
        delete this->_sys_MODEL;
        this->_sys_MODEL = NULL;
    }
    if (this->_meas_PDF)
    {
        delete this->_meas_PDF;
        this->_meas_PDF = NULL;
    }
    if (this->_meas_MODEL)
    {
        delete this->_meas_MODEL;
        this->_meas_MODEL = NULL;
    }
    if (this->_ekf)
    {
        delete this->_ekf;
        this->_ekf = NULL;
    }
}

PrismaticJointFilter::PrismaticJointFilter(const PrismaticJointFilter &prismatic_joint) :
    JointFilter(prismatic_joint)
{
    this->_sigma_delta_meas_uncertainty_linear = prismatic_joint._sigma_delta_meas_uncertainty_linear;
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( *(prismatic_joint._sys_PDF));
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( *(prismatic_joint._sys_MODEL));
    this->_meas_PDF = new NonLinearPrismaticMeasurementPdf( *(prismatic_joint._meas_PDF));
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty( *(prismatic_joint._meas_MODEL));
    this->_ekf = new ExtendedKalmanFilter(*(prismatic_joint._ekf));
}

void PrismaticJointFilter::setMeasurement(joint_measurement_t acquired_measurement, const double &measurement_timestamp_ns)
{
    _measurement_timestamp_ns = measurement_timestamp_ns;

    // Extract RB ids
    _rrb_id = acquired_measurement.first.rb_id;

    // Store the previous pose of the rrb
    this->_rrb_previous_pose_in_sf = this->_rrb_pose_in_sf;
    // Extract pose of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.first.pose_wc.twist,this->_rrb_pose_in_sf);
    // Extract velocity of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.first.velocity_wc.twist, this->_rrb_vel_in_sf);
    // Extract pose of the second RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.second.pose_wc.twist, this->_srb_pose_in_sf);
    // Extract velocity of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.second.velocity_wc.twist, this->_srb_vel_in_sf);
    // Extract covariance of the reference and the second RB wrt sensor frame
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            this->_rrb_pose_cov_in_sf(i, j) = acquired_measurement.first.pose_wc.covariance[6 * i + j];
            this->_rrb_vel_cov_in_sf(i, j) = acquired_measurement.first.velocity_wc.covariance[6 * i + j];
            this->_srb_pose_cov_in_sf(i, j) = acquired_measurement.second.pose_wc.covariance[6 * i + j];
        }
    }

    Eigen::Displacementd T_sf_rrbf = this->_rrb_pose_in_sf.exp(1.0e-20);
    Eigen::Displacementd T_sf_srbf = this->_srb_pose_in_sf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_sf = T_sf_rrbf.inverse();
    Eigen::Displacementd T_rrbf_srbf = T_rrbf_sf * T_sf_srbf;
    this->_srb_current_pose_in_rrbf = T_rrbf_srbf.log(1e-20);

    // If the rigid body makes a turn of n x PI/2 the twist changes abruptly
    // We try to avoid this by comparing to the previous delta and enforcing a smooth change
    bool inverted = false;
    this->_srb_current_pose_in_rrbf = invertTwist(this->_srb_current_pose_in_rrbf, this->_srb_previous_pose_in_rrbf, inverted);

    //ROS_ERROR_STREAM("After unwrapping pose of second in ref: " << this->_srb_current_pose_in_rrbf);
    this->_srb_previous_pose_in_rrbf = this->_srb_current_pose_in_rrbf;

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> rrb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf.inverse(), _rrb_pose_cov_in_sf, rrb_pose_cov_in_rrbf);
    Eigen::Matrix<double, 6, 6> srb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf.inverse(), _srb_pose_cov_in_sf, srb_pose_cov_in_rrbf);
    this->_srb_current_pose_cov_in_rrbf = rrb_pose_cov_in_rrbf + srb_pose_cov_in_rrbf;

    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);

    // The most important line is this: now, even if we missuse the _current_delta_pose_in_rrbf variable, we actually compute the
    // _current_delta_pose_in_srbf_t0. This eliminates the spurious rotations!
    Eigen::Displacementd delta_displ = (T_rrbf_srbf_t0.inverse()) * T_rrbf_srbf;

    //This lines change the coordinate frame in which the change in relative pose is measured
    //There is no BEST selection: while using the second rigid body usually leads to more accurate and stable joint estimations
    //it fails if the reference rigid body moves, e.g. if it rotates, because this rotation of the reference rigid body is seen as a translation
    //of the second rigid body when we use the second rigid body as reference and the radius is large
    //WRT second rigid body
    //this->_current_delta_pose_in_rrbf = delta_displ.log(1.0e-20);
    //WRT reference rigid body
    this->_current_delta_pose_in_rrbf = (_srb_current_pose_in_rrbf.exp(1e-12)*(_srb_initial_pose_in_rrbf.exp(1e-12).inverse())).log(1e-12);

    // If the rigid body makes a turn of n x PI/2 the twist changes abruptly
    // We try to avoid this by comparing to the previous delta and enforcing a smooth change
    bool inverted_before = _inverted_delta_srb_pose_in_rrbf;
    this->_current_delta_pose_in_rrbf = invertTwist(this->_current_delta_pose_in_rrbf, this->_previous_delta_pose_in_rrbf, this->_inverted_delta_srb_pose_in_rrbf);
    this->_previous_delta_pose_in_rrbf = this->_current_delta_pose_in_rrbf;

    _from_inverted_to_non_inverted = false;
    _from_non_inverted_to_inverted = false;
    if(inverted_before != _inverted_delta_srb_pose_in_rrbf)
    {
        if(inverted_before && !_inverted_delta_srb_pose_in_rrbf)
        {
            _from_inverted_to_non_inverted = true;
        }else{
            _from_non_inverted_to_inverted = true;
        }
    }

    this->_changes_in_relative_pose_in_rrbf.push_back(this->_current_delta_pose_in_rrbf);

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> transformed_cov;
    adjointXinvAdjointXcovXinvAdjointTXadjointT(T_rrbf_srbf, T_rrbf_srbf_t0, _srb_initial_pose_cov_in_rrbf, transformed_cov);
    this->_current_delta_pose_cov_in_rrbf = _srb_current_pose_cov_in_rrbf + transformed_cov;

    // Extract centroid of the reference RB in sensor frame
    if(_rrb_id == 0)
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d(0.,0.,0.);
    }
    else
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d( acquired_measurement.first.centroid.x,
                                                     acquired_measurement.first.centroid.y,
                                                     acquired_measurement.first.centroid.z);
    }

    // Extract centroid of the second RB in sensor frame
    this->_srb_centroid_in_sf = Eigen::Vector3d( acquired_measurement.second.centroid.x,
                                                 acquired_measurement.second.centroid.y,
                                                 acquired_measurement.second.centroid.z);
}

void PrismaticJointFilter::predictState(double time_interval_ns)
{
    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = 0.001;
    sys_noise_COV(2, 2) = 0.001;
    sys_noise_COV(3, 3) = 0.001;
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jv* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(5, 5) = this->_sigma_sys_noise_jvd*(this->_loop_period_ns/1e9);

    // Estimate the new updating matrix which also depends on the time elapsed between the previous and the current measurement
    // x(t+1) = x(t) + v(t) * delta_t
    Matrix A(PRISM_STATE_DIM, PRISM_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(4, 5) = time_interval_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    this->_sys_PDF->MatrixSet(0, A);
    this->_sys_PDF->AdditiveNoiseSigmaSet(sys_noise_COV);
    this->_ekf->Update(this->_sys_MODEL);
}

void PrismaticJointFilter::predictMeasurement()
{
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    ColumnVector predicted_delta_pose_in_rrbf = this->_meas_MODEL->PredictionGet(empty, state_updated_state);

    this->_change_in_relative_pose_predicted_in_rrbf = Eigen::Twistd( predicted_delta_pose_in_rrbf(4), predicted_delta_pose_in_rrbf(5),
                                                         predicted_delta_pose_in_rrbf(6), predicted_delta_pose_in_rrbf(1),
                                                         predicted_delta_pose_in_rrbf(2), predicted_delta_pose_in_rrbf(3));

    Eigen::Displacementd predicted_delta = this->_change_in_relative_pose_predicted_in_rrbf.exp(1e-20);
    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);

    //Careful: changed for prismatic joints (see comment in setMeasurement)
    Eigen::Displacementd T_rrbf_srbf_t_next = T_rrbf_srbf_t0*predicted_delta;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);

    SymmetricMatrix predicted_delta_pose_in_rrbf_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             this->_change_in_relative_pose_cov_predicted_in_rrbf(i,j) = predicted_delta_pose_in_rrbf_cov(i+1,j+1);
        }
    }
}

void PrismaticJointFilter::correctState()
{
    // New 26.8.2016 -> There is small rotations in the reference frame that cause the prismatic joint to rotate
    // We eliminate this: we search for the closest motion without rotation that resembles the relative motion
    Eigen::Matrix4d current_delta_pose_in_rrbf_matrix;
    Eigen::Twistd  current_delta_pose_in_rrbf_no_rot;
    Twist2TransformMatrix( _current_delta_pose_in_rrbf, current_delta_pose_in_rrbf_matrix);

    //Then set the rotation to the identity
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
                current_delta_pose_in_rrbf_matrix(i,j) = 1;
            }else{
                current_delta_pose_in_rrbf_matrix(i,j) = 0;
            }
        }
    }

    TransformMatrix2Twist(current_delta_pose_in_rrbf_matrix, current_delta_pose_in_rrbf_no_rot);

    ColumnVector rb2_measured_delta_relative_pose_cv(MEAS_DIM);
    rb2_measured_delta_relative_pose_cv = 0.;
    rb2_measured_delta_relative_pose_cv(1) = current_delta_pose_in_rrbf_no_rot.vx();
    rb2_measured_delta_relative_pose_cv(2) = current_delta_pose_in_rrbf_no_rot.vy();
    rb2_measured_delta_relative_pose_cv(3) = current_delta_pose_in_rrbf_no_rot.vz();
    rb2_measured_delta_relative_pose_cv(4) = current_delta_pose_in_rrbf_no_rot.rx();
    rb2_measured_delta_relative_pose_cv(5) = current_delta_pose_in_rrbf_no_rot.ry();
    rb2_measured_delta_relative_pose_cv(6) = current_delta_pose_in_rrbf_no_rot.rz();

    // Update the uncertainty on the measurement
    // NEW: The uncertainty on the measurement (the delta motion of the second rigid body wrt the reference rigid body) will be large if the measurement
    // is small and small if the measurement is large
    double meas_uncertainty_factor = 1.0 / (1.0 - exp(-this->_current_delta_pose_in_rrbf.norm()/this->_sigma_delta_meas_uncertainty_linear));

    // Truncate the factor
    meas_uncertainty_factor = std::min(meas_uncertainty_factor, 1e6);

    SymmetricMatrix current_delta_pose_cov_in_rrbf(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            current_delta_pose_cov_in_rrbf(i + 1, j + 1) = _current_delta_pose_cov_in_rrbf(i, j);
        }
    }

    this->_meas_PDF->AdditiveNoiseSigmaSet(current_delta_pose_cov_in_rrbf * meas_uncertainty_factor);

    this->_ekf->Update(this->_meas_MODEL, rb2_measured_delta_relative_pose_cv);

    ColumnVector updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    // The joint is defined in the space of the relative motion
    this->_joint_orientation(0) = updated_state(1);
    this->_joint_orientation(1) = updated_state(2);
    this->_joint_orientation(2) = updated_state(3);

    this->_joint_orientation.normalize();
    updated_state(1) = this->_joint_orientation(0);
    updated_state(2) = this->_joint_orientation(1);
    updated_state(3) = this->_joint_orientation(2);
    this->_ekf->PostGet()->ExpectedValueSet(updated_state);

    SymmetricMatrix updated_uncertainty = this->_ekf->PostGet()->CovarianceGet();

    this->_joint_state = updated_state(4);
    this->_uncertainty_joint_state = updated_uncertainty(4,4);
    this->_joint_velocity = updated_state(5);
    this->_uncertainty_joint_velocity = updated_uncertainty(5,5);

    this->_joint_orientation_phi = updated_state(1);
    this->_joint_orientation_theta = updated_state(2);


    for(int i=0; i<3 ;i++)
    {
        for(int j=0; j<3; j++)
        {
            this->_uncertainty_joint_orientation_xyz(i,j) = updated_uncertainty(1+i, 1+j);
        }
    }
}

void PrismaticJointFilter::estimateMeasurementHistoryLikelihood()
{
    double accumulated_error = 0.;

    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    double x = state_updated_state(1);
    double y = state_updated_state(2);
    double z = state_updated_state(3);

    this->_joint_states_all.push_back(state_updated_state(4));

    // This vector is in the ref RB with the initial relative transformation to the second RB
    Eigen::Vector3d prism_joint_translation_unitary = Eigen::Vector3d(x, y, z);
    prism_joint_translation_unitary.normalize();

    double frame_counter = 0.;
    size_t trajectory_length = this->_changes_in_relative_pose_in_rrbf.size();
    size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);
    double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);
    size_t current_idx = 0;

    double max_norm_of_deltas = 0;

    // Estimation of the quality of the parameters of the prismatic joint
    // If the joint is prismatic and the parameters are accurate, the joint axis orientation should not change over time
    // That means that the current orientation, multiplied by the amount of prismatic displacement at each time step, should provide the delta in the relative
    // pose between ref and second rb at each time step
    // We test amount_samples of the relative trajectory
    for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
    {
        current_idx = boost::math::round(sample_idx*delta_idx_samples);
        Eigen::Displacementd rb2_last_delta_relative_displ = this->_changes_in_relative_pose_in_rrbf.at(current_idx).exp(1e-12);

        max_norm_of_deltas = std::max(this->_changes_in_relative_pose_in_rrbf.at(current_idx).norm(), max_norm_of_deltas);

        Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.qw(),
                                                                                 rb2_last_delta_relative_displ.qx(),
                                                                                 rb2_last_delta_relative_displ.qy(),
                                                                                 rb2_last_delta_relative_displ.qz());

        Eigen::Vector3d prism_joint_translation = this->_joint_states_all.at(current_idx) * prism_joint_translation_unitary;
        Eigen::Displacementd rb2_last_delta_relative_displ_prism_hyp = Eigen::Twistd(0.,
                                                                                     0.,
                                                                                     0.,
                                                                                     prism_joint_translation.x(),
                                                                                     prism_joint_translation.y(),
                                                                                     prism_joint_translation.z()).exp(1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation_prism_hyp = rb2_last_delta_relative_displ_prism_hyp.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation_prism_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_prism_hyp.qw(),
                                                                                           rb2_last_delta_relative_displ_prism_hyp.qx(),
                                                                                           rb2_last_delta_relative_displ_prism_hyp.qy(),
                                                                                           rb2_last_delta_relative_displ_prism_hyp.qz());        

        // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
        double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_prism_hyp).norm();
        // Actually both rotations should be zero because a prismatic joint constraints the rotation
        Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_prism_hyp;
        double rotation_error_angle = Eigen::Displacementd(0., 0., 0., rotation_error.w(),rotation_error.x(), rotation_error.y(),rotation_error.z()).log(1e-12).norm();


        accumulated_error += translation_error + fabs(rotation_error_angle);

        p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(translation_error/sigma_translation, 2)) *
                (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

        p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

        frame_counter++;
    }

    if(frame_counter != 0)
    {
        this->_measurements_likelihood = p_all_meas_given_model_params;
    }else{
        this->_measurements_likelihood = 1e-5;
    }
}

void PrismaticJointFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood;
}

std::vector<visualization_msgs::Marker> PrismaticJointFilter::getJointMarkersInRRBFrame() const
{
    // The class variable _prism_joint_orientation (also _uncertainty_o_phi and _uncertainty_o_theta) are defined in the frame of the
    // ref RB with the initial relative transformation to the second RB
    // We want the variables to be in the ref RB frame, without the initial relative transformation to the second RB

    // Careful: changed because now the parameters are relative to the srb
    Eigen::Matrix4d _srb_current_pose_in_rrbf_mat;
    //Twist2TransformMatrix(_srb_current_pose_in_rrbf, _srb_current_pose_in_rrbf_mat);
    _srb_current_pose_in_rrbf_mat = Eigen::Matrix4d::Identity();
    Eigen::Vector3d prism_joint_ori_in_ref_rb = _srb_current_pose_in_rrbf_mat.block<3,3>(0,0)*this->_joint_orientation;

    std::vector<visualization_msgs::Marker> prismatic_markers;
    // AXIS MARKER 1 -> The axis ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker axis_orientation_marker;
    axis_orientation_marker.ns = "kinematic_structure";
    axis_orientation_marker.action = visualization_msgs::Marker::ADD;
    axis_orientation_marker.type = visualization_msgs::Marker::ARROW;
    axis_orientation_marker.id = 3 * this->_joint_id;
    axis_orientation_marker.scale.x = JOINT_AXIS_AND_VARIABLE_MARKER_RADIUS;
    axis_orientation_marker.scale.y = 0.f;
    axis_orientation_marker.scale.z = 0.f;
    axis_orientation_marker.color.r = 0.f;
    axis_orientation_marker.color.g = 1.f;
    axis_orientation_marker.color.b = 0.f;
    axis_orientation_marker.color.a = 1.f;
    // Estimate position from supporting features:
    //    Eigen::Displacementd current_ref_pose_displ = this->_rrb_current_pose_in_sf.exp(1e-12);
    //    Eigen::Affine3d current_ref_pose;
    //    current_ref_pose.matrix() = current_ref_pose_displ.toHomogeneousMatrix();
    //    =current_ref_pose.inverse() * this->_srb_centroid_in_sf;
    Eigen::Vector3d second_centroid_relative_to_ref_body = this->getJointPositionInRRBFrame();
    Eigen::Vector3d position1 = second_centroid_relative_to_ref_body - this->_joint_state * prism_joint_ori_in_ref_rb;
    geometry_msgs::Point pt1;
    pt1.x = position1.x();
    pt1.y = position1.y();
    pt1.z = position1.z();
    axis_orientation_marker.points.push_back(pt1);
    Eigen::Vector3d position2 = second_centroid_relative_to_ref_body + this->_joint_state * prism_joint_ori_in_ref_rb;
    geometry_msgs::Point pt2;
    pt2.x = position2.x();
    pt2.y = position2.y();
    pt2.z = position2.z();
    axis_orientation_marker.points.push_back(pt2);

    prismatic_markers.push_back(axis_orientation_marker);

    // AXIS MARKER 2 -> Proportional to joint state ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    axis_orientation_marker.points.clear();
    axis_orientation_marker.id = 3 * this->_joint_id + 1;
    axis_orientation_marker.scale.x = JOINT_AXIS_MARKER_RADIUS;
    // Estimate position from supporting features:
    Eigen::Vector3d position12 = second_centroid_relative_to_ref_body - 100 * prism_joint_ori_in_ref_rb;
    geometry_msgs::Point pt12;
    pt12.x = position12.x();
    pt12.y = position12.y();
    pt12.z = position12.z();
    axis_orientation_marker.points.push_back(pt12);
    Eigen::Vector3d position22 = second_centroid_relative_to_ref_body + 100 * prism_joint_ori_in_ref_rb;
    geometry_msgs::Point pt22;
    pt22.x = position22.x();
    pt22.y = position22.y();
    pt22.z = position22.z();
    axis_orientation_marker.points.push_back(pt22);

    prismatic_markers.push_back(axis_orientation_marker);

    // AXIS MARKER 3 -> Text with the joint state ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    axis_orientation_marker.points.clear();
    axis_orientation_marker.id = 3 * this->_joint_id + 2;
    axis_orientation_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    axis_orientation_marker.scale.z = JOINT_VALUE_TEXT_SIZE;
    std::ostringstream oss_joint_value;
    oss_joint_value << std::fixed<< std::setprecision(1) << 100*this->_joint_state;
    axis_orientation_marker.text = oss_joint_value.str() + std::string(" cm");
    axis_orientation_marker.pose.position.x = second_centroid_relative_to_ref_body.x();
    axis_orientation_marker.pose.position.y = second_centroid_relative_to_ref_body.y();
    axis_orientation_marker.pose.position.z = second_centroid_relative_to_ref_body.z();
    axis_orientation_marker.pose.orientation.x = 0;
    axis_orientation_marker.pose.orientation.y = 0;
    axis_orientation_marker.pose.orientation.z = 0;
    axis_orientation_marker.pose.orientation.w = 1;

    prismatic_markers.push_back(axis_orientation_marker);

    // UNCERTAINTY MARKERS ///////////////////////////////////////////////////////////////////////////////////////////////////////

    // This first marker is just to match the number of markers of the revolute joint, but prismatic joint has no position
    visualization_msgs::Marker axis_position_uncertainty_marker;
    axis_position_uncertainty_marker.pose.position.x = second_centroid_relative_to_ref_body.x();
    axis_position_uncertainty_marker.pose.position.y = second_centroid_relative_to_ref_body.y();
    axis_position_uncertainty_marker.pose.position.z = second_centroid_relative_to_ref_body.z();
    axis_position_uncertainty_marker.ns = "kinematic_structure_uncertainty";
    axis_position_uncertainty_marker.type = visualization_msgs::Marker::SPHERE;
    axis_position_uncertainty_marker.action = visualization_msgs::Marker::DELETE;
    axis_position_uncertainty_marker.id = 3 * this->_joint_id ;

    prismatic_markers.push_back(axis_position_uncertainty_marker);

    visualization_msgs::Marker prism_axis_unc_cone1;
    prism_axis_unc_cone1.type = visualization_msgs::Marker::MESH_RESOURCE;
    prism_axis_unc_cone1.action = visualization_msgs::Marker::ADD;
    prism_axis_unc_cone1.mesh_resource = "package://joint_tracker/meshes/cone.stl";
    prism_axis_unc_cone1.pose.position.x = second_centroid_relative_to_ref_body.x();
    prism_axis_unc_cone1.pose.position.y = second_centroid_relative_to_ref_body.y();
    prism_axis_unc_cone1.pose.position.z = second_centroid_relative_to_ref_body.z();

    // NOTE:
    // Estimation of the uncertainty cones -----------------------------------------------
    // We estimate the orientation of the prismatic axis in spherical coordinates (r=1 always)
    // We estimate phi: angle from the x axis to the projection of the prismatic joint axis to the xy plane
    // We estimate theta: angle from the z axis to the prismatic joint axis
    // [TODO: phi and theta are in the reference rigid body. Do we need to transform it (adding uncertainty) to the reference frame?]
    // The covariance of phi and theta (a 2x2 matrix) gives us the uncertainty of the orientation of the joint
    // If we look from the joint axis, we would see an ellipse given by this covariance matrix [http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/]
    // But in RVIZ we can only set the scale of our cone mesh in x and y, not in a different axis
    // The first thing is then to estimate the direction of the major and minor axis of the ellipse and their size
    double r1,r2;
    Eigen::Matrix3d Rfull;

    findIntersectionOfEllipsoidAndPlane(_uncertainty_joint_orientation_xyz, _joint_orientation, r1, r2, Rfull);

    Eigen::Quaterniond ori_quat_final_ellipse(_srb_current_pose_in_rrbf_mat.block<3,3>(0,0)*Rfull);

    prism_axis_unc_cone1.pose.orientation.x = ori_quat_final_ellipse.x();
    prism_axis_unc_cone1.pose.orientation.y = ori_quat_final_ellipse.y();
    prism_axis_unc_cone1.pose.orientation.z = ori_quat_final_ellipse.z();
    prism_axis_unc_cone1.pose.orientation.w = ori_quat_final_ellipse.w();
    prism_axis_unc_cone1.ns = "kinematic_structure_uncertainty";
    prism_axis_unc_cone1.id = 3 * this->_joint_id + 1;
    prism_axis_unc_cone1.color.a = 0.4;
    prism_axis_unc_cone1.color.r = 0.0;
    prism_axis_unc_cone1.color.g = 1.0;
    prism_axis_unc_cone1.color.b = 0.0;

    // If the uncertainty is pi/6 (30 degrees) the scale in this direction should be 1
    // If the uncertainty is pi/12 (15 degrees) the scale in this direction should be 0.5
    // If the uncertainty is close to 0 the scale in this direction should be 0
    // If the uncertainty is close to pi the scale in this direction should be inf

    prism_axis_unc_cone1.scale.x = r1 / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.y = r2 / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.z = 1.;

    this->_uncertainty_joint_orientation_phitheta(0,0) = r1;
    this->_uncertainty_joint_orientation_phitheta(1,1) = r2;

    prismatic_markers.push_back(prism_axis_unc_cone1);

    // We repeat the process for the cone in the other direction

    Eigen::Matrix3d Rfull_i;
    Rfull_i << -Rfull.col(0)[0], Rfull.col(1)[0], -Rfull.col(2)[0],
            -Rfull.col(0)[1], Rfull.col(1)[1], -Rfull.col(2)[1],
            -Rfull.col(0)[2], Rfull.col(1)[2], -Rfull.col(2)[2];

    Eigen::Quaterniond ori_quat_neg_final_ellipse(_srb_current_pose_in_rrbf_mat.block<3,3>(0,0)*Rfull_i);

    prism_axis_unc_cone1.pose.orientation.x = ori_quat_neg_final_ellipse.x();
    prism_axis_unc_cone1.pose.orientation.y = ori_quat_neg_final_ellipse.y();
    prism_axis_unc_cone1.pose.orientation.z = ori_quat_neg_final_ellipse.z();
    prism_axis_unc_cone1.pose.orientation.w = ori_quat_neg_final_ellipse.w();
    prism_axis_unc_cone1.scale.x = r1 / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.y = r2 / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.z = 1.;
    prism_axis_unc_cone1.id =3 * this->_joint_id + 2;

    prismatic_markers.push_back(prism_axis_unc_cone1);

    return prismatic_markers;
}

JointFilterType PrismaticJointFilter::getJointFilterType() const
{
    return PRISMATIC_JOINT;
}

std::string PrismaticJointFilter::getJointFilterTypeStr() const
{
    return std::string("PrismaticJointFilter");
}
