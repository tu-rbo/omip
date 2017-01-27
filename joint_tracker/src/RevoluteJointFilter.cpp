#include "joint_tracker/RevoluteJointFilter.h"

#include "omip_common/OMIPUtils.h"

#include <Eigen/Geometry>

#include <boost/math/distributions/chi_squared.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/Float64MultiArray.h"

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

// Dimensions of the system state of the filter that tracks a revolute joint: orientation (2 values), position (3 values), joint variable, and joint velocity
#define REV_STATE_DIM 7
#define MEAS_DIM 6

/**
 * EKF internal state:
 *
 * x(1) =  RevJointOrientation_phi
 * x(2) =  RevJointOrientation_theta
 * RevJointOrientation is represented in spherical coords
 * x(3) = RevJointPosition_x
 * x(4) = RevJointPosition_y
 * x(5) = RevJointPosition_z
 * x(6) = RevJointVariable
 * x(7) = RevJointVariable_d
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

using namespace omip;

RevoluteJointFilter::RevoluteJointFilter() :
    JointFilter(),
    _sys_PDF(NULL),
    _sys_MODEL(NULL),
    _meas_PDF(NULL),
    _meas_MODEL(NULL),
    _ekf(NULL),
    _sigma_delta_meas_uncertainty_angular(-1),
    _accumulated_rotation(0.0)
{
}

void RevoluteJointFilter::setCovarianceDeltaMeasurementAngular(double sigma_delta_meas_uncertainty_angular)
{
    this->_sigma_delta_meas_uncertainty_angular = sigma_delta_meas_uncertainty_angular;
}

void RevoluteJointFilter::initialize()
{
    JointFilter::initialize();
    this->_joint_orientation = Eigen::Vector3d( this->_current_delta_pose_in_rrbf.rx(),
                                                    this->_current_delta_pose_in_rrbf.ry(),
                                                    this->_current_delta_pose_in_rrbf.rz());
    this->_joint_state = this->_joint_orientation.norm();
    //this->_joint_velocity = this->_joint_state/(this->_loop_period_ns/1e9);
    // Setting it to 0 is better.
    // The best approximation would be to (this->_joint_state/num_steps_to_joint_state)/(this->_loop_period_ns/1e9)
    // but we don't know how many steps passed since we estimated the first time the joint variable
    this->_joint_velocity = 0.0;

    this->_joint_states_all.push_back(this->_joint_state);

    Eigen::Vector3d linear_part = Eigen::Vector3d( this->_current_delta_pose_in_rrbf.vx(),
                                                   this->_current_delta_pose_in_rrbf.vy(),
                                                   this->_current_delta_pose_in_rrbf.vz());
// Alternative
//    Eigen::Matrix4d ht;
//    Twist2TransformMatrix(_current_delta_pose_in_rrbf, ht);

//    this->_joint_position = ht.block<3,1>(0,3)
//            - ht.block<3,3>(0,0)*ht.block<3,1>(0,3);

    this->_joint_position = (1.0 / (pow(this->_joint_state, 2))) * this->_joint_orientation.cross(linear_part);
    this->_joint_orientation.normalize();

    this->_initializeSystemModel();
    this->_initializeMeasurementModel();
    this->_initializeEKF();
}

void RevoluteJointFilter::setMinRotationRevolute(const double& value)
{
    _rev_min_rot_for_ee = value;
}

void RevoluteJointFilter::setMaxRadiusDistanceRevolute(const double& value)
{
    _rev_max_joint_distance_for_ee = value;
}

void RevoluteJointFilter::_initializeSystemModel()
{
    // create SYSTEM MODEL
    Matrix A(REV_STATE_DIM, REV_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= REV_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(6, 7) = this->_loop_period_ns/1e9; //Adding the velocity to the position of the joint variable

    ColumnVector sys_noise_MU(REV_STATE_DIM);
    sys_noise_MU = 0;

    SymmetricMatrix sys_noise_COV(REV_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // PHI
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // THETA
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_px* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // Px
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_py* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // Py
    sys_noise_COV(5, 5) = this->_sigma_sys_noise_pz* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // Pz
    sys_noise_COV(6, 6) = this->_sigma_sys_noise_jv* (std::pow((this->_loop_period_ns/1e9),3) / 3.0); // pv
    sys_noise_COV(7, 7) = this->_sigma_sys_noise_jvd*(this->_loop_period_ns/1e9); // d(pv)/dt

    // Initialize System Model
    Gaussian system_uncertainty_PDF(sys_noise_MU, sys_noise_COV);
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( A, system_uncertainty_PDF);
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( this->_sys_PDF);
}

void RevoluteJointFilter::_initializeMeasurementModel()
{
    // create MEASUREMENT MODEL
    ColumnVector meas_noise_MU(MEAS_DIM);
    meas_noise_MU = 0.0;
    SymmetricMatrix meas_noise_COV(MEAS_DIM);
    meas_noise_COV = 0.0;
    for (unsigned int i = 1; i <= MEAS_DIM; i++)
        meas_noise_COV(i, i) = this->_sigma_meas_noise;

    Gaussian meas_uncertainty_PDF(meas_noise_MU, meas_noise_COV);

    this->_meas_PDF = new NonLinearRevoluteMeasurementPdf(meas_uncertainty_PDF);
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty(this->_meas_PDF);
}

void RevoluteJointFilter::_initializeEKF()
{
    ColumnVector prior_MU(REV_STATE_DIM);
    prior_MU = 0.0;

    SymmetricMatrix prior_COV(REV_STATE_DIM);
    prior_COV = 0.0;

    // This is weird but happens when working with synthetic data
    // We don't want to divide by 0!
    if(this->_joint_orientation.x() == 0.0)
    {
        this->_joint_orientation.x() = 1e-6;
    }

    prior_MU(1) = atan2( this->_joint_orientation.y() , this->_joint_orientation.x());
    prior_MU(2) = acos(this->_joint_orientation.z());
    prior_MU(3) = this->_joint_position.x();
    prior_MU(4) = this->_joint_position.y();
    prior_MU(5) = this->_joint_position.z();
    prior_MU(6) = this->_joint_state;
    prior_MU(7) = this->_joint_velocity;


    for (int i = 1; i <= REV_STATE_DIM; i++)
    {
        prior_COV(i, i) = _prior_cov_vel;
    }

    Gaussian prior_PDF(prior_MU, prior_COV);

    this->_ekf = new ExtendedKalmanFilter(&prior_PDF);
}

RevoluteJointFilter::~RevoluteJointFilter()
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

RevoluteJointFilter::RevoluteJointFilter(const RevoluteJointFilter &rev_joint) :
    JointFilter(rev_joint)
{
    this->_sigma_delta_meas_uncertainty_angular = rev_joint._sigma_delta_meas_uncertainty_angular;
    this->_sys_PDF = new LinearAnalyticConditionalGaussian(*(rev_joint._sys_PDF));
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( *(rev_joint._sys_MODEL));
    this->_meas_PDF = new NonLinearRevoluteMeasurementPdf(*(rev_joint._meas_PDF));
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty( *(rev_joint._meas_MODEL));
    this->_ekf = new ExtendedKalmanFilter(*(rev_joint._ekf));
}

void RevoluteJointFilter::predictState(double time_interval_ns)
{
    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    SymmetricMatrix sys_noise_COV(REV_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((time_interval_ns/1e9),3) / 3.0); // PHI
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((time_interval_ns/1e9),3) / 3.0); // THETA
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_px* (std::pow((time_interval_ns/1e9),3) / 3.0); // Px
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_py* (std::pow((time_interval_ns/1e9),3) / 3.0); // Py
    sys_noise_COV(5, 5) = this->_sigma_sys_noise_pz* (std::pow((time_interval_ns/1e9),3) / 3.0); // Pz
    sys_noise_COV(6, 6) = this->_sigma_sys_noise_jv* (std::pow((time_interval_ns/1e9),3) / 3.0); // pv
    sys_noise_COV(7, 7) = this->_sigma_sys_noise_jvd*(time_interval_ns/1e9); // d(pv)/dt

    // Estimate the new updating matrix which also depends on the time elapsed between the previous and the current measurement
    // x(t+1) = x(t) + v(t) * delta_t
    Matrix A(REV_STATE_DIM, REV_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= REV_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(6, 7) = time_interval_ns/1e9;; //Adding the velocity (times the time) to the position of the joint variable

    this->_sys_PDF->MatrixSet(0, A);
    this->_sys_PDF->AdditiveNoiseSigmaSet(sys_noise_COV);
    //The system update
    this->_ekf->Update(this->_sys_MODEL);
}

void RevoluteJointFilter::predictMeasurement()
{
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    ColumnVector predicted_delta_pose_in_rrbf = this->_meas_MODEL->PredictionGet(empty, state_updated_state);

    this->_predicted_delta_pose_in_rrbf = Eigen::Twistd( predicted_delta_pose_in_rrbf(4), predicted_delta_pose_in_rrbf(5),
                                                         predicted_delta_pose_in_rrbf(6), predicted_delta_pose_in_rrbf(1),
                                                         predicted_delta_pose_in_rrbf(2), predicted_delta_pose_in_rrbf(3));

    Eigen::Displacementd predicted_delta = this->_predicted_delta_pose_in_rrbf.exp(1e-20);
    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);

    bool change = false;
    this->_srb_predicted_pose_in_rrbf = unwrapTwist(this->_srb_predicted_pose_in_rrbf, T_rrbf_srbf_t_next, this->_srb_previous_predicted_pose_in_rrbf, change);

    this->_srb_previous_predicted_pose_in_rrbf = this->_srb_predicted_pose_in_rrbf;
}

void RevoluteJointFilter::correctState()
{
    ColumnVector updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    ColumnVector rb2_measured_delta_relative_pose_cv(MEAS_DIM);
    rb2_measured_delta_relative_pose_cv = 0.;
    rb2_measured_delta_relative_pose_cv(1) = this->_current_delta_pose_in_rrbf.vx();
    rb2_measured_delta_relative_pose_cv(2) = this->_current_delta_pose_in_rrbf.vy();
    rb2_measured_delta_relative_pose_cv(3) = this->_current_delta_pose_in_rrbf.vz();
    rb2_measured_delta_relative_pose_cv(4) = this->_current_delta_pose_in_rrbf.rx();
    rb2_measured_delta_relative_pose_cv(5) = this->_current_delta_pose_in_rrbf.ry();
    rb2_measured_delta_relative_pose_cv(6) = this->_current_delta_pose_in_rrbf.rz();

    // Update the uncertainty on the measurement
    // The uncertainty on the measurement (the delta motion of the second rigid body wrt the reference rigid body) will be large if the measurement
    // is small and small if the measurement is large
    // Also at 2PI the uncertainty should be high (rotational periodicity of the exponential map)
    Eigen::Vector3d angular_component(this->_current_delta_pose_in_rrbf.rx(), this->_current_delta_pose_in_rrbf.ry(), this->_current_delta_pose_in_rrbf.rz());
    double meas_uncertainty_factor = 1 / (1.0 - exp(-sin(angular_component.norm()/2.0)/this->_sigma_delta_meas_uncertainty_angular));

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

    this->_meas_PDF->AdditiveNoiseSigmaSet(current_delta_pose_cov_in_rrbf * meas_uncertainty_factor );

    this->_ekf->Update(this->_meas_MODEL, rb2_measured_delta_relative_pose_cv);

    updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    this->_joint_orientation(0) = sin(updated_state(2)) * cos(updated_state(1));
    this->_joint_orientation(1) = sin(updated_state(2)) * sin(updated_state(1));
    this->_joint_orientation(2) = cos(updated_state(2));

    SymmetricMatrix updated_uncertainty = this->_ekf->PostGet()->CovarianceGet();

    for(int i=0; i<3; i++)
    {
        this->_joint_position(i) = updated_state(i+3);
        for(int j=0; j<3; j++)
        {
            this->_uncertainty_joint_position(i,j) = updated_uncertainty(i+3, j+3);
        }
    }

    double joint_state_before = _joint_state;

    // This jump should happen if we are close to 2PI or -2PI rotation
    if(_from_inverted_to_non_inverted)
    {
        _accumulated_rotation = std::round(joint_state_before/(2*M_PI))*2*M_PI;
    }

    // This jump should happen if we are close to PI or -PI rotation
    if(_from_non_inverted_to_inverted)
    {
        _accumulated_rotation = std::round(joint_state_before/(M_PI))*2*M_PI;
    }

    this->_joint_state = updated_state(6);

    // If we are inverting the twist is because we are in the interval (PI, 2PI) or (-PI, -2PI)
    if(_inverted_delta_srb_pose_in_rrbf)
    {
        _joint_state = _accumulated_rotation - _joint_state;
    }else{
        _joint_state = _accumulated_rotation + _joint_state;
    }


    this->_uncertainty_joint_state = updated_uncertainty(6,6);
    this->_joint_velocity = updated_state(7);
    this->_uncertainty_joint_velocity = updated_uncertainty(7,7);

    this->_joint_orientation_phi = updated_state(1);
    this->_joint_orientation_theta = updated_state(2);
    this->_uncertainty_joint_orientation_phitheta(0,0) = updated_uncertainty(1, 1);
    this->_uncertainty_joint_orientation_phitheta(0,1) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,0) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,1) = updated_uncertainty(2, 2);
}

void RevoluteJointFilter::estimateMeasurementHistoryLikelihood()
{
    double accumulated_error = 0.;

    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    ColumnVector updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    double phi = updated_state(1);
    double theta = updated_state(2);
    double sp = sin(phi);
    double cp = cos(phi);
    double st = sin(theta);
    double ct = cos(theta);

    double px = updated_state(3);
    double py = updated_state(4);
    double pz = updated_state(5);

    this->_joint_states_all.push_back(updated_state(6));

    Eigen::Vector3d rev_joint_rotation_unitary = Eigen::Vector3d(cp * st, sp * st, ct);
    rev_joint_rotation_unitary.normalize();

    // Convert the screw attributes (line description) to a twist
    Eigen::Vector3d rev_joint_position = Eigen::Vector3d(px, py, pz);
    Eigen::Vector3d rev_joint_translation_unitary = (-rev_joint_rotation_unitary).cross(rev_joint_position);

    double counter = 0.;
    size_t trajectory_length = this->_delta_poses_in_rrbf.size();
    size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);
    double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);
    size_t current_idx = 0;

    double max_norm_of_deltas = 0;

    // Estimation of the quality of the parameters of the revolute joint
    // If the joint is revolute and the parameters are accurate, the joint axis orientation and position should not change over time
    // That means that the current orientation/position, multiplied by the amount of revolute displacement at each time step, should provide the delta in the relative
    // pose between ref and second rb at each time step
    // We test amount_samples of the relative trajectory
    // I check if the joint is too young or if there is too few rotation and the axis is very far away (a prismatic joint can be seen as a revolute joint where the
    // joint is far away)
    // We need to have memory here: if the rotation was once larger than the this->_rev_min_rot_for_ee could be that we returned to the starting relative pose

    for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
    {
        current_idx = boost::math::round(sample_idx*delta_idx_samples);
        Eigen::Displacementd rb2_last_delta_relative_displ = this->_delta_poses_in_rrbf.at(current_idx).exp(1e-12);

        max_norm_of_deltas = std::max(this->_delta_poses_in_rrbf.at(current_idx).norm(), max_norm_of_deltas);

        Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.qw(),
                                                                                 rb2_last_delta_relative_displ.qx(),
                                                                                 rb2_last_delta_relative_displ.qy(),
                                                                                 rb2_last_delta_relative_displ.qz());

        Eigen::Vector3d rev_joint_translation = this->_joint_states_all.at(current_idx) * rev_joint_translation_unitary;
        Eigen::Vector3d rev_joint_rotation = this->_joint_states_all.at(current_idx) * rev_joint_rotation_unitary;
        Eigen::Displacementd rb2_last_delta_relative_displ_rev_hyp = Eigen::Twistd( rev_joint_rotation.x(),
                                                                                    rev_joint_rotation.y(),
                                                                                    rev_joint_rotation.z(),
                                                                                    rev_joint_translation.x(),
                                                                                    rev_joint_translation.y(),
                                                                                    rev_joint_translation.z()).exp(1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation_rev_hyp = rb2_last_delta_relative_displ_rev_hyp.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation_rev_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_rev_hyp.qw(),
                                                                                         rb2_last_delta_relative_displ_rev_hyp.qx(),
                                                                                         rb2_last_delta_relative_displ_rev_hyp.qy(),
                                                                                         rb2_last_delta_relative_displ_rev_hyp.qz());

        // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
        double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_rev_hyp).norm();
        Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_rev_hyp;
        double rotation_error_angle = Eigen::Displacementd(0., 0., 0., rotation_error.w(), rotation_error.x(), rotation_error.y(), rotation_error.z()).log(1e-12).norm();

        accumulated_error += translation_error + fabs(rotation_error_angle);

        p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(translation_error/sigma_translation, 2)) *
                (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

        p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

        counter++;
    }

    if(counter != 0)
    {
        this->_measurements_likelihood = p_all_meas_given_model_params;
    }else{
        this->_measurements_likelihood = 1e-5;
    }
}

void RevoluteJointFilter::estimateUnnormalizedModelProbability()
{
    Eigen::Vector3d point1_in_rrbf = this->_joint_position + 100*this->_joint_orientation;
    Eigen::Vector3d point2_in_rrbf = this->_joint_position - 100*this->_joint_orientation;

    // The joint position and orientation are in rrb frame.

    double p_params_given_model = 1;

    // We only measure the distance to the reference rb if it is not the static environment!
    double distance_to_rrb = 0;
    if(_rrb_id != 0)
    {
        distance_to_rrb = (point2_in_rrbf- point1_in_rrbf).cross(point1_in_rrbf).norm()/((point2_in_rrbf - point1_in_rrbf).norm());
        p_params_given_model *= (1.0/(_rev_max_joint_distance_for_ee*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(distance_to_rrb/_rev_max_joint_distance_for_ee, 2));
    }

    Eigen::Vector4d point1_in_rrbf_homo(point1_in_rrbf.x(), point1_in_rrbf.y(), point1_in_rrbf.z(), 1.0);
    Eigen::Vector4d point1_in_sf_homo = _rrb_current_pose_in_sf.exp(1e-12).toHomogeneousMatrix()*point1_in_rrbf_homo;
    Eigen::Vector3d point1_in_sf(point1_in_sf_homo[0],point1_in_sf_homo[1],point1_in_sf_homo[2]);

    Eigen::Vector4d point2_in_rrbf_homo(point2_in_rrbf.x(), point2_in_rrbf.y(), point2_in_rrbf.z(), 1.0);
    Eigen::Vector4d point2_in_sf_homo = _rrb_current_pose_in_sf.exp(1e-12).toHomogeneousMatrix()*point2_in_rrbf_homo;
    Eigen::Vector3d point2_in_sf(point2_in_sf_homo[0],point2_in_sf_homo[1],point2_in_sf_homo[2]);

    double distance_to_srb = ((point2_in_sf - point1_in_sf).cross(point1_in_sf - _srb_centroid_in_sf)).norm()/((point2_in_sf - point1_in_sf).norm());

    p_params_given_model *= (1.0/(_rev_max_joint_distance_for_ee*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(distance_to_srb/_rev_max_joint_distance_for_ee, 2));

    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood*p_params_given_model;
}

geometry_msgs::TwistWithCovariance RevoluteJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    Eigen::Matrix<double, 6, 6> adjoint;
    computeAdjoint(this->_rrb_current_pose_in_sf, adjoint);
    Eigen::Twistd predicted_delta_pose_in_sf = adjoint*this->_predicted_delta_pose_in_rrbf;

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = predicted_delta_pose_in_sf.vx();
    hypothesis.twist.linear.y = predicted_delta_pose_in_sf.vy();
    hypothesis.twist.linear.z = predicted_delta_pose_in_sf.vz();
    hypothesis.twist.angular.x = predicted_delta_pose_in_sf.rx();
    hypothesis.twist.angular.y = predicted_delta_pose_in_sf.ry();
    hypothesis.twist.angular.z = predicted_delta_pose_in_sf.rz();

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             hypothesis.covariance[6 * i + j] = measurement_cov(i+1,j+1);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance RevoluteJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    Eigen::Matrix<double, 6, 6> adjoint;
    computeAdjoint(this->_rrb_current_pose_in_sf, adjoint);
    Eigen::Twistd predicted_delta_pose_in_sf = adjoint*(this->_predicted_delta_pose_in_rrbf/(_loop_period_ns/1e9));

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = predicted_delta_pose_in_sf.vx();
    hypothesis.twist.linear.y = predicted_delta_pose_in_sf.vy();
    hypothesis.twist.linear.z = predicted_delta_pose_in_sf.vz();
    hypothesis.twist.angular.x = predicted_delta_pose_in_sf.rx();
    hypothesis.twist.angular.y = predicted_delta_pose_in_sf.ry();
    hypothesis.twist.angular.z = predicted_delta_pose_in_sf.rz();

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             hypothesis.covariance[6 * i + j] = measurement_cov(i+1,j+1);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance RevoluteJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    Eigen::Twistd rrb_next_pose_in_sf = (delta_rrb_in_sf.exp(1e-12)*this->_rrb_current_pose_in_sf.exp(1e-12)).log(1e-12);

    Eigen::Displacementd T_sf_rrbf_next = rrb_next_pose_in_sf.exp(1e-12);
    Eigen::Displacementd T_rrbf_srbf_next = this->_srb_predicted_pose_in_rrbf.exp(1e-12);

    Eigen::Displacementd T_sf_srbf_next = T_rrbf_srbf_next*T_sf_rrbf_next;

    Eigen::Twistd srb_next_pose_in_sf = T_sf_srbf_next.log(1e-12);

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = srb_next_pose_in_sf.vx();
    hypothesis.twist.linear.y = srb_next_pose_in_sf.vy();
    hypothesis.twist.linear.z = srb_next_pose_in_sf.vz();
    hypothesis.twist.angular.x = srb_next_pose_in_sf.rx();
    hypothesis.twist.angular.y = srb_next_pose_in_sf.ry();
    hypothesis.twist.angular.z = srb_next_pose_in_sf.rz();

    // This call gives me the covariance of the predicted measurement: the relative pose between RBs
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    SymmetricMatrix measurement_cov = this->_meas_MODEL->CovarianceGet(empty, state_updated_state);
    Eigen::Matrix<double,6,6> measurement_cov_eigen;
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            measurement_cov_eigen(i,j) = measurement_cov(i+1,j+1);
        }
    }
    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> tranformed_cov;
    adjointXcovXadjointT(_rrb_current_pose_in_sf, measurement_cov_eigen, tranformed_cov);
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf + tranformed_cov;
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = new_pose_covariance(i, j);
        }
    }


#ifdef PUBLISH_PREDICTED_POSE_AS_PWC
    // This is used to visualize the predictions based on the joint hypothesis
    geometry_msgs::PoseWithCovarianceStamped pose_with_cov_stamped;
    pose_with_cov_stamped.header.stamp = ros::Time::now();
    pose_with_cov_stamped.header.frame_id = "camera_rgb_optical_frame";

    Eigen::Displacementd displ_from_twist = srb_next_pose_in_sf.exp(1e-12);
    pose_with_cov_stamped.pose.pose.position.x = displ_from_twist.x();
    pose_with_cov_stamped.pose.pose.position.y = displ_from_twist.y();
    pose_with_cov_stamped.pose.pose.position.z = displ_from_twist.z();
    pose_with_cov_stamped.pose.pose.orientation.x = displ_from_twist.qx();
    pose_with_cov_stamped.pose.pose.orientation.y = displ_from_twist.qy();
    pose_with_cov_stamped.pose.pose.orientation.z = displ_from_twist.qz();
    pose_with_cov_stamped.pose.pose.orientation.w = displ_from_twist.qw();

    for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
            pose_with_cov_stamped.pose.covariance[6 * i + j] = new_pose_covariance(i, j);

    _predicted_next_pose_publisher.publish(pose_with_cov_stamped);
#endif

    return hypothesis;
}

std::vector<visualization_msgs::Marker> RevoluteJointFilter::getJointMarkersInRRBFrame() const
{
    // The class variable _joint_orientation and _rev_joint_posi (also _uncertainty_o_phi and _uncertainty_o_theta) are defined in the frame of the
    // ref RB with the initial relative transformation to the second RB
    // We want the variables to be in the ref RB frame, without the initial relative transformation to the second RB
    Eigen::Vector3d rev_joint_ori_in_ref_rb = this->_joint_orientation;
    Eigen::Vector3d rev_joint_posi_in_ref_rb = this->_joint_position;


    std::vector<visualization_msgs::Marker> revolute_markers;
    // AXIS MARKER 1 -> The axis ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker axis_orientation_marker;
    axis_orientation_marker.ns = "kinematic_structure";
    axis_orientation_marker.action = visualization_msgs::Marker::ADD;
    axis_orientation_marker.type = visualization_msgs::Marker::ARROW;

    // Define the joint parameters relative to the REFERENCE RIGID BODY and the marker will be also
    // defined wrt to the frame of the REFERENCE RIGID BODY!
    // The frame name will be assigned in the MultiJointTrackerNode because we don't know the rb ids here
    //axis_orientation_marker.header.frame_id = "camera_rgb_optical_frame";
    axis_orientation_marker.id = 3 * this->_joint_id;
    axis_orientation_marker.scale.x = JOINT_AXIS_AND_VARIABLE_MARKER_RADIUS;
    axis_orientation_marker.scale.y = 0.f;
    axis_orientation_marker.scale.z = 0.f;
    axis_orientation_marker.color.r = 1.f;
    axis_orientation_marker.color.g = 0.f;
    axis_orientation_marker.color.b = 0.f;
    axis_orientation_marker.color.a = 1.f;
    Eigen::Vector3d point_on_rot_axis_1 = rev_joint_posi_in_ref_rb - this->_joint_state * rev_joint_ori_in_ref_rb;
    geometry_msgs::Point pt1;
    pt1.x = point_on_rot_axis_1.x();
    pt1.y = point_on_rot_axis_1.y();
    pt1.z = point_on_rot_axis_1.z();
    axis_orientation_marker.points.push_back(pt1);
    Eigen::Vector3d point_on_rot_axis_2 = rev_joint_posi_in_ref_rb + this->_joint_state * rev_joint_ori_in_ref_rb;
    geometry_msgs::Point pt2;
    pt2.x = point_on_rot_axis_2.x();
    pt2.y = point_on_rot_axis_2.y();
    pt2.z = point_on_rot_axis_2.z();
    axis_orientation_marker.points.push_back(pt2);
    revolute_markers.push_back(axis_orientation_marker);

    // AXIS MARKER 2 -> Proportional to the joint state///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker axis_orientation_markerb;
    axis_orientation_markerb.ns = "kinematic_structure";
    axis_orientation_markerb.action = visualization_msgs::Marker::ADD;
    axis_orientation_markerb.type = visualization_msgs::Marker::ARROW;
    // Define the joint parameters relative to the REFERENCE RIGID BODY and the marker will be also
    // defined wrt to the frame of the REFERENCE RIGID BODY!
    // The frame name will be assigned in the MultiJointTrackerNode because we don't know the rb ids here
    //axis_orientation_markerb.header.frame_id = "camera_rgb_optical_frame";
    axis_orientation_markerb.id = 3 * this->_joint_id + 1;
    axis_orientation_markerb.scale.x = JOINT_AXIS_MARKER_RADIUS;
    axis_orientation_markerb.scale.y = 0.f;
    axis_orientation_markerb.scale.z = 0.f;
    axis_orientation_markerb.color.r = 1.f;
    axis_orientation_markerb.color.g = 0.f;
    axis_orientation_markerb.color.b = 0.f;
    axis_orientation_markerb.color.a = 1.f;
    Eigen::Vector3d point_on_rot_axis_1b = rev_joint_posi_in_ref_rb - 100 * rev_joint_ori_in_ref_rb;
    geometry_msgs::Point pt1b;
    pt1b.x = point_on_rot_axis_1b.x();
    pt1b.y = point_on_rot_axis_1b.y();
    pt1b.z = point_on_rot_axis_1b.z();
    axis_orientation_markerb.points.push_back(pt1b);
    Eigen::Vector3d point_on_rot_axis_2b = rev_joint_posi_in_ref_rb  + 100 * rev_joint_ori_in_ref_rb;
    geometry_msgs::Point pt2b;
    pt2b.x = point_on_rot_axis_2b.x();
    pt2b.y = point_on_rot_axis_2b.y();
    pt2b.z = point_on_rot_axis_2b.z();
    axis_orientation_markerb.points.push_back(pt2b);

    revolute_markers.push_back(axis_orientation_markerb);

    // AXIS MARKER 3 -> Text with the joint state ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    axis_orientation_markerb.points.clear();
    axis_orientation_markerb.id = 3 * this->_joint_id + 2;
    axis_orientation_markerb.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    axis_orientation_markerb.scale.z = JOINT_VALUE_TEXT_SIZE;
    std::ostringstream oss_joint_value;
    oss_joint_value << std::fixed<< std::setprecision(0) << (180/M_PI)*this->_joint_state;
    axis_orientation_markerb.text = oss_joint_value.str() + std::string(" deg");
    axis_orientation_markerb.pose.position.x = rev_joint_posi_in_ref_rb.x();
    axis_orientation_markerb.pose.position.y = rev_joint_posi_in_ref_rb.y();
    axis_orientation_markerb.pose.position.z = rev_joint_posi_in_ref_rb.z();
    axis_orientation_markerb.pose.orientation.x = 0;
    axis_orientation_markerb.pose.orientation.y = 0;
    axis_orientation_markerb.pose.orientation.z = 0;
    axis_orientation_markerb.pose.orientation.w = 1;

    revolute_markers.push_back(axis_orientation_markerb);

    // UNCERTAINTY MARKERS ///////////////////////////////////////////////////////////////////////////////////////////////////////

    visualization_msgs::Marker axis_position_uncertainty_marker;
    axis_position_uncertainty_marker.pose.position.x = rev_joint_posi_in_ref_rb.x();
    axis_position_uncertainty_marker.pose.position.y = rev_joint_posi_in_ref_rb.y();
    axis_position_uncertainty_marker.pose.position.z = rev_joint_posi_in_ref_rb.z();
    // Define the joint parameters relative to the REFERENCE RIGID BODY and the marker will be also
    // defined wrt to the frame of the REFERENCE RIGID BODY!
    // The frame name will be assigned in the MultiJointTrackerNode because we don't know the rb ids here
    //axis_position_uncertainty_marker.header.frame_id = "camera_rgb_optical_frame";
    axis_position_uncertainty_marker.ns = "kinematic_structure_uncertainty";
    axis_position_uncertainty_marker.type = visualization_msgs::Marker::SPHERE;
    axis_position_uncertainty_marker.action = visualization_msgs::Marker::ADD;
    axis_position_uncertainty_marker.id = 3 * this->_joint_id ;
    // Reliability = 1 --> position_sphere_uncertainty = 0
    // Reliability = 0 --> position_sphere_uncertainty = 1m
    axis_position_uncertainty_marker.scale.x = this->_uncertainty_joint_position(0,0); //Using start and end points, scale.x is the radius of the array body
    axis_position_uncertainty_marker.scale.y = this->_uncertainty_joint_position(1,1); //Using start and end points, scale.y is the radius of the array head
    axis_position_uncertainty_marker.scale.z = this->_uncertainty_joint_position(2,2); //Using start and end points, scale.y is the radius of the array head
    axis_position_uncertainty_marker.color.a = 0.3;
    axis_position_uncertainty_marker.color.r = 1.0;
    axis_position_uncertainty_marker.color.g = 0.0;
    axis_position_uncertainty_marker.color.b = 0.0;
    revolute_markers.push_back(axis_position_uncertainty_marker);

    visualization_msgs::Marker rev_axis_unc_cone1;
    rev_axis_unc_cone1.type = visualization_msgs::Marker::MESH_RESOURCE;
    rev_axis_unc_cone1.action = visualization_msgs::Marker::ADD;
    rev_axis_unc_cone1.mesh_resource = "package://joint_tracker/meshes/cone.stl";
    rev_axis_unc_cone1.pose.position.x = rev_joint_posi_in_ref_rb.x();
    rev_axis_unc_cone1.pose.position.y = rev_joint_posi_in_ref_rb.y();
    rev_axis_unc_cone1.pose.position.z = rev_joint_posi_in_ref_rb.z();


    // NOTE:
    // Estimation of the uncertainty cones -----------------------------------------------
    // We estimate the orientation of the revolute axis in spherical coordinates (r=1 always)
    // We estimate phi: angle from the x axis to the projection of the revolute joint axis to the xy plane
    // We estimate theta: angle from the z axis to the revolute joint axis
    // [TODO: phi and theta are in the reference rigid body. Do we need to transform it (adding uncertainty) to the reference frame?]
    // The covariance of phi and theta (a 2x2 matrix) gives us the uncertainty of the orientation of the joint
    // If we look from the joint axis, we would see an ellipse given by this covariance matrix [http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/]
    // But in RVIZ we can only set the scale of our cone mesh in x and y, not in a different axis
    // The first thing is then to estimate the direction of the major and minor axis of the ellipse and their size
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(this->_uncertainty_joint_orientation_phitheta);

    // The sizes of the major and minor axes of the ellipse are given by the eigenvalues and the chi square distribution P(x<critical_value) = confidence_value
    // For 50% of confidence on the cones shown
    double confidence_value = 0.5;
    boost::math::chi_squared chi_sq_dist(2);
    double critical_value = boost::math::quantile(chi_sq_dist, confidence_value);
    double major_axis_length = 2*eigensolver.eigenvalues()[1]*std::sqrt(critical_value);
    double minor_axis_length = 2*eigensolver.eigenvalues()[0]*std::sqrt(critical_value);

    // If z is pointing in the direction of the joint, the angle between the x axis and the largest axis of the ellipse is arctg(v1_y/v1_x) where v1 is the eigenvector of
    // largest eigenvalue (the last column in the matrix returned by eigenvectors() in eigen library):
    double alpha = atan2(eigensolver.eigenvectors().col(1)[1],eigensolver.eigenvectors().col(1)[0]);
    // We create a rotation around the z axis to align the ellipse to have the major axis aligned to the x-axis (we UNDO the rotation of the ellipse):
    Eigen::AngleAxisd init_rot(-alpha, Eigen::Vector3d::UnitZ());

    // Now I need to rotate the mesh so that:
    // 1) The z axis of the mesh points in the direction of the joint
    // 2) The x axis of the mesh is contained in the x-y plane of the reference frame

    // To get the z axis of the mesh to point in the direction of the joint
    Eigen::Quaterniond ori_quat;
    ori_quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), rev_joint_ori_in_ref_rb);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori = rev_joint_ori_in_ref_rb.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_pos = rev_joint_ori_in_ref_rb.cross(coplanar_xy_orthogonal_to_joint_ori);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_pos;
    rotation_pos << coplanar_xy_orthogonal_to_joint_ori.x(),y_pos.x(),rev_joint_ori_in_ref_rb.x(),
            coplanar_xy_orthogonal_to_joint_ori.y(),y_pos.y(),rev_joint_ori_in_ref_rb.y(),
            coplanar_xy_orthogonal_to_joint_ori.z(),y_pos.z(),rev_joint_ori_in_ref_rb.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_final(rotation_pos);

    Eigen::Quaterniond ori_quat_final_ellipse(ori_quat_final.toRotationMatrix()*init_rot.toRotationMatrix());

    rev_axis_unc_cone1.pose.orientation.x = ori_quat_final_ellipse.x();
    rev_axis_unc_cone1.pose.orientation.y = ori_quat_final_ellipse.y();
    rev_axis_unc_cone1.pose.orientation.z = ori_quat_final_ellipse.z();
    rev_axis_unc_cone1.pose.orientation.w = ori_quat_final_ellipse.w();
    // Define the joint parameters relative to the REFERENCE RIGID BODY and the marker will be also
    // defined wrt to the frame of the REFERENCE RIGID BODY!
    // The frame name will be assigned in the MultiJointTrackerNode because we don't know the rb ids here
    //rev_axis_unc_cone1.header.frame_id = "camera_rgb_optical_frame";
    rev_axis_unc_cone1.ns = "kinematic_structure_uncertainty";
    rev_axis_unc_cone1.id = 3 * this->_joint_id + 1;
    rev_axis_unc_cone1.color.a = 0.4;
    rev_axis_unc_cone1.color.r = 1.0;
    rev_axis_unc_cone1.color.g = 0.0;
    rev_axis_unc_cone1.color.b = 0.0;

    // If the uncertainty is pi/6 (30 degrees) the scale in this direction should be 1
    // If the uncertainty is pi/12 (15 degrees) the scale in this direction should be 0.5
    // If the uncertainty is close to 0 the scale in this direction should be 0
    // If the uncertainty is close to pi the scale in this direction should be inf

    rev_axis_unc_cone1.scale.x = major_axis_length / (M_PI / 6.0);
    rev_axis_unc_cone1.scale.y = minor_axis_length / (M_PI / 6.0);
    rev_axis_unc_cone1.scale.z = 1.;
    revolute_markers.push_back(rev_axis_unc_cone1);

    // We repeat the process for the cone in the other direction
    // To get the z axis of the mesh to point in the direction of the joint (negative)
    Eigen::Vector3d rev_joint_ori_in_ref_rb_neg = -rev_joint_ori_in_ref_rb;
    Eigen::Quaterniond ori_quat_neg;
    ori_quat_neg.setFromTwoVectors(Eigen::Vector3d::UnitZ(), rev_joint_ori_in_ref_rb_neg);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori_neg = rev_joint_ori_in_ref_rb_neg.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori_neg.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_neg = rev_joint_ori_in_ref_rb_neg.cross(coplanar_xy_orthogonal_to_joint_ori_neg);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_neg;
    rotation_neg << coplanar_xy_orthogonal_to_joint_ori_neg.x(),y_neg.x(),rev_joint_ori_in_ref_rb_neg.x(),
            coplanar_xy_orthogonal_to_joint_ori_neg.y(),y_neg.y(),rev_joint_ori_in_ref_rb_neg.y(),
            coplanar_xy_orthogonal_to_joint_ori_neg.z(),y_neg.z(),rev_joint_ori_in_ref_rb_neg.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_neg_final(rotation_neg);

    // We undo the rotation of the ellipse (but negative!):
    Eigen::AngleAxisd init_rot_neg(alpha, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond ori_quat_neg_final_ellipse(ori_quat_neg_final.toRotationMatrix()*init_rot_neg.toRotationMatrix());

    rev_axis_unc_cone1.pose.orientation.x = ori_quat_neg_final_ellipse.x();
    rev_axis_unc_cone1.pose.orientation.y = ori_quat_neg_final_ellipse.y();
    rev_axis_unc_cone1.pose.orientation.z = ori_quat_neg_final_ellipse.z();
    rev_axis_unc_cone1.pose.orientation.w = ori_quat_neg_final_ellipse.w();
    rev_axis_unc_cone1.scale.x = major_axis_length / (M_PI / 6.0);
    rev_axis_unc_cone1.scale.y = minor_axis_length / (M_PI / 6.0);
    rev_axis_unc_cone1.scale.z = 1.;
    rev_axis_unc_cone1.id = 3 * this->_joint_id + 2;
    revolute_markers.push_back(rev_axis_unc_cone1);

    return revolute_markers;
}

JointFilterType RevoluteJointFilter::getJointFilterType() const
{
    return REVOLUTE_JOINT;
}

std::string RevoluteJointFilter::getJointFilterTypeStr() const
{
    return std::string("RevoluteJointFilter");
}
