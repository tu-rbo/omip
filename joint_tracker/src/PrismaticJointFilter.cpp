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
#define PRISM_STATE_DIM 4
#define MEAS_DIM 6

/**
 * EKF internal state:
 *
 * x(1) =  PrismJointOrientation_phi
 * x(2) =  PrismJointOrientation_theta
 * PrismJointOrientation is represented in spherical coords
 * NOTE: I try to keep theta between 0 and pi and phi between 0 and two pi
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

void PrismaticJointFilter::initialize()
{
    JointFilter::initialize();

//    std::cout << "this->_srb_current_pose_in_sff" << std::endl;
//    std::cout << this->_srb_current_pose_in_sf << std::endl;
//    std::cout << this->_srb_current_pose_in_sf.exp(1e-9).toHomogeneousMatrix() << std::endl;
//    std::cout << "this->_srb_initial_pose_in_rrbf" << std::endl;
//    std::cout << this->_srb_initial_pose_in_rrbf << std::endl;
//    std::cout << this->_srb_initial_pose_in_rrbf.exp(1e-9).toHomogeneousMatrix() << std::endl;
//    std::cout << "this->_current_delta_pose_in_rrbf" << std::endl;
//    std::cout << this->_current_delta_pose_in_rrbf << std::endl;
//    std::cout << this->_current_delta_pose_in_rrbf.exp(1e-9).toHomogeneousMatrix() << std::endl;

    this->_joint_orientation = Eigen::Vector3d(
                this->_current_delta_pose_in_rrbf.vx(),
                this->_current_delta_pose_in_rrbf.vy(),
                this->_current_delta_pose_in_rrbf.vz());
    this->_joint_state = this->_joint_orientation.norm();

    this->_joint_states_all.push_back(this->_joint_state);
    //this->_joint_velocity = this->_joint_state/(this->_loop_period_ns/1e9);
    // Setting it to 0 is better.
    // The best approximation would be to (this->_rev_variable/num_steps_to_rev_variable)/(this->_loop_period_ns/1e9)
    // but we don't know how many steps passed since we estimated the first time the joint variable
    this->_joint_velocity = 0.0;
    this->_joint_orientation.normalize();

    // The position of the prismatic joint is the centroid of the features that belong to the second rigid body (in RBF)
    Eigen::Displacementd current_ref_pose_displ = this->_rrb_current_pose_in_sf.exp(1e-12);
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
    A(3, 4) = this->_loop_period_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    ColumnVector sys_noise_MU(PRISM_STATE_DIM);
    sys_noise_MU = 0;

    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_jv* (std::pow((this->_loop_period_ns/1e9),3) / 3.0);
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jvd*(this->_loop_period_ns/1e9);

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


    // create MEASUREMENT MODEL Force Torque sensor
    ColumnVector meas_noise_ft_MU(6);
    meas_noise_ft_MU = 0;
    SymmetricMatrix meas_noise_ft_COV(6);
    meas_noise_ft_COV = 0;
    for (unsigned int i=1; i<=6; i++)
        meas_noise_ft_COV(i,i) = 1;
    Gaussian meas_uncertainty_ft_PDF(meas_noise_ft_MU, meas_noise_ft_COV);

    //    Matrix Himu(3,6);  Himu = 0;
    //    Himu(1,4) = 1;    Himu(2,5) = 1;    Himu(3,6) = 1;
    //    imu_meas_pdf_   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    //    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);
}

void PrismaticJointFilter::_initializeEKF()
{
    ColumnVector prior_MU(PRISM_STATE_DIM);
    prior_MU = 0.0;

    SymmetricMatrix prior_COV(PRISM_STATE_DIM);
    prior_COV = 0.0;

    prior_MU(1) = atan2(this->_joint_orientation.y() , this->_joint_orientation.x());

    // NOTE: I make phi between 0 and 2pi
    //    if(prior_MU(1) < 0.0)
    //    {
    //        prior_MU(1) += 2*M_PI;
    //    }

    prior_MU(2) = acos(this->_joint_orientation.z());
    prior_MU(3) = this->_joint_state;
    prior_MU(4) = this->_joint_velocity;

    ROS_INFO_STREAM_NAMED( "PrismaticJointFilter::_initializeEKF",
                           "Prismatic initial state (phi, theta, jv, jv_dot): " << prior_MU(1) << " " << prior_MU(2) << " " << prior_MU(3) << " " << prior_MU(4));

    for (int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        prior_COV(i, i) = this->_prior_cov_vel;
    }

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

void PrismaticJointFilter::predictState(double time_interval_ns)
{
    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    SymmetricMatrix sys_noise_COV(PRISM_STATE_DIM);
    sys_noise_COV = 0.0;
    sys_noise_COV(1, 1) = this->_sigma_sys_noise_phi* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(2, 2) = this->_sigma_sys_noise_theta* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(3, 3) = this->_sigma_sys_noise_jv* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(4, 4) = this->_sigma_sys_noise_jvd*(time_interval_ns/1e9);

    // Estimate the new updating matrix which also depends on the time elapsed between the previous and the current measurement
    // x(t+1) = x(t) + v(t) * delta_t
    Matrix A(PRISM_STATE_DIM, PRISM_STATE_DIM);
    A = 0.;
    for (unsigned int i = 1; i <= PRISM_STATE_DIM; i++)
    {
        A(i, i) = 1.0;
    }
    A(3, 4) = time_interval_ns/1e9; //Adding the velocity (times the time) to the position of the joint variable

    this->_sys_PDF->MatrixSet(0, A);
    this->_sys_PDF->AdditiveNoiseSigmaSet(sys_noise_COV);
    this->_ekf->Update(this->_sys_MODEL);
}

geometry_msgs::TwistWithCovariance PrismaticJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    Eigen::Twistd predicted_delta_pose_in_sf = this->_rrb_current_pose_in_sf.exp(1e-12).adjoint()*this->_predicted_delta_pose_in_rrbf;

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

geometry_msgs::TwistWithCovariance PrismaticJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    Eigen::Twistd predicted_delta_pose_in_sf = this->_rrb_current_pose_in_sf.exp(1e-12).adjoint()*(this->_predicted_delta_pose_in_rrbf/(_loop_period_ns/1e9));

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

void PrismaticJointFilter::predictMeasurement()
{
    ColumnVector empty;
    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();

    ROS_DEBUG_STREAM_NAMED( "PrismaticJointFilter::UpdateJointParameters",
                            "Prismatic state after state update: " << state_updated_state(1) << " " << state_updated_state(2) << " " << state_updated_state(3) << " " << state_updated_state(4));

    ColumnVector predicted_delta_pose_in_rrbf = this->_meas_MODEL->PredictionGet(empty, state_updated_state);

    this->_predicted_delta_pose_in_rrbf = Eigen::Twistd( predicted_delta_pose_in_rrbf(4), predicted_delta_pose_in_rrbf(5),
                                                         predicted_delta_pose_in_rrbf(6), predicted_delta_pose_in_rrbf(1),
                                                         predicted_delta_pose_in_rrbf(2), predicted_delta_pose_in_rrbf(3));

    Eigen::Displacementd predicted_delta = this->_predicted_delta_pose_in_rrbf.exp(1e-20);
    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);
}

void PrismaticJointFilter::correctState()
{
    // New 26.8.2016 -> There is small rotations in the reference frame that cause the prismatic joint to rotate
    // We eliminate this: we search for the closest motion without rotation that resembles the relative motion
    Eigen::Matrix4d  _srb_current_pose_in_rrbf_matrix;
    Twist2TransformMatrix( _srb_current_pose_in_rrbf, _srb_current_pose_in_rrbf_matrix);

    //Then set the rotation to the identity
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            if(i==j)
            {
                _srb_current_pose_in_rrbf_matrix(i,j) = 1;
            }else{
                _srb_current_pose_in_rrbf_matrix(i,j) = 0;
            }
        }
    }

    Eigen::Twistd  _srb_current_pose_in_rrbf_no_rot;
    TransformMatrix2Twist(_srb_current_pose_in_rrbf_matrix, _srb_current_pose_in_rrbf_no_rot);

    Eigen::Twistd _current_delta_pose_in_rrbf_no_rot = (_srb_current_pose_in_rrbf_no_rot.exp(1e-12)*_srb_initial_pose_in_rrbf.exp(1e-12).inverse()).log(1e-12);

    ColumnVector rb2_measured_delta_relative_pose_cv(MEAS_DIM);
    rb2_measured_delta_relative_pose_cv = 0.;
    rb2_measured_delta_relative_pose_cv(1) = _current_delta_pose_in_rrbf_no_rot.vx();
    rb2_measured_delta_relative_pose_cv(2) = _current_delta_pose_in_rrbf_no_rot.vy();
    rb2_measured_delta_relative_pose_cv(3) = _current_delta_pose_in_rrbf_no_rot.vz();
    rb2_measured_delta_relative_pose_cv(4) = _current_delta_pose_in_rrbf_no_rot.rx();
    rb2_measured_delta_relative_pose_cv(5) = _current_delta_pose_in_rrbf_no_rot.ry();
    rb2_measured_delta_relative_pose_cv(6) = _current_delta_pose_in_rrbf_no_rot.rz();

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
    this->_joint_orientation(0) = sin(updated_state(2)) * cos(updated_state(1));
    this->_joint_orientation(1) = sin(updated_state(2)) * sin(updated_state(1));
    this->_joint_orientation(2) = cos(updated_state(2));

    SymmetricMatrix updated_uncertainty = this->_ekf->PostGet()->CovarianceGet();

    this->_joint_state = updated_state(3);
    this->_uncertainty_joint_state = updated_uncertainty(3,3);
    this->_joint_velocity = updated_state(4);
    this->_uncertainty_joint_velocity = updated_uncertainty(4,4);

    this->_joint_orientation_phi = updated_state(1);
    this->_joint_orientation_theta = updated_state(2);
    this->_uncertainty_joint_orientation_phitheta(0,0) = updated_uncertainty(1, 1);
    this->_uncertainty_joint_orientation_phitheta(0,1) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,0) = updated_uncertainty(1, 2);
    this->_uncertainty_joint_orientation_phitheta(1,1) = updated_uncertainty(2, 2);
}

void PrismaticJointFilter::estimateMeasurementHistoryLikelihood()
{
    double accumulated_error = 0.;

    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    ColumnVector state_updated_state = this->_ekf->PostGet()->ExpectedValueGet();
    double phi = state_updated_state(1);
    double theta = state_updated_state(2);

    this->_joint_states_all.push_back(state_updated_state(3));

    // This vector is in the ref RB with the initial relative transformation to the second RB
    Eigen::Vector3d prism_joint_translation_unitary = Eigen::Vector3d(cos(phi) * sin(theta), sin(phi) * sin(theta), cos(theta));
    prism_joint_translation_unitary.normalize();

    double frame_counter = 0.;
    size_t trajectory_length = this->_delta_poses_in_rrbf.size();
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
        Eigen::Displacementd rb2_last_delta_relative_displ = this->_delta_poses_in_rrbf.at(current_idx).exp(1e-12);

        max_norm_of_deltas = std::max(this->_delta_poses_in_rrbf.at(current_idx).norm(), max_norm_of_deltas);

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

geometry_msgs::TwistWithCovariance PrismaticJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    Eigen::Twistd rrb_next_pose_in_sf = (delta_rrb_in_sf.exp(1e-12)*this->_rrb_current_pose_in_sf.exp(1e-12)).log(1e-12);

    //Eigen::Twistd rrb_next_pose_in_sf = this->_rrb_current_pose_in_sf + this->_rrb_current_vel_in_sf;
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
    Eigen::Matrix<double,6,6> adjoint_eigen = this->_rrb_current_pose_in_sf.exp(1e-12).adjoint();
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf + adjoint_eigen*measurement_cov_eigen*adjoint_eigen.transpose();
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

std::vector<visualization_msgs::Marker> PrismaticJointFilter::getJointMarkersInRRBFrame() const
{
    // The class variable _prism_joint_orientation (also _uncertainty_o_phi and _uncertainty_o_theta) are defined in the frame of the
    // ref RB with the initial relative transformation to the second RB
    // We want the variables to be in the ref RB frame, without the initial relative transformation to the second RB
    Eigen::Vector3d prism_joint_ori_in_ref_rb = this->_joint_orientation;

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
    ori_quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), prism_joint_ori_in_ref_rb);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori = prism_joint_ori_in_ref_rb.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_pos = prism_joint_ori_in_ref_rb.cross(coplanar_xy_orthogonal_to_joint_ori);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_pos;
    rotation_pos << coplanar_xy_orthogonal_to_joint_ori.x(),y_pos.x(),prism_joint_ori_in_ref_rb.x(),
            coplanar_xy_orthogonal_to_joint_ori.y(),y_pos.y(),prism_joint_ori_in_ref_rb.y(),
            coplanar_xy_orthogonal_to_joint_ori.z(),y_pos.z(),prism_joint_ori_in_ref_rb.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_final(rotation_pos);

    Eigen::Quaterniond ori_quat_final_ellipse(ori_quat_final.toRotationMatrix()*init_rot.toRotationMatrix());

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

    prism_axis_unc_cone1.scale.x = major_axis_length / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.y = minor_axis_length / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.z = 1.;

    prismatic_markers.push_back(prism_axis_unc_cone1);

    // We repeat the process for the cone in the other direction
    // To get the z axis of the mesh to point in the direction of the joint (negative)
    Eigen::Vector3d prism_joint_ori_in_ref_rb_neg = -prism_joint_ori_in_ref_rb;
    Eigen::Quaterniond ori_quat_neg;
    ori_quat_neg.setFromTwoVectors(Eigen::Vector3d::UnitZ(), prism_joint_ori_in_ref_rb_neg);

    // To get the x axis of the mesh to be contained in the x-y plane of the reference frame
    // First, find a vector that is orthogonal to the joint orientation and also to the z_axis (this latter implies to be contained in the xy plane)
    Eigen::Vector3d coplanar_xy_orthogonal_to_joint_ori_neg = prism_joint_ori_in_ref_rb_neg.cross(Eigen::Vector3d::UnitZ());
    // Normalize it -> Gives me the desired x axis after the rotation
    coplanar_xy_orthogonal_to_joint_ori_neg.normalize();

    // Then find the corresponding y axis after the rotation as the cross product of the z axis after rotation (orientation of the joint)
    // and the x axis after rotation
    Eigen::Vector3d y_neg = prism_joint_ori_in_ref_rb_neg.cross(coplanar_xy_orthogonal_to_joint_ori_neg);

    // Create a matrix with the values of the vectors after rotation
    Eigen::Matrix3d rotation_neg;
    rotation_neg << coplanar_xy_orthogonal_to_joint_ori_neg.x(),y_neg.x(),prism_joint_ori_in_ref_rb_neg.x(),
            coplanar_xy_orthogonal_to_joint_ori_neg.y(),y_neg.y(),prism_joint_ori_in_ref_rb_neg.y(),
            coplanar_xy_orthogonal_to_joint_ori_neg.z(),y_neg.z(),prism_joint_ori_in_ref_rb_neg.z();

    // Create a quaternion with the matrix
    Eigen::Quaterniond ori_quat_neg_final(rotation_neg);

    // We undo the rotation of the ellipse (but negative!):
    Eigen::AngleAxisd init_rot_neg(alpha, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond ori_quat_neg_final_ellipse(ori_quat_neg_final.toRotationMatrix()*init_rot_neg.toRotationMatrix());

    prism_axis_unc_cone1.pose.orientation.x = ori_quat_neg_final_ellipse.x();
    prism_axis_unc_cone1.pose.orientation.y = ori_quat_neg_final_ellipse.y();
    prism_axis_unc_cone1.pose.orientation.z = ori_quat_neg_final_ellipse.z();
    prism_axis_unc_cone1.pose.orientation.w = ori_quat_neg_final_ellipse.w();
    prism_axis_unc_cone1.scale.x = major_axis_length / (M_PI / 6.0);
    prism_axis_unc_cone1.scale.y = minor_axis_length / (M_PI / 6.0);
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
