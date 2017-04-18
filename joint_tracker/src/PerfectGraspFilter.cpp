#include "joint_tracker/PerfectGraspFilter.h"

#include "omip_common/OMIPUtils.h"

#include <Eigen/Geometry>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

// Dimensions of the system state of the filter that tracks a jacobian between 2 rigid bodies: 6x6 values of the jacobian matrix (36)
#define JACOBIAN_STATE_DIM 36
#define MEAS_DIM 6

/**
 * EKF internal state:
 *
 * x(1:36) =  Values of the Jacobian matrix
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

PerfectGraspFilter::PerfectGraspFilter() :
    JointFilter(),
    _sys_PDF(NULL),
    _sys_MODEL(NULL),
    _meas_PDF(NULL),
    _meas_MODEL(NULL),
    _pp_weight(1)
{
    // create SYSTEM MODEL -> The Jacobian does not change between time steps
    _A = BFL::Matrix(JACOBIAN_STATE_DIM, JACOBIAN_STATE_DIM);
    _A = 0.;
    for (unsigned int i = 1; i <= JACOBIAN_STATE_DIM; i++)
    {
        _A(i, i) = 1.0;
    }

    this->_measurements_likelihood = 1.0;
    this->_model_prior_probability = 1.0/3.9;
}

void PerfectGraspFilter::initialize()
{
    JointFilter::initialize();

    this->_initializeSystemModel();
    this->_initializeMeasurementModel();
    this->_initializeEKF();
}

void PerfectGraspFilter::_initializeSystemModel()
{
    ColumnVector sys_noise_MU(JACOBIAN_STATE_DIM);
    sys_noise_MU = 0;

    SymmetricMatrix sys_noise_COV(JACOBIAN_STATE_DIM);
    sys_noise_COV = 0.00000000001;

    // Initialize System Model
    Gaussian system_uncertainty_PDF(sys_noise_MU, sys_noise_COV);
    this->_sys_PDF = new LinearAnalyticConditionalGaussian( this->_A, system_uncertainty_PDF);
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( this->_sys_PDF);
}

void PerfectGraspFilter::_initializeMeasurementModel()
{
    // create MEASUREMENT MODEL
    ColumnVector meas_noise_MU(MEAS_DIM);
    meas_noise_MU = 0.0;
    SymmetricMatrix meas_noise_COV(MEAS_DIM);
    meas_noise_COV = 0.0;
    for (unsigned int i = 1; i <= MEAS_DIM; i++)
        meas_noise_COV(i, i) = 0.00000001;

    Gaussian meas_uncertainty_PDF(meas_noise_MU, meas_noise_COV);

    this->_meas_PDF = new NonLinearGraspMeasurementPdf(meas_uncertainty_PDF);
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty(this->_meas_PDF);
}

void PerfectGraspFilter::_initializeEKF()
{
    // THIS IS THE CRUCIAL PART
    // DEPENDING HOW WE INITIALIZE THE JACOBIAN WE ASSUME A TYPE OF GRASP OR ANOTHER
    ColumnVector prior_MU(JACOBIAN_STATE_DIM);
    prior_MU = 0.0;

    // We initialize with the identity matrix as Jacobian

    for(int i =0; i<6; i++)
        prior_MU(6*i+i+1) = 1;


    SymmetricMatrix prior_COV(JACOBIAN_STATE_DIM);
    prior_COV = 0.0;
    for (int i = 1; i <= JACOBIAN_STATE_DIM; i++)
    {
        prior_COV(i, i) = 1;
    }

    Gaussian prior_PDF(prior_MU, prior_COV);

    this->_ekf = new ExtendedKalmanFilter(&prior_PDF);
}

PerfectGraspFilter::~PerfectGraspFilter()
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

PerfectGraspFilter::PerfectGraspFilter(const PerfectGraspFilter &rev_joint) :
    JointFilter(rev_joint)
{
    this->_sys_PDF = new LinearAnalyticConditionalGaussian(*(rev_joint._sys_PDF));
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( *(rev_joint._sys_MODEL));
    this->_meas_PDF = new NonLinearGraspMeasurementPdf(*(rev_joint._meas_PDF));
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty( *(rev_joint._meas_MODEL));
    this->_ekf = new ExtendedKalmanFilter(*(rev_joint._ekf));
}

void PerfectGraspFilter::predictState(double time_interval_ns)
{
    //    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    //    SymmetricMatrix sys_noise_COV(JACOBIAN_STATE_DIM);
    //    sys_noise_COV = 0.0;
    //    for (int i = 1; i <= JACOBIAN_STATE_DIM; i++)
    //    {
    //        // Very small system noise because we provide the Jacobian matrix and we don't want to correct it
    //        sys_noise_COV(i, i) = 0.000001;//1.0*time_interval_ns/1e9;
    //    }

    //    this->_sys_PDF->MatrixSet(0, this->_A);
    //    this->_sys_PDF->AdditiveNoiseSigmaSet(sys_noise_COV);
    //    //The system update
    //    this->_ekf->Update(this->_sys_MODEL);

    //    _delta_eef_wrt_cf_ec = (time_interval_ns/1e9)*_rrb_current_vel_in_sf;
}

void PerfectGraspFilter::predictMeasurement()
{
    //This is between the grasping point and the interacted rigid body frame
    this->_change_in_relative_pose_predicted_in_rrbf = Eigen::Twistd(0., 0., 0., 0., 0., 0.);

    Eigen::Displacementd predicted_delta = this->_change_in_relative_pose_predicted_in_rrbf.exp(1e-20);

    Eigen::Displacementd T_rrbf_srbf_t_previous = this->_srb_previous_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t_previous;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);

    // The predicted pose has some uncertainty (make sure that it is positive definite!)
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            if(i == j)
                this->_change_in_relative_pose_cov_predicted_in_rrbf(i,j) = 0.0000000000005;
            else
                this->_change_in_relative_pose_cov_predicted_in_rrbf(i,j) = 0;
        }
    }

    // The predicted pose has some uncertainty (make sure that it is positive definite!)
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            if(i == j)
                this->_srb_predicted_pose_cov_in_rrbf(i,j) = 0.0000000000005;
            else
                this->_srb_predicted_pose_cov_in_rrbf(i,j) = 0;
        }
    }
}

void PerfectGraspFilter::correctState()
{
}

void PerfectGraspFilter::estimateMeasurementHistoryLikelihood()
{
    if(!_externally_set_likelihood)
    {
        double p_one_meas_given_model_params = 0;
        double p_all_meas_given_model_params = 0;

        double sigma_translation = 0.05;
        double sigma_rotation = 0.2;

        double accumulated_error = 0.;

        double frame_counter = 0.;

        // Estimate the number of samples used for likelihood estimation
        size_t trajectory_length = this->_changes_in_relative_pose_in_rrbf.size();
        size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);

        // Estimate the delta for the iterator over the number of samples
        // This makes that we take _likelihood_sample_num uniformly distributed over the whole trajectory, instead of the last _likelihood_sample_num
        double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);

        size_t current_idx = 0;

        // We test the last window_length motions
        for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
        {
            current_idx = boost::math::round(sample_idx*delta_idx_samples);
            Eigen::Displacementd rb2_last_delta_relative_displ = this->_changes_in_relative_pose_in_rrbf.at(current_idx).exp(1e-12);

            Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.getTranslation();
            Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.qw(),
                                                                                     rb2_last_delta_relative_displ.qx(),
                                                                                     rb2_last_delta_relative_displ.qy(),
                                                                                     rb2_last_delta_relative_displ.qz());

            Eigen::Displacementd rb2_last_delta_relative_displ_pg_hyp = Eigen::Twistd(0., 0., 0., 0., 0., 0.).exp(1e-12);

            Eigen::Vector3d rb2_last_delta_relative_translation_pg_hyp = rb2_last_delta_relative_displ_pg_hyp.getTranslation();

            Eigen::Quaterniond rb2_last_delta_relative_rotation_pg_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_pg_hyp.qw(),
                                                                                            rb2_last_delta_relative_displ_pg_hyp.qx(),
                                                                                            rb2_last_delta_relative_displ_pg_hyp.qy(),
                                                                                            rb2_last_delta_relative_displ_pg_hyp.qz());

            // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
            double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_pg_hyp).norm();

            Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_pg_hyp;
            double rotation_error_angle = Eigen::Displacementd(0., 0., 0., rotation_error.w(),rotation_error.x(), rotation_error.y(),rotation_error.z()).log(1e-12).norm();

            accumulated_error += translation_error + fabs(rotation_error_angle);

            p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))* exp((-1.0/2.0) * (pow(translation_error/sigma_translation, 2))) *
                    (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

            p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

            frame_counter++;
        }

        // If we receive ft sensor signals we use them to discriminate between grasp failure and any other model
        if(_ft_meas.size() == 6)
        {
            this->_estimateFTMeasurementLikelihood();
        }

        this->_measurements_likelihood = p_all_meas_given_model_params*(1-_current_prob_grasp_failure);
    }
}

void PerfectGraspFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood;
}

JointFilterType PerfectGraspFilter::getJointFilterType() const
{
    return PERFECT_GRASP_JOINT;
}

std::string PerfectGraspFilter::getJointFilterTypeStr() const
{
    return std::string("PerfectGraspFilter");
}

std::vector<visualization_msgs::Marker> PerfectGraspFilter::getJointMarkersInRRBFrame() const
{
    std::vector<visualization_msgs::Marker> pg_markers;

    // TRANSMITTED TRANSLATION MARKER ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker transmitted_translation_marker;
    transmitted_translation_marker.ns = "kinematic_structure";
    transmitted_translation_marker.action = visualization_msgs::Marker::ADD;
    transmitted_translation_marker.type = visualization_msgs::Marker::CUBE_LIST;
    transmitted_translation_marker.id = 3 * this->_joint_id;
    transmitted_translation_marker.scale.x = 0.04f;
    transmitted_translation_marker.scale.y = 0.04f;
    transmitted_translation_marker.scale.z = 0.04f;
    transmitted_translation_marker.color.r = 0.f;
    transmitted_translation_marker.color.g = 0.f;
    transmitted_translation_marker.color.b = 1.f;
    transmitted_translation_marker.color.a = 0.5f;
    geometry_msgs::Point pt1;
    pt1.x = 0.05;
    pt1.y = 0;
    pt1.z = 0;
    transmitted_translation_marker.points.push_back(pt1);
    pt1.x = 0;
    pt1.y = 0.05;
    pt1.z = 0;
    transmitted_translation_marker.points.push_back(pt1);
    pt1.x = 0;
    pt1.y = 0;
    pt1.z = 0.05;
    transmitted_translation_marker.points.push_back(pt1);

    pg_markers.push_back(transmitted_translation_marker);

    // TRANSMITTED ROTATION MARKER ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker transmitted_rotation_marker;
    transmitted_rotation_marker.ns = "kinematic_structure";
    transmitted_rotation_marker.action = visualization_msgs::Marker::ADD;
    transmitted_rotation_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    transmitted_rotation_marker.id = 3 * this->_joint_id + 1;
    transmitted_rotation_marker.scale.x = 0.04f;
    transmitted_rotation_marker.color.r = 0.f;
    transmitted_rotation_marker.color.g = 0.f;
    transmitted_rotation_marker.color.b = 1.f;
    transmitted_rotation_marker.color.a = 0.5f;

    pt1.x = 0.1;
    pt1.y = 0;
    pt1.z = 0;
    transmitted_rotation_marker.points.push_back(pt1);
    pt1.x = 0;
    pt1.y = 0.1;
    pt1.z = 0;
    transmitted_rotation_marker.points.push_back(pt1);
    pt1.x = 0;
    pt1.y = 0;
    pt1.z = 0.1;
    transmitted_rotation_marker.points.push_back(pt1);

    pg_markers.push_back(transmitted_rotation_marker);

    visualization_msgs::Marker empty_marker;
    empty_marker.pose.position.x = 0.;
    empty_marker.pose.position.y = 0.;
    empty_marker.pose.position.z = 0.;
    empty_marker.header.frame_id = "camera_rgb_optical_frame";
    empty_marker.type = visualization_msgs::Marker::SPHERE;
    empty_marker.action = visualization_msgs::Marker::DELETE;
    empty_marker.scale.x = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.scale.y = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.scale.z = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.color.a = 0.3;
    empty_marker.color.r = 0.0;
    empty_marker.color.g = 0.0;
    empty_marker.color.b = 1.0;
    empty_marker.ns = "kinematic_structure";
    empty_marker.id = 3 * this->_joint_id + 2;
    pg_markers.push_back(empty_marker);

    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;

    pg_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 1;

    pg_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 2;

    pg_markers.push_back(empty_marker);

    // To generate plots for IROS17 without the grasp markers
    std::vector<visualization_msgs::Marker> empty_markers;
    return empty_markers;
    //return pg_markers;
}
