#include "joint_tracker/DeformationFilter.h"

#include "omip_common/OMIPUtils.h"

#include <Eigen/Geometry>

#include <boost/math/distributions/chi_squared.hpp>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "tf_conversions/tf_eigen.h"

#include "std_msgs/Float64MultiArray.h"

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

DeformationFilter::DeformationFilter() :
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
    this->_model_prior_probability = 1.0/4.0;

    _motion_memory_prior = 1.0;

    _predicted_next_relpose_publisher = _predicted_next_relpose_nh.advertise<std_msgs::Float64MultiArray>("/ee2cp/predicted_state",10);
}

void DeformationFilter::initialize()
{
    JointFilter::initialize();

    this->_initializeSystemModel();
    this->_initializeMeasurementModel();
    this->_initializeEKF();

    _ee_2_cp_pose_ht = Eigen::Matrix4d::Identity();
    _ee_2_cp_pose_cov = 0.001*Eigen::Matrix<double, 6, 6>::Identity();
    _ee_2_cp_previous_pose_ht = Eigen::Matrix4d::Identity();
    _ee_2_cp_previous_pose_cov = 0.001*Eigen::Matrix<double, 6, 6>::Identity();
    _ee_2_cp_predicted_pose_ht = Eigen::Matrix4d::Identity();
    _ee_2_cp_predicted_pose_cov = 0.001*Eigen::Matrix<double, 6, 6>::Identity();

    _ee_2_cp_system_noise = Eigen::Matrix<double, 6, 6>::Identity();

    //Noise for the translation
    for(int i=0; i<3;i++)
    {
        _ee_2_cp_system_noise(i,i) *= 0.01;
    }

    //Noise for the rotation
    for(int i=3; i<6;i++)
    {
        _ee_2_cp_system_noise *= 0.1;
    }

    _ee_2_cp_measured_pose_ht = Eigen::Matrix4d::Identity();
    _ee_2_cp_measured_pose_cov = 0.01*Eigen::Matrix<double, 6, 6>::Identity();
}

void DeformationFilter::_initializeSystemModel()
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

void DeformationFilter::_initializeMeasurementModel()
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

void DeformationFilter::_initializeEKF()
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

DeformationFilter::~DeformationFilter()
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

DeformationFilter::DeformationFilter(const DeformationFilter &rev_joint) :
    JointFilter(rev_joint)
{
    this->_sys_PDF = new LinearAnalyticConditionalGaussian(*(rev_joint._sys_PDF));
    this->_sys_MODEL = new LinearAnalyticSystemModelGaussianUncertainty( *(rev_joint._sys_MODEL));
    this->_meas_PDF = new NonLinearGraspMeasurementPdf(*(rev_joint._meas_PDF));
    this->_meas_MODEL = new AnalyticMeasurementModelGaussianUncertainty( *(rev_joint._meas_MODEL));
    this->_ekf = new ExtendedKalmanFilter(*(rev_joint._ekf));
}

void DeformationFilter::predictState(double time_interval_ns)
{
    // Forward model is the identity -> Predicted relative pose is the previous relative pose (+ some uncertainty)
    _ee_2_cp_predicted_pose_ht = _ee_2_cp_pose_ht;
    _ee_2_cp_predicted_pose_cov = _ee_2_cp_pose_cov + _ee_2_cp_system_noise;

    // Convert into the right format for the inverse measurement model
    // The relative pose is given as (tx, ty, tz), (rx, ry, rz) euler angles
    tf::Matrix3x3 rotation;
    Eigen::Matrix3d rotation_eigen = _ee_2_cp_predicted_pose_ht.block<3,3>(0,0);
    tf::matrixEigenToTF(rotation_eigen, rotation);

    double roll = 0, pitch = 0, yaw = 0;
    rotation.getRPY(roll, pitch, yaw);

    std_msgs::Float64MultiArray msg;
    msg.data.push_back(_ee_2_cp_predicted_pose_ht(0,3));
    msg.data.push_back(_ee_2_cp_predicted_pose_ht(1,3));
    msg.data.push_back(_ee_2_cp_predicted_pose_ht(2,3));
    msg.data.push_back(roll);
    msg.data.push_back(pitch);
    msg.data.push_back(yaw);

    _predicted_next_relpose_publisher.publish(msg);
}

void DeformationFilter::setMeasurementEE2CP(const std::vector<double>& ee2cp_rel_pose_meas, const double& measurement_timestamp_ns)
{
    if(ee2cp_rel_pose_meas.size() > 0)
    {
        // The relative pose is given as (tx, ty, tz), (rx, ry, rz) euler angles
        tf::Matrix3x3 rotation;
        rotation.setRPY(ee2cp_rel_pose_meas[3], ee2cp_rel_pose_meas[4], ee2cp_rel_pose_meas[5]);
        Eigen::Matrix3d rotation_eigen;
        tf::matrixTFToEigen(rotation, rotation_eigen);

        _ee_2_cp_measured_pose_ht = Eigen::Matrix4d::Identity();

        _ee_2_cp_measured_pose_ht.block<3,3>(0,0) = rotation_eigen;
        _ee_2_cp_measured_pose_ht(0,3) = ee2cp_rel_pose_meas[0];
        _ee_2_cp_measured_pose_ht(1,3) = ee2cp_rel_pose_meas[1];
        _ee_2_cp_measured_pose_ht(2,3) = ee2cp_rel_pose_meas[2];

        //Covariance of the predicted pose from the measurement model
        for(int i=0; i<6; i++)
        {
            _ee_2_cp_measured_pose_cov(i,i) = ee2cp_rel_pose_meas[i+6];
        }
    }else{
        _ee_2_cp_measured_pose_ht = Eigen::Matrix4d::Identity();
    }
}

void DeformationFilter::predictMeasurement()
{
    _srb_previous_pose_in_rrbf = _srb_current_pose_in_rrbf;
    TransformMatrix2Twist(_ee_2_cp_predicted_pose_ht.inverse(), _srb_current_pose_in_rrbf);
    TransformMatrix2Twist(_ee_2_cp_predicted_pose_ht.inverse(), _srb_predicted_pose_in_rrbf);

    //This is between the grasping point and the interacted rigid body frame
    this->_change_in_relative_pose_predicted_in_rrbf = _srb_current_pose_in_rrbf;

    this->_change_in_relative_pose_cov_predicted_in_rrbf = _ee_2_cp_pose_cov + _ee_2_cp_system_noise;

    this->_srb_predicted_pose_cov_in_rrbf = _ee_2_cp_pose_cov + _ee_2_cp_system_noise;
}

void DeformationFilter::correctState()
{
    //Integrate measurement: merge it with the predicted state
    // Using: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1238637 "Computing MAP trajectories by representing, propagating and combining PDFs over groups"
    // Equations 9 and 13
    Eigen::Matrix4d innovation_ht = _ee_2_cp_measured_pose_ht*_ee_2_cp_predicted_pose_ht.inverse();
    Eigen::Twistd innovation_ec;
    TransformMatrix2Twist(innovation_ht, innovation_ec);

    Eigen::Matrix<double, 6, 1> innovation_vec;
    innovation_vec << innovation_ec.vx(), innovation_ec.vy(), innovation_ec.vz(),
            innovation_ec.rx(), innovation_ec.ry(), innovation_ec.rz();

    Eigen::Matrix<double, 6, 1> pose_update_vec = _ee_2_cp_predicted_pose_cov*(_ee_2_cp_predicted_pose_cov + _ee_2_cp_measured_pose_cov).inverse()*innovation_vec;
    Eigen::Twistd pose_update_ec(pose_update_vec[3], pose_update_vec[4], pose_update_vec[5], pose_update_vec[0], pose_update_vec[1], pose_update_vec[2]);
    Eigen::Matrix4d pose_update_ht;
    Twist2TransformMatrix(pose_update_ec, pose_update_ht);
    Eigen::Matrix4d updated_pose = pose_update_ht*_ee_2_cp_predicted_pose_ht;

    Eigen::Matrix<double, 6, 6> updated_pose_cov = _ee_2_cp_predicted_pose_cov*
            (_ee_2_cp_predicted_pose_cov + _ee_2_cp_measured_pose_cov).inverse()*
            _ee_2_cp_measured_pose_cov;

    _ee_2_cp_pose_ht = updated_pose;
    _ee_2_cp_pose_cov = updated_pose_cov;

    Eigen::Matrix4d delta_pose = _ee_2_cp_pose_ht*_ee_2_cp_previous_pose_ht;

    Eigen::Twistd delta_pose_ec;
    TransformMatrix2Twist(delta_pose, delta_pose_ec);

    //_changes_in_relative_pose_in_rrbf = delta_pose_ec;
}

void DeformationFilter::estimateMeasurementHistoryLikelihood()
{
    this->_measurements_likelihood = 1;
}

void DeformationFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood;
}

JointFilterType DeformationFilter::getJointFilterType() const
{
    return CONTACT_POINT_JOINT;
}

std::string DeformationFilter::getJointFilterTypeStr() const
{
    return std::string("DeformationFilter");
}

std::vector<visualization_msgs::Marker> DeformationFilter::getJointMarkersInRRBFrame() const
{
    std::vector<visualization_msgs::Marker> pg_markers;

    // TRANSMITTED TRANSLATION MARKER ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker transmitted_translation_marker;
    transmitted_translation_marker.ns = "kinematic_structure";
    transmitted_translation_marker.action = visualization_msgs::Marker::ADD;
    transmitted_translation_marker.type = visualization_msgs::Marker::LINE_LIST;
    transmitted_translation_marker.id = 3 * this->_joint_id;
    transmitted_translation_marker.scale.x = 0.04f;
    transmitted_translation_marker.scale.y = 0.04f;
    transmitted_translation_marker.scale.z = 0.04f;
    transmitted_translation_marker.color.r = 0.f;
    transmitted_translation_marker.color.g = 0.f;
    transmitted_translation_marker.color.b = 1.f;
    transmitted_translation_marker.color.a = 1.f;
    geometry_msgs::Point pt1;
    pt1.x = 0;
    pt1.y = 0;
    pt1.z = 0;
    transmitted_translation_marker.points.push_back(pt1);
    pt1.x = _ee_2_cp_pose_ht(0,3);
    pt1.y = _ee_2_cp_pose_ht(1,3);;
    pt1.z = _ee_2_cp_pose_ht(2,3);;
    transmitted_translation_marker.points.push_back(pt1);

    pg_markers.push_back(transmitted_translation_marker);

    //visualization_msgs::Marker empty_marker;
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
    empty_marker.id = 3 * this->_joint_id + 1;

    pg_markers.push_back(empty_marker);

    empty_marker.id = 3 * this->_joint_id + 2;
    pg_markers.push_back(empty_marker);

    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;

    pg_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 1;

    pg_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 2;

    pg_markers.push_back(empty_marker);

    return pg_markers;
}
