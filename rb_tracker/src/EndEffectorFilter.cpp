
#include <rb_tracker/EndEffectorFilter.h>

#include <boost/foreach.hpp>

#include "omip_common/OMIPUtils.h"

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

EndEffectorFilter::EndEffectorFilter(double loop_period_ns,
                                     const std::vector<Eigen::Matrix4d> &first_transform,
                                     const Eigen::Twistd &initial_velocity,
                                     FeaturesDataBase::Ptr feats_database,
                                     double estimation_error_threshold) :
    RBFilter(loop_period_ns, first_transform, initial_velocity, feats_database, estimation_error_threshold),   
    _previous_time_pose_ns(0),
    _current_time_pose_ns(0),
    _invalid_measurement(true)
{
    _previous_eef_wrt_cf = first_transform[first_transform.size()-1];

    _current_measured_pose_ee_wrt_cf = Eigen::Matrix4d::Identity();
    _delta_eef_wrt_cf = Eigen::Matrix4d::Identity();

    _ee_location_wrt_cf = Feature::Location(0,0,0);

    _proprioception_meas_cov_eig = Eigen::Matrix<double, 6, 6>::Identity();

    // I want to use proprioception to update pose only if there is no information from vision
    for(int i=0; i<6; i++)
    {
        _proprioception_meas_cov_eig(i,i) = 0.005;
    }
}

void EndEffectorFilter::_setInitialEndEffectorPose(const Eigen::Matrix4d& initial_ee_pose, double time_pose_ns)
{
    _previous_eef_wrt_cf = initial_ee_pose;
    _previous_time_pose_ns = time_pose_ns;
}

void EndEffectorFilter::setMeasurement(const Eigen::Matrix4d& current_eef_wrt_cf, double time_pose_ns)
{
    // This special homogeneous matrix indicates that we couldn't retrieve the pose of the end effector
    // and therefore we should use the prediction as corrected state (equivalent to updating with not enough features)
    if(current_eef_wrt_cf == Eigen::Matrix4d::Identity())
    {
        std::cout << "Invalid proprioception measurement. Using prediction as corrected state!" << std::endl;
        _invalid_measurement = true;
    }
    // If it is the first time we receive a valid measurement of the pose of the end effector
    else if(_previous_eef_wrt_cf == Eigen::Matrix4d::Identity())
    {
        _invalid_measurement = true;
        this->_setInitialEndEffectorPose(current_eef_wrt_cf, time_pose_ns);
    }else{
        _invalid_measurement = false;
        _current_measured_pose_ee_wrt_cf = current_eef_wrt_cf;
        _current_time_pose_ns = time_pose_ns;

        // Estimate the delta between the previous and the current poses of the end effector wrt the camera frame
        _delta_eef_wrt_cf = _current_measured_pose_ee_wrt_cf*(_previous_eef_wrt_cf.inverse());

        Eigen::Twistd delta_eef_wrt_cf_ec;
        TransformMatrix2Twist(_delta_eef_wrt_cf, delta_eef_wrt_cf_ec);
        _current_measured_velocity_ee_wrt_cf = delta_eef_wrt_cf_ec/((_current_time_pose_ns - _previous_time_pose_ns)/1e9);

        _previous_eef_wrt_cf = _current_measured_pose_ee_wrt_cf;
        _previous_time_pose_ns = _current_time_pose_ns;
    }
}

// This version uses the internal forward model (based on velocity) to predict the state of the filter (the pose and velocity of the end effector)
// It is the normal prediction step of all RBFilters
void EndEffectorFilter::predictState(double time_interval_ns)
{
    //TODO: Try to obtain a better estimation of the velocity of the EE from TF (pose now - pose in the future)/time

    RBFilter::predictState(time_interval_ns);
    _predicted_pose = _predicted_pose_vh;
    _predicted_pose_covariance = _predicted_pose_cov_vh;
}

void EndEffectorFilter::correctState()
{
    // If the measurement was wrong we propagate the predicted state as corrected state
    if(_invalid_measurement)
    {
        _pose = _predicted_pose;
        _pose_covariance = _predicted_pose_covariance;

        Eigen::Matrix4d delta_pose = _pose*(_trajectory.at(_trajectory.size()-1).inverse());

        Eigen::Twistd delta_pose_ec;
        TransformMatrix2Twist(delta_pose, delta_pose_ec);
        this->_velocity = delta_pose_ec/(_last_time_interval_ns/1e9);
        this->_trajectory.push_back(this->_pose);

        ROS_ERROR_STREAM_NAMED("EndEffectorFilter.correctState","RB" << this->_id << ": Proprioception using predicted state as corrected state!");
    }else{
        // Compute the MAP of two pose estimates (http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1238637 Computing MAP trajectories by representing, propagating
        //and combining PDFs over groups)
        Eigen::Matrix<double, 6, 6> inv_sum_of_covs = (_predicted_pose_covariance + _proprioception_meas_cov_eig).inverse();
        Eigen::Matrix4d delta_ht = _predicted_pose*(_current_measured_pose_ee_wrt_cf.inverse());
        Eigen::Twistd delta_ec;
        TransformMatrix2Twist(delta_ht, delta_ec);
        Eigen::Matrix<double, 6, 1> delta_vec;
        delta_vec << delta_ec.vx(), delta_ec.vy(), delta_ec.vz(),
                delta_ec.rx(), delta_ec.ry(), delta_ec.rz();

        //Equation 13
        Eigen::Matrix<double, 6, 1> epsilon_1_vec = _proprioception_meas_cov_eig*inv_sum_of_covs*delta_vec;

        Eigen::Twistd epsilon_1_ec(epsilon_1_vec[3], epsilon_1_vec[4], epsilon_1_vec[5], epsilon_1_vec[0], epsilon_1_vec[1], epsilon_1_vec[2]);
        Eigen::Matrix4d epsilon_1_ht;
        Twist2TransformMatrix(epsilon_1_ec, epsilon_1_ht);

        //Equation 9
        Eigen::Matrix4d new_pose = epsilon_1_ht*_current_measured_pose_ee_wrt_cf;

        //Equation 19
        Eigen::Matrix<double, 6, 6> new_pose_cov = _proprioception_meas_cov_eig*inv_sum_of_covs*_predicted_pose_covariance;

        _pose = new_pose;
        _pose_covariance = new_pose_cov;

        Eigen::Matrix4d delta_pose = _pose*_trajectory.at(_trajectory.size()-1).inverse();
        Eigen::Twistd delta_pose_ec;
        TransformMatrix2Twist(delta_pose, delta_pose_ec);
        this->_velocity = delta_pose_ec/(_last_time_interval_ns/1e9);
        this->_trajectory.push_back(this->_pose);

        Eigen::Twistd pose_ec;
        TransformMatrix2Twist( this->_pose, pose_ec);

        ROS_INFO_STREAM_NAMED("EndEffectorFilter.correctState","RB " << this->_id << ": Proprioception, Pose (vx,vy,vz,rx,ry,rz): " << pose_ec );//<< " " << this->_velocity);
    }
}

//        // This is like a simple kalman filter with state transition and measurement models the identity
//        // We cannot subtract them directly because the logarithm of homogeneous transformations is not continuous
//        // which makes that the exponential coordinates that represent two very close homogeneous transformations could be
//        // very different!!!!
//        Eigen::Matrix4d innovation_ht = _current_measured_pose_ee_wrt_cf*_predicted_pose.inverse();
//        Eigen::Matrix<double, 6, 1> innovation;
//        Eigen::Twistd innovation_ec;
//        TransformMatrix2Twist(innovation_ht, innovation_ec);
//        innovation << innovation_ec.rx(), innovation_ec.ry(), innovation_ec.rz(),
//                innovation_ec.vx(), innovation_ec.vy(), innovation_ec.vz();

//        Eigen::Matrix<double, 6, 6> innovation_covariance = _predicted_pose_covariance + _proprioception_meas_cov_eig;

//        Eigen::Matrix<double, 6, 6> kalman_gain = _predicted_pose_covariance*innovation_covariance.inverse();

//        Eigen::Matrix<double, 6, 1> pose_update = kalman_gain*innovation;

//        Eigen::Twistd pose_update_ec(pose_update[3], pose_update[4], pose_update[5], pose_update[0], pose_update[1], pose_update[2]);

//        //TODO: Remove this!!! This is taking directly the measured pose based on proprioception as current pose!!!!
//        //_pose = pose_update_ec.exp(1e-12).toHomogeneousMatrix()*_predicted_pose;
//        _pose = _current_measured_pose_ee_wrt_cf;

//        Eigen::Matrix<double, 6, 6> identity6 = Eigen::Matrix<double, 6, 6>::Identity();
//        _pose_covariance = (identity6 - kalman_gain)*_predicted_pose_covariance;

//        Eigen::Matrix4d delta_pose = _pose*_trajectory.at(_trajectory.size()-1).inverse();

//        Eigen::Twistd delta_pose_ec;
//        TransformMatrix2Twist(delta_pose, delta_pose_ec);
//        this->_velocity = delta_pose_ec/(_last_time_interval_ns/1e9);
//        this->_trajectory.push_back(this->_pose);
