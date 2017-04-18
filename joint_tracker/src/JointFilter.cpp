#include "joint_tracker/JointFilter.h"

#include "omip_common/OMIPUtils.h"

#include <tf/transform_listener.h>

using namespace omip;

JointFilter::JointFilter():
    _loop_period_ns(-1),
    _joint_id(-1),
    _likelihood_sample_num(-1),
    _externally_set_likelihood(false),
    _measurement_timestamp_ns(0),
    _unnormalized_model_probability(0),
    _rrb_id(-1),
    _inverted_delta_srb_pose_in_rrbf(false),
    _from_inverted_to_non_inverted(false),
    _from_non_inverted_to_inverted(false),
    _slippage(0),
    _current_prob_grasp_failure(0.0)
{
    this->_initVariables(_joint_id);
    _max_ft_meas = Eigen::Vector3d(0,0,0);
    _previous_ft_running_avg =  Eigen::Vector3d(0,0,0);
}

void JointFilter::setInitialMeasurement(const joint_measurement_t &initial_measurement,
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

    // Estimate the transformation between the initial and the current pose of the second RB expressed in the coordinates of the reference RB frame
    this->_current_delta_pose_in_rrbf = (_srb_current_pose_in_rrbf.exp(1e-12)*(_srb_initial_pose_in_rrbf.exp(1e-12).inverse())).log(1e-12);

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

void JointFilter::_initVariables(JointCombinedFilterId joint_id)
{
    this->_joint_id = joint_id;

    this->_joint_state = 0;
    this->_joint_velocity = 0;

    this->_measurements_likelihood = 0.;
    this->_model_prior_probability = 1.0/4.0;
    this->_joint_position = Eigen::Vector3d(0,0,0);
    this->_uncertainty_joint_position = Eigen::Matrix3d::Identity();
    this->_joint_orientation_phi = 0;
    this->_joint_orientation_theta = 0;
    this->_joint_orientation = Eigen::Vector3d(1,0,0);
    this->_uncertainty_joint_orientation_phitheta = Eigen::Matrix2d::Identity();
    this->_uncertainty_joint_orientation_xyz = Eigen::Matrix3d::Identity();

    this->_rrb_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_rrb_previous_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_rrb_vel_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_vel_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);

    this->_srb_initial_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_current_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_previous_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_predicted_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_previous_predicted_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_current_delta_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_previous_delta_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_change_in_relative_pose_predicted_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);

    this->_rrb_pose_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();
    this->_rrb_vel_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();
    this->_srb_pose_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();

    this->_srb_predicted_pose_cov_in_rrbf = Eigen::Matrix<double, 6, 6>::Zero();
    this->_change_in_relative_pose_cov_predicted_in_rrbf = Eigen::Matrix<double, 6, 6>::Zero();

    this->_normalizing_term = 1;
}

JointFilter::~JointFilter()
{

}

JointFilter::JointFilter(const JointFilter &joint)
{
    this->_joint_id = joint._joint_id;
    this->_measurements_likelihood = joint._measurements_likelihood;
    this->_model_prior_probability = joint._model_prior_probability;
    this->_rrb_centroid_in_sf = joint._rrb_centroid_in_sf;
    this->_srb_centroid_in_sf = joint._srb_centroid_in_sf;
    this->_joint_position = joint._joint_position;
    this->_uncertainty_joint_position = joint._uncertainty_joint_position;
    this->_joint_orientation_phi = joint._joint_orientation_phi;
    this->_joint_orientation_theta = joint._joint_orientation_theta;
    this->_joint_orientation = joint._joint_orientation;
    this->_uncertainty_joint_orientation_phitheta = joint._uncertainty_joint_orientation_phitheta;
    this->_uncertainty_joint_orientation_xyz = joint._uncertainty_joint_orientation_xyz;

    this->_rrb_pose_in_sf = joint._rrb_pose_in_sf;
    this->_rrb_previous_pose_in_sf = joint._rrb_previous_pose_in_sf;
    this->_srb_pose_in_sf = joint._srb_pose_in_sf;
    this->_rrb_vel_in_sf= joint._rrb_vel_in_sf;
    this->_srb_vel_in_sf = joint._srb_vel_in_sf;

    this->_srb_initial_pose_in_rrbf = joint._srb_initial_pose_in_rrbf;
    this->_srb_current_pose_in_rrbf= joint._srb_current_pose_in_rrbf;
    this->_srb_previous_pose_in_rrbf = joint._srb_previous_pose_in_rrbf;
    this->_srb_predicted_pose_in_rrbf = joint._srb_predicted_pose_in_rrbf;

    this->_srb_previous_predicted_pose_in_rrbf = joint._srb_previous_predicted_pose_in_rrbf;

    this->_current_delta_pose_in_rrbf = joint._current_delta_pose_in_rrbf;
    this->_previous_delta_pose_in_rrbf = joint._previous_delta_pose_in_rrbf;
    this->_change_in_relative_pose_predicted_in_rrbf = joint._change_in_relative_pose_predicted_in_rrbf;

    this->_changes_in_relative_pose_in_rrbf = joint._changes_in_relative_pose_in_rrbf;

    this->_rrb_pose_cov_in_sf = joint._rrb_pose_cov_in_sf;
    this->_srb_pose_cov_in_sf = joint._srb_pose_cov_in_sf;

    this->_loop_period_ns = joint._loop_period_ns;

    this->_srb_predicted_pose_cov_in_rrbf = joint._srb_predicted_pose_cov_in_rrbf;
    this->_change_in_relative_pose_cov_predicted_in_rrbf = joint._change_in_relative_pose_cov_predicted_in_rrbf;
}

void JointFilter::setMeasurement(joint_measurement_t acquired_measurement, const double &measurement_timestamp_ns)
{
    _measurement_timestamp_ns = measurement_timestamp_ns;

    // Extract RB ids
    _rrb_id = acquired_measurement.first.rb_id;
    int srb_id = acquired_measurement.second.rb_id;

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
    adjointXcovXadjointT(T_rrbf_sf, _rrb_pose_cov_in_sf, rrb_pose_cov_in_rrbf);
    Eigen::Matrix<double, 6, 6> srb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf, _srb_pose_cov_in_sf, srb_pose_cov_in_rrbf);
    this->_srb_current_pose_cov_in_rrbf = rrb_pose_cov_in_rrbf + srb_pose_cov_in_rrbf;

    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd delta_displ = T_rrbf_srbf * (T_rrbf_srbf_t0.inverse());

    this->_current_delta_pose_in_rrbf = delta_displ.log(1.0e-20);

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

geometry_msgs::TwistWithCovariance JointFilter::getPredictedSRBPoseWithCovInRRBFrame()
{
    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = this->_srb_predicted_pose_in_rrbf.vx();
    hypothesis.twist.linear.y = this->_srb_predicted_pose_in_rrbf.vy();
    hypothesis.twist.linear.z = this->_srb_predicted_pose_in_rrbf.vz();
    hypothesis.twist.angular.x = this->_srb_predicted_pose_in_rrbf.rx();
    hypothesis.twist.angular.y = this->_srb_predicted_pose_in_rrbf.ry();
    hypothesis.twist.angular.z = this->_srb_predicted_pose_in_rrbf.rz();

    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> adjoint_eigen = this->_rrb_pose_in_sf.exp(1e-12).adjoint();
    Eigen::Matrix<double,6,6> new_pose_covariance = adjoint_eigen*this->_srb_predicted_pose_cov_in_rrbf*adjoint_eigen.transpose();
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = new_pose_covariance(i, j);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance JointFilter::getPredictedSRBDeltaPoseWithCovInRRBFrame()
{
    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = _change_in_relative_pose_predicted_in_rrbf.vx();
    hypothesis.twist.linear.y = _change_in_relative_pose_predicted_in_rrbf.vy();
    hypothesis.twist.linear.z = _change_in_relative_pose_predicted_in_rrbf.vz();
    hypothesis.twist.angular.x = _change_in_relative_pose_predicted_in_rrbf.rx();
    hypothesis.twist.angular.y = _change_in_relative_pose_predicted_in_rrbf.ry();
    hypothesis.twist.angular.z = _change_in_relative_pose_predicted_in_rrbf.rz();

    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
             hypothesis.covariance[6 * i + j] = _change_in_relative_pose_cov_predicted_in_rrbf(i,j);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance JointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_vel_in_sf*(this->_loop_period_ns/1e9);
    Eigen::Displacementd T_sf_rrbf_next = (delta_rrb_in_sf.exp(1e-12)*this->_rrb_pose_in_sf.exp(1e-12));

    Eigen::Twistd predicted_delta_pose_in_sf = this->_rrb_vel_in_sf*(this->_loop_period_ns/1e9)
            + T_sf_rrbf_next.adjoint()*this->_change_in_relative_pose_predicted_in_rrbf;

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = predicted_delta_pose_in_sf.vx();
    hypothesis.twist.linear.y = predicted_delta_pose_in_sf.vy();
    hypothesis.twist.linear.z = predicted_delta_pose_in_sf.vz();
    hypothesis.twist.angular.x = predicted_delta_pose_in_sf.rx();
    hypothesis.twist.angular.y = predicted_delta_pose_in_sf.ry();
    hypothesis.twist.angular.z = predicted_delta_pose_in_sf.rz();

    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> adjoint_eigen = this->_rrb_pose_in_sf.exp(1e-12).adjoint();
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf + this->_rrb_vel_cov_in_sf*(this->_loop_period_ns/1e9) + adjoint_eigen*this->_change_in_relative_pose_cov_predicted_in_rrbf*adjoint_eigen.transpose();
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = new_pose_covariance(i, j);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance JointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    Eigen::Twistd predicted_delta_pose_in_sf = this->_rrb_vel_in_sf + this->_rrb_pose_in_sf.exp(1e-12).adjoint()*(this->_change_in_relative_pose_predicted_in_rrbf/(_loop_period_ns/1e9));

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = predicted_delta_pose_in_sf.vx();
    hypothesis.twist.linear.y = predicted_delta_pose_in_sf.vy();
    hypothesis.twist.linear.z = predicted_delta_pose_in_sf.vz();
    hypothesis.twist.angular.x = predicted_delta_pose_in_sf.rx();
    hypothesis.twist.angular.y = predicted_delta_pose_in_sf.ry();
    hypothesis.twist.angular.z = predicted_delta_pose_in_sf.rz();

    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> adjoint_eigen = this->_rrb_pose_in_sf.exp(1e-12).adjoint();
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_vel_cov_in_sf + adjoint_eigen*(this->_change_in_relative_pose_cov_predicted_in_rrbf/(_loop_period_ns/1e9))*adjoint_eigen.transpose();
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = new_pose_covariance(i, j);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance JointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_vel_in_sf*(this->_loop_period_ns/1e9);
    Eigen::Displacementd T_sf_rrbf_next = (delta_rrb_in_sf.exp(1e-12)*this->_rrb_pose_in_sf.exp(1e-12));
    Eigen::Displacementd T_rrbf_srbf_next = this->_srb_predicted_pose_in_rrbf.exp(1e-12);

    Eigen::Displacementd T_sf_srbf_next = T_sf_rrbf_next*T_rrbf_srbf_next;

    Eigen::Twistd srb_next_pose_in_sf = T_sf_srbf_next.log(1e-12);

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = srb_next_pose_in_sf.vx();
    hypothesis.twist.linear.y = srb_next_pose_in_sf.vy();
    hypothesis.twist.linear.z = srb_next_pose_in_sf.vz();
    hypothesis.twist.angular.x = srb_next_pose_in_sf.rx();
    hypothesis.twist.angular.y = srb_next_pose_in_sf.ry();
    hypothesis.twist.angular.z = srb_next_pose_in_sf.rz();

    // I need the covariance of the absolute pose of the second RB, so I add the cov of the relative pose to the
    // cov of the reference pose. I need to "move" the second covariance to align it to the reference frame (see Barfoot)
    Eigen::Matrix<double,6,6> adjoint_eigen = this->_rrb_pose_in_sf.exp(1e-12).adjoint();
    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf + adjoint_eigen*this->_change_in_relative_pose_cov_predicted_in_rrbf*adjoint_eigen.transpose();
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

Eigen::Twistd JointFilter::getPredictedMeasurement() const
{
    return this->_srb_predicted_pose_in_rrbf;
}

void JointFilter::setLoopPeriodNS(double loop_period_ns)
{
    this->_loop_period_ns = loop_period_ns;
}

double JointFilter::getLoopPeriodNS() const
{
    return this->_loop_period_ns;
}

void JointFilter::setNumSamplesForLikelihoodEstimation(int likelihood_sample_num)
{
    this->_likelihood_sample_num = likelihood_sample_num;
}

int JointFilter::getNumSamplesForLikelihoodEstimation() const
{
    return this->_likelihood_sample_num;
}

void JointFilter::setNormalizingTerm(double normalizing_term)
{
    this->_normalizing_term = normalizing_term;
}

double JointFilter::getNormalizingTerm()
{
    return this->_normalizing_term;
}

void JointFilter::setModelPriorProbability(double model_prior_probability)
{
    this->_model_prior_probability = model_prior_probability;
}

double JointFilter::getModelPriorProbability() const
{
    return this->_model_prior_probability;
}

double JointFilter::getLikelihoodOfLastMeasurements() const
{
    return this->_measurements_likelihood;
}

void JointFilter::setLikelihoodOfLastMeasurements(double likelihood)
{
    this->_measurements_likelihood = likelihood;
    this->_externally_set_likelihood = true;
}

double JointFilter::getUnnormalizedProbabilityOfJointFilter() const
{
    return _unnormalized_model_probability;
}

double JointFilter::getProbabilityOfJointFilter() const
{
    return (_unnormalized_model_probability / this->_normalizing_term);
}

void JointFilter::setJointId(JointCombinedFilterId joint_id)
{
    this->_joint_id = joint_id;
}

JointCombinedFilterId JointFilter::getJointId() const
{
    return this->_joint_id;
}

double JointFilter::getJointState() const
{
    return _joint_state;
}

double JointFilter::getJointVelocity() const
{
    return _joint_velocity;
}

double JointFilter::getOrientationPhiInRRBFrame()  const
{
    return this->_joint_orientation_phi;
}

double JointFilter::getOrientationThetaInRRBFrame()  const
{
    return this->_joint_orientation_theta;
}

double JointFilter::getCovarianceOrientationPhiPhiInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(0,0);
}

double JointFilter::getCovarianceOrientationThetaThetaInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(1,1);
}

double JointFilter::getCovarianceOrientationPhiThetaInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(1,0);
}

Eigen::Matrix3d JointFilter::getCovariancePositionXYZInRRBFrame() const
{
    return this->_uncertainty_joint_position;
}

Eigen::Vector3d JointFilter::getJointPositionInRRBFrame() const
{
    return this->_joint_position;
}

Eigen::Vector3d JointFilter::getJointOrientationRPYInRRBFrame() const
{
    double phi = std::acos(this->_joint_orientation.z() / this->_joint_orientation.norm());
    double theta = std::atan(this->_joint_orientation.y() / this->_joint_orientation.x());
    Eigen::Vector3d joint_ori_rpy(0., theta, phi);

    return joint_ori_rpy;
}

Eigen::Vector3d JointFilter::getJointOrientationUnitaryVector() const
{
    return this->_joint_orientation;
}

void JointFilter::setCovariancePrior(double  prior_cov_vel)
{
    this->_prior_cov_vel =  prior_cov_vel;
}

void JointFilter::setCovarianceAdditiveSystemNoisePhi(double  sigma_sys_noise_phi)
{
    this->_sigma_sys_noise_phi =  sigma_sys_noise_phi;
}

void JointFilter::setCovarianceAdditiveSystemNoiseTheta(double  sigma_sys_noise_theta)
{
    this->_sigma_sys_noise_theta =  sigma_sys_noise_theta;
}

void JointFilter::setCovarianceAdditiveSystemNoiseOx(double  sigma_sys_noise_ox)
{
    this->_sigma_sys_noise_ox =  sigma_sys_noise_ox;
}

void JointFilter::setCovarianceAdditiveSystemNoiseOy(double  sigma_sys_noise_oy)
{
    this->_sigma_sys_noise_oy =  sigma_sys_noise_oy;
}

void JointFilter::setCovarianceAdditiveSystemNoiseOz(double  sigma_sys_noise_oz)
{
    this->_sigma_sys_noise_oz =  sigma_sys_noise_oz;
}

void JointFilter::setCovarianceAdditiveSystemNoisePx(double  sigma_sys_noise_px)
{
    this->_sigma_sys_noise_px =  sigma_sys_noise_px;
}

void JointFilter::setCovarianceAdditiveSystemNoisePy(double  sigma_sys_noise_py)
{
    this->_sigma_sys_noise_py =  sigma_sys_noise_py;
}

void JointFilter::setCovarianceAdditiveSystemNoisePz(double  sigma_sys_noise_pz)
{
    this->_sigma_sys_noise_pz =  sigma_sys_noise_pz;
}

void JointFilter::setCovarianceAdditiveSystemNoiseJointState(double  sigma_sys_noise_pv)
{
    this->_sigma_sys_noise_jv =  sigma_sys_noise_pv;
}

void JointFilter::setCovarianceAdditiveSystemNoiseJointVelocity(double  sigma_sys_noise_pvd)
{
    this->_sigma_sys_noise_jvd =  sigma_sys_noise_pvd;
}

void JointFilter::setCovarianceMeasurementNoise(double  sigma_meas_noise)
{
    this->_sigma_meas_noise =  sigma_meas_noise;
}

void JointFilter::initialize()
{
#ifdef PUBLISH_PREDICTED_POSE_AS_PWC
    // This is used to visualize the predictions based on the joint hypothesis
    std::ostringstream oss_pwc_topic;
    oss_pwc_topic << "/joint_tracker/predpose_srb_j" << this->_joint_id << "_" << this->getJointFilterTypeStr();
    _predicted_next_pose_publisher = _predicted_next_pose_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(oss_pwc_topic.str(), 100);
#endif
}

void JointFilter::_estimateFTMeasurementLikelihood()
{
    Eigen::Vector3d force(_ft_meas.at(0), _ft_meas.at(1), _ft_meas.at(2));

    if(_previous_ft_running_avg.norm() < force.norm())
    {
        _previous_ft_running_avg = force;
    }else{
        _previous_ft_running_avg = 0.9*_previous_ft_running_avg + 0.1*force;
    }

    if(_max_ft_meas.norm() < force.norm())
    {
        _max_ft_meas = force;
    }

    // Two factors to detect grasp failure -> disconnected:
    // 1) The absolute exerted force
    // 2) The difference between current force and the running average of the last forces
    double prob_of_grasp_failure1 = 0.;
    if(_max_ft_meas.norm() > MIN_ACTUATING_FORCE)
    {
        prob_of_grasp_failure1 = 1 - 0.5*(1.0 + erf((force.norm() - MIN_ACTUATING_FORCE)/(SIGMA_ACTUATING_FORCE*sqrt(2))));
    }

    double prob_of_grasp_failure2 = 0.5*(1.0 + erf(((_previous_ft_running_avg.norm() - force.norm()) - MAX_DECAY_FORCE)/(SIGMA_DECAY_FORCE*sqrt(2))));

    _current_prob_grasp_failure = std::max(prob_of_grasp_failure1, prob_of_grasp_failure2);
}
