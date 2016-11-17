#include "joint_tracker/RigidJointFilter.h"

using namespace omip;

RigidJointFilter::RigidJointFilter():
    JointFilter()
{
    // Initially the most probable joint type is rigid
    _measurements_likelihood = 1.0;
    _motion_memory_prior = 1.0;
}

RigidJointFilter::~RigidJointFilter()
{

}

RigidJointFilter::RigidJointFilter(const RigidJointFilter &rigid_joint) :
    JointFilter(rigid_joint)
{
}

void RigidJointFilter::initialize()
{
    JointFilter::initialize();
    // Initially the most probable joint type is rigid
    _measurements_likelihood = 1.0;
}

void RigidJointFilter::setMaxTranslationRigid(double max_trans)
{
    this->_rig_max_translation = max_trans;
}

void RigidJointFilter::setMaxRotationRigid(double max_rot)
{
    this->_rig_max_rotation = max_rot;
}

void RigidJointFilter::predictMeasurement()
{
    this->_predicted_delta_pose_in_rrbf = Eigen::Twistd(0., 0., 0., 0., 0., 0.);

    Eigen::Displacementd predicted_delta = this->_predicted_delta_pose_in_rrbf.exp(1e-20);
    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);
}

void RigidJointFilter::estimateMeasurementHistoryLikelihood()
{
    double p_one_meas_given_model_params = 0;
    double p_all_meas_given_model_params = 0;

    double sigma_translation = 0.05;
    double sigma_rotation = 0.2;

    double accumulated_error = 0.;

    double frame_counter = 0.;

    // Estimate the number of samples used for likelihood estimation
    size_t trajectory_length = this->_delta_poses_in_rrbf.size();
    size_t amount_samples = std::min(trajectory_length, (size_t)this->_likelihood_sample_num);

    // Estimate the delta for the iterator over the number of samples
    // This makes that we take _likelihood_sample_num uniformly distributed over the whole trajectory, instead of the last _likelihood_sample_num
    double delta_idx_samples = (double)std::max(1., (double)trajectory_length/(double)this->_likelihood_sample_num);

    size_t current_idx = 0;

    // We test the last window_length motions
    for (size_t sample_idx = 0; sample_idx < amount_samples; sample_idx++)
    {
        current_idx = boost::math::round(sample_idx*delta_idx_samples);
        Eigen::Displacementd rb2_last_delta_relative_displ = this->_delta_poses_in_rrbf.at(current_idx).exp(1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation = rb2_last_delta_relative_displ.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation = Eigen::Quaterniond(rb2_last_delta_relative_displ.qw(),
                                                                                 rb2_last_delta_relative_displ.qx(),
                                                                                 rb2_last_delta_relative_displ.qy(),
                                                                                 rb2_last_delta_relative_displ.qz());

        Eigen::Vector3d rigid_joint_translation = Eigen::Vector3d(0.,0.,0.);
        Eigen::Displacementd rb2_last_delta_relative_displ_rigid_hyp = Eigen::Twistd(0.,
                                                                                     0.,
                                                                                     0.,
                                                                                     rigid_joint_translation.x(),
                                                                                     rigid_joint_translation.y(),
                                                                                     rigid_joint_translation.z()).exp(1e-12);
        Eigen::Vector3d rb2_last_delta_relative_translation_rigid_hyp = rb2_last_delta_relative_displ_rigid_hyp.getTranslation();
        Eigen::Quaterniond rb2_last_delta_relative_rotation_rigid_hyp = Eigen::Quaterniond(rb2_last_delta_relative_displ_rigid_hyp.qw(),
                                                                                           rb2_last_delta_relative_displ_rigid_hyp.qx(),
                                                                                           rb2_last_delta_relative_displ_rigid_hyp.qy(),
                                                                                           rb2_last_delta_relative_displ_rigid_hyp.qz());

        // Distance proposed by park and okamura in "Kinematic calibration using the product of exponentials formula"
        double translation_error = (rb2_last_delta_relative_translation - rb2_last_delta_relative_translation_rigid_hyp).norm();

        if(translation_error > this->_rig_max_translation)
        {
            _motion_memory_prior = 0.0;
        }

        Eigen::Quaterniond rotation_error = rb2_last_delta_relative_rotation.inverse() * rb2_last_delta_relative_rotation_rigid_hyp;
        double rotation_error_angle = Eigen::Displacementd(0., 0., 0., rotation_error.w(),rotation_error.x(), rotation_error.y(),rotation_error.z()).log(1e-12).norm();


        if(rotation_error_angle > this->_rig_max_rotation)
        {
            _motion_memory_prior = 0.0;
        }

        accumulated_error += translation_error + fabs(rotation_error_angle);

        p_one_meas_given_model_params = (1.0/(sigma_translation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(translation_error/sigma_translation, 2)) *
                (1.0/(sigma_rotation*sqrt(2.0*M_PI)))*exp((-1.0/2.0)*pow(rotation_error_angle/sigma_rotation, 2));

        p_all_meas_given_model_params += (p_one_meas_given_model_params/(double)amount_samples);

        frame_counter++;
    }

    this->_measurements_likelihood = p_all_meas_given_model_params;
}

void RigidJointFilter::estimateUnnormalizedModelProbability()
{
    this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood*_motion_memory_prior;
}

geometry_msgs::TwistWithCovariance RigidJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    // The delta in the pose of the SRB is the delta in the pose of the RRB (its velocity!)
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    Eigen::Twistd delta_srb_in_sf = delta_rrb_in_sf;

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = delta_srb_in_sf.vx();
    hypothesis.twist.linear.y = delta_srb_in_sf.vy();
    hypothesis.twist.linear.z = delta_srb_in_sf.vz();
    hypothesis.twist.angular.x = delta_srb_in_sf.rx();
    hypothesis.twist.angular.y = delta_srb_in_sf.ry();
    hypothesis.twist.angular.z = delta_srb_in_sf.rz();

    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = this->_rrb_vel_cov_in_sf(i, j)*(this->_loop_period_ns/1e9);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance RigidJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    // The delta in the pose of the SRB is the delta in the pose of the RRB (its velocity!)
    Eigen::Twistd delta_rrb_in_sf = this->_rrb_current_vel_in_sf;
    Eigen::Twistd delta_srb_in_sf = delta_rrb_in_sf;

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = delta_srb_in_sf.vx();
    hypothesis.twist.linear.y = delta_srb_in_sf.vy();
    hypothesis.twist.linear.z = delta_srb_in_sf.vz();
    hypothesis.twist.angular.x = delta_srb_in_sf.rx();
    hypothesis.twist.angular.y = delta_srb_in_sf.ry();
    hypothesis.twist.angular.z = delta_srb_in_sf.rz();

    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            hypothesis.covariance[6 * i + j] = this->_rrb_vel_cov_in_sf(i, j);
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance RigidJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
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

    Eigen::Matrix<double,6,6> new_pose_covariance = this->_rrb_pose_cov_in_sf;
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
            pose_with_cov_stamped.pose.covariance[6 * i + j] = hypothesis.covariance[6 * i + j];
    _predicted_next_pose_publisher.publish(pose_with_cov_stamped);
#endif

    return hypothesis;
}

std::vector<visualization_msgs::Marker> RigidJointFilter::getJointMarkersInRRBFrame() const
{
    std::vector<visualization_msgs::Marker> rigid_markers;
    // CONNECTION MARKER ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    visualization_msgs::Marker connection_marker;
    connection_marker.ns = "kinematic_structure";
    connection_marker.action = visualization_msgs::Marker::ADD;
    connection_marker.type = visualization_msgs::Marker::ARROW;
    connection_marker.id = 3 * this->_joint_id;
    connection_marker.scale.x = 0.02f;
    connection_marker.scale.y = 0.f;
    connection_marker.scale.z = 0.f;
    connection_marker.color.r = 0.f;
    connection_marker.color.g = 0.f;
    connection_marker.color.b = 1.f;
    connection_marker.color.a = 1.f;
    // Estimate position from supporting features:
    Eigen::Displacementd current_ref_pose_displ = this->_rrb_current_pose_in_sf.exp(1e-12);
    Eigen::Affine3d current_ref_pose;
    current_ref_pose.matrix() = current_ref_pose_displ.toHomogeneousMatrix();
    Eigen::Vector3d second_centroid_relative_to_ref_body = current_ref_pose.inverse() * this->_srb_centroid_in_sf;
    geometry_msgs::Point pt1;
    pt1.x = second_centroid_relative_to_ref_body.x();
    pt1.y = second_centroid_relative_to_ref_body.y();
    pt1.z = second_centroid_relative_to_ref_body.z();
    connection_marker.points.push_back(pt1);
    // The markers are now defined wrt to the reference frame and I want the rigid joint marker to go from the centroid of
    // the second rigid body to the centroid of the reference rigid body
    Eigen::Vector3d first_centroid_relative_to_ref_body = current_ref_pose.inverse() * this->_rrb_centroid_in_sf;
    geometry_msgs::Point pt2;
    pt2.x = first_centroid_relative_to_ref_body.x();
    pt2.y = first_centroid_relative_to_ref_body.y();
    pt2.z = first_centroid_relative_to_ref_body.z();
    connection_marker.points.push_back(pt2);

    rigid_markers.push_back(connection_marker);

    // Delete other markers
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

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3 * this->_joint_id + 2;
    rigid_markers.push_back(empty_marker);

    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 1;

    rigid_markers.push_back(empty_marker);

    empty_marker.id = 3* this->_joint_id + 2;

    rigid_markers.push_back(empty_marker);

    return rigid_markers;
}

JointFilterType RigidJointFilter::getJointFilterType() const
{
    return RIGID_JOINT;
}

std::string RigidJointFilter::getJointFilterTypeStr() const
{
    return std::string("RigidJointFilter");
}

