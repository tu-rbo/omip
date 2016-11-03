#include "joint_tracker/DisconnectedJointFilter.h"

#include <random>

using namespace omip;

DisconnectedJointFilter::DisconnectedJointFilter() :
    JointFilter()
{
    _unnormalized_model_probability = 0.8;
}

DisconnectedJointFilter::~DisconnectedJointFilter()
{

}

DisconnectedJointFilter::DisconnectedJointFilter(const DisconnectedJointFilter &disconnected_joint) :
    JointFilter(disconnected_joint)
{
}

void DisconnectedJointFilter::initialize()
{

}

double DisconnectedJointFilter::getProbabilityOfJointFilter() const
{
    return (_unnormalized_model_probability / this->_normalizing_term);//this->_measurements_likelihood;
}

geometry_msgs::TwistWithCovariance DisconnectedJointFilter::getPredictedSRBDeltaPoseWithCovInSensorFrame()
{
    // We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Eigen::Twistd srb_delta_pose_in_sf_next = Eigen::Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = srb_delta_pose_in_sf_next.vx();
    hypothesis.twist.linear.y = srb_delta_pose_in_sf_next.vy();
    hypothesis.twist.linear.z = srb_delta_pose_in_sf_next.vz();
    hypothesis.twist.angular.x = srb_delta_pose_in_sf_next.rx();
    hypothesis.twist.angular.y = srb_delta_pose_in_sf_next.ry();
    hypothesis.twist.angular.z = srb_delta_pose_in_sf_next.rz();

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance[6 * i + j] = 1.0e6;
            }else
            {
                hypothesis.covariance[6 * i + j] = 1.0e3;
            }
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance DisconnectedJointFilter::getPredictedSRBVelocityWithCovInSensorFrame()
{
    // We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Eigen::Twistd srb_delta_pose_in_sf_next = Eigen::Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    geometry_msgs::TwistWithCovariance hypothesis;

    hypothesis.twist.linear.x = srb_delta_pose_in_sf_next.vx();
    hypothesis.twist.linear.y = srb_delta_pose_in_sf_next.vy();
    hypothesis.twist.linear.z = srb_delta_pose_in_sf_next.vz();
    hypothesis.twist.angular.x = srb_delta_pose_in_sf_next.rx();
    hypothesis.twist.angular.y = srb_delta_pose_in_sf_next.ry();
    hypothesis.twist.angular.z = srb_delta_pose_in_sf_next.rz();

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance[6 * i + j] = 1.0e6;
            }else
            {
                hypothesis.covariance[6 * i + j] = 1.0e3;
            }
        }
    }

    return hypothesis;
}

geometry_msgs::TwistWithCovariance DisconnectedJointFilter::getPredictedSRBPoseWithCovInSensorFrame()
{
    // If the two rigid bodies are disconnecte, the pose of the second rigid body is INDEPENDENT of the motion of the reference rigid body

    // OPTION1: This prediction is the same as predicting in the rigid body level using velocity
    //Eigen::Twistd delta_motion_srb = this->_srb_current_vel_in_sf*(this->_loop_period_ns/1e9);
    //Eigen::Twistd srb_pose_in_sf_next = (delta_motion_srb.exp(1e-12)*this->_srb_current_pose_in_sf.exp(1e-12)).log(1e-12);

    // OPTION2: We just give a random prediction
    std::default_random_engine generator;;
    std::uniform_real_distribution<double> distr(-100.0, 100.0); // define the range
    Eigen::Twistd srb_pose_in_sf_next = Eigen::Twistd(distr(generator),distr(generator),distr(generator),
                                                      distr(generator),distr(generator),distr(generator) );

    geometry_msgs::TwistWithCovariance hypothesis;
    hypothesis.twist.linear.x = srb_pose_in_sf_next.vx();
    hypothesis.twist.linear.y = srb_pose_in_sf_next.vy();
    hypothesis.twist.linear.z = srb_pose_in_sf_next.vz();
    hypothesis.twist.angular.x = srb_pose_in_sf_next.rx();
    hypothesis.twist.angular.y = srb_pose_in_sf_next.ry();
    hypothesis.twist.angular.z = srb_pose_in_sf_next.rz();

    // If the bodies are disconnected, we cannot use the kinematic structure to give an accurate predition
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            if(i == j)
            {
                hypothesis.covariance[6 * i + j] = 1.0e6;
            }else
            {
                hypothesis.covariance[6 * i + j] = 1.0e3;
            }
        }
    }


#ifdef PUBLISH_PREDICTED_POSE_AS_PWC
    // This is used to visualize the predictions based on the joint hypothesis
    // Get the poses and covariances on the poses (pose with cov) of each RB
    geometry_msgs::PoseWithCovarianceStamped pose_with_cov_stamped;
    pose_with_cov_stamped.header.stamp = ros::Time::now();
    pose_with_cov_stamped.header.frame_id = "camera_rgb_optical_frame";

    Eigen::Displacementd displ_from_twist = srb_pose_in_sf_next.exp(1e-12);
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

std::vector<visualization_msgs::Marker> DisconnectedJointFilter::getJointMarkersInRRBFrame() const
{
    // Delete other markers
    std::vector<visualization_msgs::Marker> empty_vector;
    visualization_msgs::Marker empty_marker;
    empty_marker.pose.position.x = 0.;
    empty_marker.pose.position.y = 0.;
    empty_marker.pose.position.z = 0.;
    //empty_marker.header.frame_id = "camera_rgb_optical_frame";
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
    empty_marker.id = 3 * this->_joint_id;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);
    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);

    return empty_vector;
}

JointFilterType DisconnectedJointFilter::getJointFilterType() const
{
    return DISCONNECTED_JOINT;
}

std::string DisconnectedJointFilter::getJointFilterTypeStr() const
{
    return std::string("DisconnectedJointFilter");
}


