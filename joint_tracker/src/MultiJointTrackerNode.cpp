#include <boost/foreach.hpp>

#include "joint_tracker/MultiJointTrackerNode.h"
#include "joint_tracker/MultiJointTracker.h"

#include "geometry_msgs/PoseArray.h"

#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <stdio.h>

#include <rosbag/message_instance.h>

#include <rosbag/query.h>

#include <rosbag/view.h>

#include <ros/package.h>

#include <Eigen/Eigen>

#include <Eigen/Geometry>

#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>

#include <omip_msgs/JointMsg.h>

#define LINES_OF_TERMINAL_JT 20

using namespace omip;

MultiJointTrackerNode::MultiJointTrackerNode() :
    RecursiveEstimatorNodeInterface(1),
    _loop_period_ns(0.),
    _sensor_fps(0.0),
    _processing_factor(0),
    _ft_values_being_used(false)

{
    this->_namespace = std::string("joint_tracker");
    this->ReadParameters();
    this->_measurement_subscriber= this->_measurements_node_handle.subscribe( "/rb_tracker/state", 1, &MultiJointTrackerNode::measurementCallback, this);

    this->_measurement_subscriber_ft = this->_measurements_node_handle.subscribe( this->_ft_topic, 1, &MultiJointTrackerNode::measurementFTCallback, this);


    this->_state_publisher = this->_measurements_node_handle.advertise<omip_msgs::KinematicStructureMsg>(this->_namespace + "/state", 100);

    this->_state_publisher_rviz_markers = this->_measurements_node_handle.advertise<visualization_msgs::MarkerArray>(this->_namespace + "/markers", 100);

    this->_measurement_prediction_publisher = this->_measurements_node_handle.advertise<rbt_state_t>( "/rb_tracker/predicted_measurement", 100);


    this->_urdf_pub_service = this->_measurements_node_handle.advertiseService("/joint_tracker/urdf_publisher_srv", &MultiJointTrackerNode::publishURDF, this);

    this->_state_publisher_urdf = this->_measurements_node_handle.advertise<std_msgs::String>(this->_namespace + "/state_urdf", 100);

    this->_state_publisher_joint_states = this->_measurements_node_handle.advertise<sensor_msgs::JointState>(this->_namespace + "/joint_states", 100);

    // This is used to build URDF models that link to the reconstructed shapes
    this->_sr_path = ros::package::getPath("shape_reconstruction");

    _grasping_type_publisher = this->_measurements_node_handle.advertise<std_msgs::Float64MultiArray>(this->_namespace + "/grasping_type", 100);

    if(_robot_interaction)
    {
        this->_measurement_subscriber_cp = this->_measurements_node_handle.subscribe( "/ee2cp/measurement", 1, &MultiJointTrackerNode::measurementEE2CPCallback, this);
        this->_slippage_detector_cp = this->_measurements_node_handle.subscribe( "/ee2cp/slippage_detected", 1, &MultiJointTrackerNode::slippageDetectedCallback, this);
    }
}

MultiJointTrackerNode::~MultiJointTrackerNode()
{
}

void MultiJointTrackerNode::ReadParameters()
{
    int ks_analysis_type;
    this->getROSParameter<int>(this->_namespace + std::string("/ks_analysis_type"), ks_analysis_type);
    double disconnected_j_ne;
    this->getROSParameter<double>(this->_namespace + std::string("/disconnected_ne"), disconnected_j_ne);
    int likelihood_sample_num;
    this->getROSParameter<int>(this->_namespace + std::string("/likelihood_sample_num"), likelihood_sample_num);
    double sigma_delta_meas_uncertainty_linear;
    this->getROSParameter<double>(this->_namespace + std::string("/sigma_delta_meas_uncertainty_linear"), sigma_delta_meas_uncertainty_linear);
    double sigma_delta_meas_uncertainty_angular;
    this->getROSParameter<double>(this->_namespace + std::string("/sigma_delta_meas_uncertainty_angular"), sigma_delta_meas_uncertainty_angular);

    double prism_prior_cov_vel;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_prior_cov_vel"), prism_prior_cov_vel);
    double prism_sigma_sys_noise_phi;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_sigma_sys_noise_phi"), prism_sigma_sys_noise_phi);
    double prism_sigma_sys_noise_theta;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_sigma_sys_noise_theta"), prism_sigma_sys_noise_theta);
    double prism_sigma_sys_noise_pv;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_sigma_sys_noise_pv"), prism_sigma_sys_noise_pv);
    double prism_sigma_sys_noise_pvd;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_sigma_sys_noise_pvd"), prism_sigma_sys_noise_pvd);
    double prism_sigma_meas_noise;
    this->getROSParameter<double>(this->_namespace + std::string("/prism_sigma_meas_noise"), prism_sigma_meas_noise);

    double rev_prior_cov_vel;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_prior_cov_vel"), rev_prior_cov_vel);
    double rev_sigma_sys_noise_phi;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_phi"), rev_sigma_sys_noise_phi);
    double rev_sigma_sys_noise_theta;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_theta"), rev_sigma_sys_noise_theta);
    double rev_sigma_sys_noise_px;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_px"), rev_sigma_sys_noise_px);
    double rev_sigma_sys_noise_py;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_py"), rev_sigma_sys_noise_py);
    double rev_sigma_sys_noise_pz;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_pz"), rev_sigma_sys_noise_pz);
    double rev_sigma_sys_noise_rv;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_rv"), rev_sigma_sys_noise_rv);
    double rev_sigma_sys_noise_rvd;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_sys_noise_rvd"), rev_sigma_sys_noise_rvd);
    double rev_sigma_meas_noise;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_sigma_meas_noise"), rev_sigma_meas_noise);
    double rev_min_rot_for_ee;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_min_rot_for_ee"), rev_min_rot_for_ee);
    double rev_max_joint_distance_for_ee;
    this->getROSParameter<double>(this->_namespace + std::string("/rev_max_joint_distance_for_ee"), rev_max_joint_distance_for_ee);

    double rig_max_translation;
    this->getROSParameter<double>(this->_namespace + std::string("/rig_max_translation"), rig_max_translation);
    double rig_max_rotation;
    this->getROSParameter<double>(this->_namespace + std::string("/rig_max_rotation"), rig_max_rotation);

    this->getROSParameter<int>(this->_namespace + std::string("/min_joint_age_for_ee"), this->_min_joint_age_for_ee);

    this->getROSParameter<double>(std::string("/omip/sensor_fps"), this->_sensor_fps);
    this->getROSParameter<int>(std::string("/omip/processing_factor"), this->_processing_factor);
    this->_loop_period_ns = 1e9/(this->_sensor_fps/(double)this->_processing_factor);

    int min_num_frames_for_new_rb;
    this->getROSParameter<int>(std::string("/rb_tracker/min_num_frames_for_new_rb"), min_num_frames_for_new_rb);

    this->getROSParameter<bool>(std::string("/omip/robot_interaction"), _robot_interaction);
    int type_of_grasp;
    this->getROSParameter<int>(this->_namespace + std::string("/type_of_grasp"), type_of_grasp);
    this->getROSParameter<std::string>(this->_namespace + std::string("/ft_topic"), _ft_topic);

    ROS_INFO_STREAM_NAMED( "MultiJointTrackerNode.ReadParameters", "MultiJointTrackerNode Parameters: " << std::endl << //
                           "\tType of kinematic analysis: " << ks_analysis_type << std::endl << //
                           "\tDisconnected joint normalized error (joint classification threshold): " << disconnected_j_ne << std::endl << //
                           "\tNumber of samples to estimate likelihood: " << likelihood_sample_num << std::endl << //
                           "\tSigma of the delta of measurements (linear): " << sigma_delta_meas_uncertainty_linear << std::endl << //
                           "\tSigma of the delta of measurements (angular): " << sigma_delta_meas_uncertainty_angular << std::endl <<
                           "\tSigma of the prior of the velocity prismatic joint: " << prism_prior_cov_vel << std::endl <<
                           "\tSigma of the system noise phi prismatic joint: " << prism_sigma_sys_noise_phi << std::endl <<
                           "\tSigma of the system noise theta prismatic joint: " << prism_sigma_sys_noise_theta << std::endl <<
                           "\tSigma of the system noise pv prismatic joint: " << prism_sigma_sys_noise_pv << std::endl <<
                           "\tSigma of the system noise pvd prismatic joint: " << prism_sigma_sys_noise_pvd << std::endl <<
                           "\tSigma of the measurement noise prismatic joint: " << prism_sigma_meas_noise << std::endl <<
                           "\tSigma of the prior of the velocity revolute joint: " << rev_prior_cov_vel << std::endl <<
                           "\tSigma of the system noise phi revolute joint: " << rev_sigma_sys_noise_phi << std::endl <<
                           "\tSigma of the system noise theta revolute joint: " << rev_sigma_sys_noise_theta << std::endl <<
                           "\tSigma of the system noise px revolute joint: " << rev_sigma_sys_noise_px << std::endl <<
                           "\tSigma of the system noise py revolute joint: " << rev_sigma_sys_noise_py << std::endl <<
                           "\tSigma of the system noise pz revolute joint: " << rev_sigma_sys_noise_pz << std::endl <<
                           "\tSigma of the system noise rv revolute joint: " << rev_sigma_sys_noise_rv << std::endl <<
                           "\tSigma of the system noise rvd revolute joint: " << rev_sigma_sys_noise_rvd << std::endl <<
                           "\tSigma of the measurement noise revolute joint: " << rev_sigma_meas_noise << std::endl <<
                           "\tMax translation to reject the rigid joint hyp: " << rig_max_translation << std::endl <<
                           "\tMax rotation to reject the rigid joint hyp: " << rig_max_rotation << std::endl <<
                           "\tMinimum joint age for ee: " << _min_joint_age_for_ee <<
                           "\tMinimum num frames for new rb: " << min_num_frames_for_new_rb <<
                           "\n\trobot_interaction: " << _robot_interaction <<
                           "\n\ttype_of_grasp: " << type_of_grasp <<
                           "\n\ttopic of the ft measurements: " << _ft_topic
                           );

    this->_re_filter = new MultiJointTracker(this->_loop_period_ns, (ks_analysis_t)ks_analysis_type, disconnected_j_ne);
    this->_re_filter->setNumSamplesLikelihoodEstimation(likelihood_sample_num);
    this->_re_filter->setSigmaDeltaMeasurementUncertaintyLinear(sigma_delta_meas_uncertainty_linear);
    this->_re_filter->setSigmaDeltaMeasurementUncertaintyAngular(sigma_delta_meas_uncertainty_angular);

    this->_re_filter->setPrismaticPriorCovarianceVelocity(prism_prior_cov_vel);
    this->_re_filter->setPrismaticSigmaSystemNoisePhi(prism_sigma_sys_noise_phi);
    this->_re_filter->setPrismaticSigmaSystemNoiseTheta(prism_sigma_sys_noise_theta);
    this->_re_filter->setPrismaticSigmaSystemNoisePV(prism_sigma_sys_noise_pv);
    this->_re_filter->setPrismaticSigmaSystemNoisePVd(prism_sigma_sys_noise_pvd);
    this->_re_filter->setPrismaticSigmaMeasurementNoise(prism_sigma_meas_noise);

    this->_re_filter->setRevolutePriorCovarianceVelocity(rev_prior_cov_vel);
    this->_re_filter->setRevoluteSigmaSystemNoisePhi(rev_sigma_sys_noise_phi);
    this->_re_filter->setRevoluteSigmaSystemNoiseTheta(rev_sigma_sys_noise_theta);
    this->_re_filter->setRevoluteSigmaSystemNoisePx(rev_sigma_sys_noise_px);
    this->_re_filter->setRevoluteSigmaSystemNoisePy(rev_sigma_sys_noise_py);
    this->_re_filter->setRevoluteSigmaSystemNoisePz(rev_sigma_sys_noise_pz);
    this->_re_filter->setRevoluteSigmaSystemNoiseRV(rev_sigma_sys_noise_rv);
    this->_re_filter->setRevoluteSigmaSystemNoiseRVd(rev_sigma_sys_noise_rvd);
    this->_re_filter->setRevoluteSigmaMeasurementNoise(rev_sigma_meas_noise);

    this->_re_filter->setRevoluteMinimumRotForEstimation(rev_min_rot_for_ee);
    this->_re_filter->setRevoluteMaximumJointDistanceForEstimation(rev_max_joint_distance_for_ee);

    this->_re_filter->setRigidMaxTranslation(rig_max_translation);
    this->_re_filter->setRigidMaxRotation(rig_max_rotation);

    this->_re_filter->setMinimumJointAgeForEE(this->_min_joint_age_for_ee);

    this->_re_filter->setMinimumNumFramesForNewRB(min_num_frames_for_new_rb);

    this->_re_filter->setRobotInteraction(_robot_interaction);
    this->_re_filter->setTypeOfGrasp(type_of_grasp);
}

void MultiJointTrackerNode::measurementFTCallback(const std_msgs::Float64MultiArrayConstPtr &ft_values)
{
    if(!_ft_values_being_used)
    {
        this->_last_ft_values = ft_values->data;
    }
}

void MultiJointTrackerNode::measurementEE2CPCallback(const std_msgs::Float64MultiArrayConstPtr &rel_pose)
{
    this->_last_ee2cp_relpose = rel_pose->data;
}

void MultiJointTrackerNode::slippageDetectedCallback(const std_msgs::Int32ConstPtr &msg)
{
    this->_re_filter->slippageDetected(msg->data);
}

void MultiJointTrackerNode::measurementCallback(const boost::shared_ptr<ks_measurement_ros_t const> &poses_and_vels)
{   
    ros::Time tinit = ros::Time::now();

    // Get the time of the measurement as reference time
    this->_current_measurement_time = poses_and_vels->header.stamp;

    this->_re_filter->setMeasurement(*poses_and_vels, (double)poses_and_vels->header.stamp.toNSec());

    if(_robot_interaction)
    {
        this->_ft_values_being_used = true;
        this->_re_filter->setMeasurementFT(this->_last_ft_values, 0);
        this->_re_filter->setMeasurementEE2CP(this->_last_ee2cp_relpose, 0);
        this->_ft_values_being_used = false;
    }

    // Measure the time interval between the previous measurement and the current measurement
    ros::Duration time_between_meas = this->_current_measurement_time - this->_previous_measurement_time;

    // The time interval between frames is the inverse of the max_framerate
    double period_between_frames = 1.0/this->_sensor_fps;

    // The number of frames elapsed is the time elapsed divided by the period between frames
    int frames_between_meas = round((time_between_meas.toSec())/period_between_frames);

    // Processing fps: ignore (this->_processing_factor - 1) frames and process one. This gives effectively the desired processing fps: _sensor_fps/_processing_factor
    if( this->_previous_measurement_time.toSec() == 0.)
    {
        ROS_ERROR("Initial prediction of state and measurements (in KS estimation)!");

        // In the first iteration we clear the markers of the previous run (if the user wants that)
        bool clear_rviz_markers;
        this->getROSParameter<bool>(this->_namespace + std::string("/clear_rviz_markers"), clear_rviz_markers);
        if(clear_rviz_markers)
        {
            ROS_INFO_STREAM("Clearing RVIZ markers");
            // Clear all previous markers
            // Create a map of the most probable joints
            visualization_msgs::MarkerArray cleaning_markers;
            for (int num_rbs=0; num_rbs<100; num_rbs++)
            {
                for(int num_joint_markers=0; num_joint_markers < 3; num_joint_markers++)
                {
                    visualization_msgs::Marker cleaning_joint_marker;
                    cleaning_joint_marker.ns = "kinematic_structure";
                    cleaning_joint_marker.action = visualization_msgs::Marker::DELETE;
                    cleaning_joint_marker.type = visualization_msgs::Marker::ARROW;

                    cleaning_joint_marker.header.stamp = ros::Time::now();
                    cleaning_joint_marker.id = 3*num_rbs + num_joint_markers;

                    cleaning_markers.markers.push_back(cleaning_joint_marker);

                    cleaning_joint_marker.ns = "kinematic_structure_uncertainty";
                    cleaning_markers.markers.push_back(cleaning_joint_marker);
                }
            }
            this->_state_publisher_rviz_markers.publish(cleaning_markers);
        }
    }
    else if( frames_between_meas != this->_processing_factor)
    {
        ROS_ERROR("Lost frames:%3d. Need to predict state and measurement again.", frames_between_meas - this->_processing_factor);

        // Predict next RE state
        this->_re_filter->predictState((double)time_between_meas.toNSec());

        // Predict next measurement based on the predicted state
        this->_re_filter->predictMeasurement();
    }else{
        ROS_INFO("Frames between measurements:%3d.", frames_between_meas);
    }

    // In the first iteration we cannot correct the state because we have only one measurement
    if(this->_previous_measurement_time.toSec() != 0.)
    {
        // Use the predicted measurement and the received measurement to correct the state
        this->_re_filter->correctState();
    }

    this->_re_filter->estimateJointFiltersProbabilities();


    this->_publishState();
    this->_PrintResults();

    this->_re_filter->predictState(this->_loop_period_ns);
    this->_re_filter->predictMeasurement();

    this->_publishPredictedMeasurement();

    this->_previous_measurement_time = this->_current_measurement_time;
    ros::Time tend = ros::Time::now();
    ROS_WARN_STREAM("Time between meas: " << time_between_meas.toSec()*1000 << " ms");
    ROS_WARN_STREAM("Total meas processing time: " << (tend-tinit).toSec()*1000 << " ms");
}

bool MultiJointTrackerNode::publishURDF(joint_tracker::publish_urdf::Request& request, joint_tracker::publish_urdf::Response& response)
{
    std_msgs::String urdf_string_msg;
    sensor_msgs::JointState joint_states_msg;
    this->_generateURDF(urdf_string_msg, joint_states_msg);
    response.urdf_str = urdf_string_msg.data;


    if(request.write_to_file)
    {
        struct stat buffer2;
        struct stat buffer3;
        std::string urdf_file_path_dir = ros::package::getPath("joint_tracker");
        std::string urdf_file_path_old = urdf_file_path_dir + std::string("/urdf/km1.urdf");
        std::string urdf_file_path_new = urdf_file_path_dir + std::string("/urdf/km2.urdf");


        if (stat (urdf_file_path_old.c_str(), &buffer2) == 0)
        {
            if (stat (urdf_file_path_new.c_str(), &buffer3) == 0)
            {
                std::remove(urdf_file_path_new.c_str());
            }
            std::rename(urdf_file_path_old.c_str(), urdf_file_path_new.c_str());
        }
        std::ofstream new_urdf_file(urdf_file_path_old.c_str());
        new_urdf_file << response.urdf_str;
        new_urdf_file.close();
    }

    return true;
}

void MultiJointTrackerNode::_generateURDF(std_msgs::String& urdf_string_msg, sensor_msgs::JointState& joint_state_msg) const
{
    std::ostringstream urdf_stream;
    int RB_id_first;
    int RB_id_second;
    KinematicModel kinematic_model_original = this->_re_filter->getState();

    typedef std::map<std::pair<int, int>, JointFilterPtr> map_type_temp;
    map_type_temp kinematic_model;
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::const_iterator joint_combined_filters_it = kinematic_model_original.begin();
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::const_iterator joint_combined_filters_it_end = kinematic_model_original.end();
    for (; joint_combined_filters_it != joint_combined_filters_it_end; joint_combined_filters_it++)
    {
            if(!(_robot_interaction && joint_combined_filters_it->first.first == DEFORMED_END_EFFECTOR_FILTER_ID && joint_combined_filters_it->first.second == INTERACTED_RB_FILTER_ID)
                    && !(_robot_interaction && joint_combined_filters_it->first.first == MULTIMODAL_EE_FILTER_ID && joint_combined_filters_it->first.second ==DEFORMED_END_EFFECTOR_FILTER_ID ))
        {

            JointFilterPtr joint_hypothesis = joint_combined_filters_it->second->getMostProbableJointFilter();
            kinematic_model[joint_combined_filters_it->first] = joint_hypothesis;
        }
    }


    urdf_stream << "<?xml version=\"1.0\"?>" << std::endl;
    urdf_stream << "<robot name=\"KinematicStructure\">" << std::endl;

    // First add the joints
    // We need to add first the joints because this gives us the transformations we need to apply to the link visual's to bring them back to the origin
    bool prism_or_rev = false;
    std::map<int, std::pair<Eigen::Vector3d, Eigen::Vector3d> > link_visuals_origin;
    double joint_state = 0.;
    double joint_velocity = 0.;
    Eigen::Vector3d joint_position(0.,0.,0.);
    Eigen::Vector3d joint_orientation_rpy(0.,0.,0.);
    Eigen::Vector3d joint_orientation_axis(0.,0.,0.);
    BOOST_FOREACH(map_type_temp::value_type km_it, kinematic_model)
    {
        prism_or_rev = false;
        RB_id_first = km_it.first.first;
        RB_id_second = km_it.first.second;
        joint_state = km_it.second->getJointState();
        joint_velocity = km_it.second->getJointVelocity();
        joint_position = km_it.second->getJointPositionInRRBFrame();
        joint_orientation_rpy = km_it.second->getJointOrientationRPYInRRBFrame();
        joint_orientation_axis = km_it.second->getJointOrientationUnitaryVector();

        std::ostringstream joint_unique_name;

        joint_unique_name << "joint" << km_it.second->getJointId();

        urdf_stream << "  <joint name=\"" << joint_unique_name.str() << "\" type=\"";

        switch(km_it.second->getJointFilterType())
        {
        case RIGID_JOINT:
            urdf_stream << "fixed\">" << std::endl;
            break;
        case PRISMATIC_JOINT:
            prism_or_rev = true;
            urdf_stream << "prismatic\">" << std::endl;
            joint_state_msg.name.push_back(joint_unique_name.str());
            joint_state_msg.position.push_back(joint_state);
            joint_state_msg.velocity.push_back(joint_velocity);
            joint_state_msg.effort.push_back(0.0);
            break;
        case REVOLUTE_JOINT:
            prism_or_rev = true;
            urdf_stream << "revolute\">" << std::endl;
            joint_state_msg.name.push_back(joint_unique_name.str());
            joint_state_msg.position.push_back(joint_state);
            joint_state_msg.velocity.push_back(joint_velocity);
            joint_state_msg.effort.push_back(0.0);
            break;
        case DISCONNECTED_JOINT:
            urdf_stream << "floating\">" << std::endl;
            break;
        default:
            std::cout << "ERROR deparsing the results into a urdf string. Joint type not defined" << std::endl;
            return;
            break;
        }

        // Set the parent link
        if(RB_id_first == 0)
        {
            urdf_stream << "    <parent link=\"static_environment\"/>" << std::endl;
            link_visuals_origin[RB_id_first] = std::pair<Eigen::Vector3d,Eigen::Vector3d>( Eigen::Vector3d(0.,0.,0.), Eigen::Vector3d(0.,0.,0.));
        }
        else{
            urdf_stream << "    <parent link=\"rb" << RB_id_first << "\"/>"<< std::endl;
        }

        // Set the child link
        urdf_stream << "    <child link=\"rb" << RB_id_second << "\"/>"<< std::endl;

        // Set the properties of the link for revolute and prismatic joints
        // We define the joint frame using the parameters of the joint
        // The joint axis is always a unitary vector pointing in the z direction
        if(prism_or_rev)
        {
            urdf_stream << "    <origin xyz=\"" << joint_position.x() << " " << joint_position.y() << " "<< joint_position.z() <<
                           "\" rpy=\"" << 0.0 << " "<< 0.0 << " "<< 0.0 << "\"/>" << std::endl;
            urdf_stream << "    <axis xyz=\"" << joint_orientation_axis.x() << " " << joint_orientation_axis.y() << " " << joint_orientation_axis.z() << "\"/>" << std::endl;
            link_visuals_origin[RB_id_second] = std::pair<Eigen::Vector3d,Eigen::Vector3d>(-joint_position, Eigen::Vector3d(0.,0.,0.));


            // With rotation:
            //            urdf_stream << "    <origin xyz=\"" << joint_position.x() << " " << joint_position.y() << " "<< joint_position.z() <<
            //                           "\" rpy=\"" << joint_orientation_rpy.x() << " "<< joint_orientation_rpy.y() << " "<< joint_orientation_rpy.z() << "\"/>" << std::endl;
            //            urdf_stream << "    <axis xyz=\"1 0 0\"/>" << std::endl;



            urdf_stream << "    <limit effort=\"30\" velocity=\"1.0\" lower=\"-3.1416\" upper=\"3.1416\"/>" << std::endl;
        }

        urdf_stream << "  </joint>" << std::endl;
    }

    // We add all the links to the model
    std::vector<omip::RB_id_t> all_rb_ids;
    BOOST_FOREACH(map_type_temp::value_type km_it, kinematic_model)
    {
        RB_id_first = km_it.first.first;
        RB_id_second = km_it.first.second;

        if(std::find(all_rb_ids.begin(), all_rb_ids.end(), RB_id_first) == all_rb_ids.end())
        {
            all_rb_ids.push_back(RB_id_first);
        }

        if(std::find(all_rb_ids.begin(), all_rb_ids.end(), RB_id_second) == all_rb_ids.end())
        {
            all_rb_ids.push_back(RB_id_second);
        }
    }

    std::vector<Eigen::Vector3d > rgb_colors;
    rgb_colors.push_back(Eigen::Vector3d(255,255,255)/255.0); // white
    rgb_colors.push_back(Eigen::Vector3d(255,0,0)/255.0); // red
    rgb_colors.push_back(Eigen::Vector3d(0,0,255)/255.0); // blue
    rgb_colors.push_back(Eigen::Vector3d(0,255,0)/255.0); // green
    rgb_colors.push_back(Eigen::Vector3d(255,255,0)/255.0); // yellow
    rgb_colors.push_back(Eigen::Vector3d(255,0,255)/255.0); // violet
    rgb_colors.push_back(Eigen::Vector3d(0,255,255)/255.0); // turquese
    rgb_colors.push_back(Eigen::Vector3d(255,128,0)/255.0); // orange
    rgb_colors.push_back(Eigen::Vector3d(153,153,255)/255.0); // pink
    rgb_colors.push_back(Eigen::Vector3d(128,128,128)/255.0); // gray


    for(int links_idx = 0; links_idx < all_rb_ids.size(); links_idx++)
    {
        // Static environment
        if(all_rb_ids.at(links_idx) == 0)
        {
            urdf_stream << "  <link name=\"static_environment\"";
        }
        else{
            urdf_stream << "  <link name=\"rb" << all_rb_ids.at(links_idx) << "\"";
        }

        std::string sr_path = this->_sr_path;

        sr_path += std::string("/meshes/");
        std::stringstream name_ss;

        if(all_rb_ids.at(links_idx) == 0)
        {
            name_ss << "static_env.stl";
        }else{
            name_ss << "shape_rb" << all_rb_ids.at(links_idx) << ".stl";
        }

        sr_path += name_ss.str();

        std::string sr_path_urdf = std::string("          <mesh filename=\"package://shape_reconstruction/meshes/");
        sr_path_urdf += name_ss.str();

        // We check if there exists a mesh file for this link and if it exists we add it
        struct stat buffer;

        if (stat (sr_path.c_str(), &buffer) == 0)
        {
            urdf_stream << ">" << std::endl;
            urdf_stream << "    <visual>" << std::endl <<
                           // "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\" />" << std::endl <<
                           "      <origin xyz=\"" << link_visuals_origin[all_rb_ids.at(links_idx)].first.x() << " "
                        << link_visuals_origin[all_rb_ids.at(links_idx)].first.y() << " "
                        << link_visuals_origin[all_rb_ids.at(links_idx)].first.z() << "\" rpy=\""
                        << link_visuals_origin[all_rb_ids.at(links_idx)].second.x() << " "
                        << link_visuals_origin[all_rb_ids.at(links_idx)].second.y() << " "
                        << link_visuals_origin[all_rb_ids.at(links_idx)].second.z() << "\"/>" << std::endl <<
                           "      <geometry>" << std::endl <<
                           sr_path_urdf << "\"/>"<< std::endl <<
                           "      </geometry>" << std::endl <<
                           "      <material name=\"" << "shape_rb" << all_rb_ids.at(links_idx) << "_color\">" << std::endl <<
                           "        <color rgba=\"" << rgb_colors.at(all_rb_ids.at(links_idx)%rgb_colors.size()).x() << " "<<
                           rgb_colors.at(all_rb_ids.at(links_idx)%rgb_colors.size()).y() << " "<<
                           rgb_colors.at(all_rb_ids.at(links_idx)%rgb_colors.size()).z() << " 1\"/>" << std::endl <<
                           "      </material>" << std::endl<<
                           "    </visual>"<<std::endl <<
                           "  </link>" << std::endl;
        }else
        {
            urdf_stream << "/>" << std::endl;
        }
    }

    urdf_stream << "</robot>" << std::endl;

    urdf_string_msg.data = urdf_stream.str();

    joint_state_msg.header.stamp = ros::Time::now();
}

void MultiJointTrackerNode::_publishState() const
{
    visualization_msgs::MarkerArray cleaning_markers;
    for (int num_rbs=0; num_rbs<15; num_rbs++)
    {
        for(int num_joint_markers=0; num_joint_markers < 3; num_joint_markers++)
        {
            visualization_msgs::Marker cleaning_joint_marker;
            cleaning_joint_marker.ns = "kinematic_structure";
            cleaning_joint_marker.action = visualization_msgs::Marker::DELETE;
            cleaning_joint_marker.type = visualization_msgs::Marker::ARROW;

            cleaning_joint_marker.header.stamp = ros::Time::now();
            cleaning_joint_marker.id = 3*num_rbs + num_joint_markers;

            cleaning_markers.markers.push_back(cleaning_joint_marker);

            cleaning_joint_marker.ns = "kinematic_structure_uncertainty";
            cleaning_markers.markers.push_back(cleaning_joint_marker);
        }
    }
    this->_state_publisher_rviz_markers.publish(cleaning_markers);

    visualization_msgs::MarkerArray markers_of_most_probable_structure;

    ros::Time time_markers = this->_current_measurement_time;
    int RB_id;
    KinematicModel kinematic_model = this->_re_filter->getState();

    omip_msgs::KinematicStructureMsg ks_msg;
    // Populate a ros state message containing the kinematic structure (most probable joint and all others)
    BOOST_FOREACH(KinematicModel::value_type km_it, kinematic_model)
    {
        if(!(_robot_interaction && km_it.first.first == DEFORMED_END_EFFECTOR_FILTER_ID && km_it.first.second == INTERACTED_RB_FILTER_ID)
                && !(_robot_interaction && km_it.first.first == MULTIMODAL_EE_FILTER_ID && km_it.first.second == DEFORMED_END_EFFECTOR_FILTER_ID))
        {
            omip_msgs::JointMsg joint_msg;
            joint_msg.parent_rb_id = km_it.first.first;
            joint_msg.child_rb_id = km_it.first.second;

            joint_msg.rigid_probability = km_it.second->getJointFilter(RIGID_JOINT)->getProbabilityOfJointFilter();
            joint_msg.discon_probability = km_it.second->getJointFilter(DISCONNECTED_JOINT)->getProbabilityOfJointFilter();
            joint_msg.rev_probability = km_it.second->getJointFilter(REVOLUTE_JOINT)->getProbabilityOfJointFilter();
            joint_msg.prism_probability = km_it.second->getJointFilter(PRISMATIC_JOINT)->getProbabilityOfJointFilter();

            Eigen::Vector3d temp_eigen = km_it.second->getJointFilter(PRISMATIC_JOINT)->getJointPositionInRRBFrame();
            geometry_msgs::Point temp_ros;
            temp_ros.x = temp_eigen.x();
            temp_ros.y = temp_eigen.y();
            temp_ros.z = temp_eigen.z();
            joint_msg.prism_position = temp_ros;

            geometry_msgs::Vector3 temp_ros_vect;
            temp_eigen = km_it.second->getJointFilter(PRISMATIC_JOINT)->getJointOrientationUnitaryVector();
            temp_ros_vect.x = temp_eigen.x();
            temp_ros_vect.y = temp_eigen.y();
            temp_ros_vect.z = temp_eigen.z();
            joint_msg.prism_orientation = temp_ros_vect;

            joint_msg.prism_ori_phi = km_it.second->getJointFilter(PRISMATIC_JOINT)->getOrientationPhiInRRBFrame();
            joint_msg.prism_ori_theta = km_it.second->getJointFilter(PRISMATIC_JOINT)->getOrientationThetaInRRBFrame();

            joint_msg.prism_ori_cov[0] = km_it.second->getJointFilter(PRISMATIC_JOINT)->getCovarianceOrientationPhiPhiInRRBFrame();
            joint_msg.prism_ori_cov[1] = km_it.second->getJointFilter(PRISMATIC_JOINT)->getCovarianceOrientationPhiThetaInRRBFrame();
            joint_msg.prism_ori_cov[2] = km_it.second->getJointFilter(PRISMATIC_JOINT)->getCovarianceOrientationPhiThetaInRRBFrame();
            joint_msg.prism_ori_cov[3] = km_it.second->getJointFilter(PRISMATIC_JOINT)->getCovarianceOrientationThetaThetaInRRBFrame();

            joint_msg.prism_joint_value = km_it.second->getJointFilter(PRISMATIC_JOINT)->getJointState();

            temp_eigen = km_it.second->getJointFilter(REVOLUTE_JOINT)->getJointPositionInRRBFrame();
            temp_ros.x = temp_eigen.x();
            temp_ros.y = temp_eigen.y();
            temp_ros.z = temp_eigen.z();
            joint_msg.rev_position = temp_ros;

            Eigen::Matrix3d posi_unc = km_it.second->getJointFilter(REVOLUTE_JOINT)->getCovariancePositionXYZInRRBFrame();

            for(int i =0; i<3; i++)
            {
                for(int j=0; j<3; j++)
                {
                    joint_msg.rev_position_uncertainty[3*j + i] =  posi_unc(i, j);
                }
            }

            temp_eigen = km_it.second->getJointFilter(REVOLUTE_JOINT)->getJointOrientationUnitaryVector();
            temp_ros_vect.x = temp_eigen.x();
            temp_ros_vect.y = temp_eigen.y();
            temp_ros_vect.z = temp_eigen.z();
            joint_msg.rev_orientation = temp_ros_vect;

            joint_msg.rev_ori_phi = km_it.second->getJointFilter(REVOLUTE_JOINT)->getOrientationPhiInRRBFrame();
            joint_msg.rev_ori_theta = km_it.second->getJointFilter(REVOLUTE_JOINT)->getOrientationThetaInRRBFrame();

            joint_msg.rev_ori_cov[0] = km_it.second->getJointFilter(REVOLUTE_JOINT)->getCovarianceOrientationPhiPhiInRRBFrame();
            joint_msg.rev_ori_cov[1] = km_it.second->getJointFilter(REVOLUTE_JOINT)->getCovarianceOrientationPhiThetaInRRBFrame();
            joint_msg.rev_ori_cov[2] = km_it.second->getJointFilter(REVOLUTE_JOINT)->getCovarianceOrientationPhiThetaInRRBFrame();
            joint_msg.rev_ori_cov[3] = km_it.second->getJointFilter(REVOLUTE_JOINT)->getCovarianceOrientationThetaThetaInRRBFrame();

            joint_msg.rev_joint_value = km_it.second->getJointFilter(REVOLUTE_JOINT)->getJointState();

            joint_msg.most_likely_joint =  (int)km_it.second->getMostProbableJointFilter()->getJointFilterType();

            ks_msg.kinematic_structure.push_back(joint_msg);
        }
    }
    ks_msg.header.stamp = time_markers;
    this->_state_publisher.publish(ks_msg);

    // Create a map of the most probable joints
    std::map<std::pair<int, int>, boost::shared_ptr<JointFilter> > most_probable_joints;
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::const_iterator joint_combined_filters_it = kinematic_model.begin();
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::const_iterator joint_combined_filters_it_end = kinematic_model.end();
    for (; joint_combined_filters_it != joint_combined_filters_it_end; joint_combined_filters_it++)
    {
        if(!(_robot_interaction && joint_combined_filters_it->first.first == MULTIMODAL_EE_FILTER_ID && joint_combined_filters_it->first.second == DEFORMED_END_EFFECTOR_FILTER_ID))
        {

            JointFilterPtr joint_hypothesis = joint_combined_filters_it->second->getMostProbableJointFilter();
            most_probable_joints[joint_combined_filters_it->first] = joint_hypothesis;

            RB_id = joint_combined_filters_it->first.first;
            std::ostringstream oss;

            // Static environment
            if(RB_id == 0)
            {
                oss << "static_environment";
            }
            else{
                oss << "ip/rb" << RB_id;
            }
            std::vector<visualization_msgs::Marker> joint_markers = joint_hypothesis->getJointMarkersInRRBFrame();
            for (size_t markers_idx = 0; markers_idx < joint_markers.size(); markers_idx++)
            {
                joint_markers.at(markers_idx).header.stamp = time_markers;
                joint_markers.at(markers_idx).header.frame_id = oss.str();

                markers_of_most_probable_structure.markers.push_back(joint_markers.at(markers_idx));
            }
        }
    }
    this->_state_publisher_rviz_markers.publish(markers_of_most_probable_structure);

    std_msgs::String urdf_string_msg;
    sensor_msgs::JointState joint_states_msg;
    this->_generateURDF(urdf_string_msg, joint_states_msg);

    this->_state_publisher_urdf.publish(urdf_string_msg);
    this->_state_publisher_joint_states.publish(joint_states_msg);
}

void MultiJointTrackerNode::_publishPredictedMeasurement() const
{
    omip_msgs::RigidBodyPosesAndVelsMsg hypotheses = this->_re_filter->getPredictedMeasurement();
    hypotheses.header.frame_id = "/camera_rgb_optical_frame";
    hypotheses.header.stamp = this->_current_measurement_time + ros::Duration(this->_loop_period_ns/1e9);
    this->_measurement_prediction_publisher.publish(hypotheses);
}

void MultiJointTrackerNode::_PrintResults() const
{
    KinematicModel kinematic_model = this->_re_filter->getState();
    int combined_filter_idx = 0;
    BOOST_FOREACH(KinematicModel::value_type km_it, kinematic_model)
    {
        JointCombinedFilterPtr cf_ptr = this->_re_filter->getCombinedFilter(combined_filter_idx);
        JointFilterPtr joint_hypothesis = km_it.second->getMostProbableJointFilter();
        if(!(_robot_interaction && km_it.first.first == DEFORMED_END_EFFECTOR_FILTER_ID && km_it.first.second == INTERACTED_RB_FILTER_ID)
                && !(_robot_interaction && km_it.first.first == MULTIMODAL_EE_FILTER_ID && km_it.first.second == DEFORMED_END_EFFECTOR_FILTER_ID))
        {
            ROS_INFO_NAMED( "MultiJointTrackerNode::PrintAndPublishResults",
                            "Joint between RB%3d and RB%3d  (JointID =%3d) of type %s (P=%2.2f, Rev=%2.2f, Rig=%2.2f, D=%2.2f)",
                            (int)km_it.first.first,
                            (int)km_it.first.second,
                            (int)km_it.second->getJointCombinedFilterId(),
                            joint_hypothesis->getJointFilterTypeStr().c_str(),
                            cf_ptr->getJointFilter(PRISMATIC_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(REVOLUTE_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(RIGID_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(DISCONNECTED_JOINT)->getProbabilityOfJointFilter());
        }else if (_robot_interaction && km_it.first.first == DEFORMED_END_EFFECTOR_FILTER_ID && km_it.first.second == INTERACTED_RB_FILTER_ID){
            ROS_INFO_NAMED( "MultiJointTrackerNode::PrintAndPublishResults",
                            "Joint between RB%3d (deformed end-effector) and RB%3d (interacted body)  (JointID =%3d) of type %s (PG=%2.2f, URY=%2.2f, URTY=%2.2f, D=%2.2f)",
                            (int)km_it.first.first,
                            (int)km_it.first.second,
                            (int)km_it.second->getJointCombinedFilterId(),
                            joint_hypothesis->getJointFilterTypeStr().c_str(),
                            cf_ptr->getJointFilter(PERFECT_GRASP_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(UNCONSTRAINED_RY_GRASP_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(UNCONSTRAINED_RY_TY_GRASP_JOINT)->getProbabilityOfJointFilter(),
                            cf_ptr->getJointFilter(DISCONNECTED_JOINT)->getProbabilityOfJointFilter());
            std_msgs::Float64MultiArray grasping_types;
            grasping_types.data.push_back(cf_ptr->getJointFilter(PERFECT_GRASP_JOINT)->getProbabilityOfJointFilter());
            grasping_types.data.push_back(cf_ptr->getJointFilter(UNCONSTRAINED_RY_GRASP_JOINT)->getProbabilityOfJointFilter());
            grasping_types.data.push_back(cf_ptr->getJointFilter(UNCONSTRAINED_RY_TY_GRASP_JOINT)->getProbabilityOfJointFilter());
            grasping_types.data.push_back(cf_ptr->getJointFilter(DISCONNECTED_JOINT)->getProbabilityOfJointFilter());
            _grasping_type_publisher.publish(grasping_types);
        }else if (_robot_interaction && km_it.first.first == MULTIMODAL_EE_FILTER_ID && km_it.first.second == DEFORMED_END_EFFECTOR_FILTER_ID )
        {
            ROS_INFO_NAMED( "MultiJointTrackerNode::PrintAndPublishResults",
                            "Joint between RB%3d (end effector) and RB%3d (deformed end-effector)  (JointID =%3d) of type DeformationFilter",
                            (int)km_it.first.first,
                            (int)km_it.first.second,
                            (int)km_it.second->getJointCombinedFilterId()
                            );
        }
        combined_filter_idx++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_tracker");
    MultiJointTrackerNode ks_tracker_node;
    ks_tracker_node.run();

    return (0);
}
