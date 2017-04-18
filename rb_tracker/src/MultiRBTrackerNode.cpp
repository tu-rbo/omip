#include <vector>
#include <stdio.h>
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rosbag/message_instance.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <sensor_msgs/Range.h>

#include "rb_tracker/MultiRBTrackerNode.h"
#include "rb_tracker/RBFilter.h"

#include "omip_common/OMIPUtils.h"

#include <pcl_conversions/pcl_conversions.h>

#if ROS_VERSION_MINIMUM(1, 11, 1) // if current ros version is >= 1.11.1 (indigo)
#else
#include "rb_tracker/pcl_conversions_indigo.h"
#endif

#define LINES_OF_TERMINAL_RBT 20

using namespace omip;

#define MAX_DELAY_BETWEEN_FT_AND_ST 0.02

MultiRBTrackerNode::MultiRBTrackerNode() :
    RecursiveEstimatorNodeInterface(1),
    _loop_period_ns(0.),
    _ransac_iterations(-1),
    _estimation_error_threshold(-1.),
    _static_motion_threshold(-1.),
    _new_rbm_error_threshold(-1.),
    _max_error_to_reassign_feats(-1.),
    _sensor_fps(0.0),
    _processing_factor(0),
    _supporting_features_threshold(-1),
    _min_num_feats_for_new_rb(-1),
    _min_num_frames_for_new_rb(-1),
    _publishing_rbposes_with_cov(false),
    _publishing_clustered_pc(true),
    _publishing_tf(true),
    _printing_rb_poses(true),
    _shape_tracker_received(false),
    _robot_interaction(false)
{
    this->_namespace = std::string("rb_tracker");

    this->getROSParameter<bool>(std::string("/omip/only_proprioception"), _only_proprioception);

    this->ReadParameters();

    this->_previous_measurement_time.fromSec(0.);

    // Setup the callback for the dynamic reconfigure
    this->_dr_callback = boost::bind(&MultiRBTrackerNode::DynamicReconfigureCallback, this, _1, _2);
    this->_dr_srv.setCallback(this->_dr_callback);

    // Setup the subscriber for the measurements from the lower RE level
    this->_measurement_subscriber= this->_measurements_node_handle.subscribe("/feature_tracker/state", 100, &MultiRBTrackerNode::measurementCallback, this);
    // Setup the subscriber for state predictions from the higher RE level
    this->_state_prediction_subscriber = this->_state_prediction_node_handles[0].subscribe(this->_namespace + "/predicted_measurement", 10,
                                                                                           &MultiRBTrackerNode::statePredictionCallback, this);

    this->_meas_from_st_subscriber = this->_state_prediction_node_handles[0].subscribe("/shape_tracker/state", 10,
                                                                                       &MultiRBTrackerNode::MeasurementFromShapeTrackerCallback, this);

    // Setup the publisher for the predicted measurements. They are used as state predictions by the lower RE level
    this->_measurement_prediction_publisher = this->_measurements_node_handle.advertise<ft_state_ros_t>("/feature_tracker/predicted_measurement", 1);
    this->_state_publisher = this->_measurements_node_handle.advertise<omip_msgs::RigidBodyPosesAndVelsMsg>(this->_namespace + "/state", 1);
    this->_state_publisher2 = this->_measurements_node_handle.advertise<omip_msgs::RigidBodyPosesAndVelsMsg>(this->_namespace + "/state_after_feat_correction", 1);

    // Additional published topics
    this->_clustered_pc_publisher = this->_measurements_node_handle.advertise<sensor_msgs::PointCloud2>(this->_namespace + "/clustered_tracked_feats", 100);
    this->_freefeats_pc_publisher = this->_measurements_node_handle.advertise<sensor_msgs::PointCloud2>(this->_namespace + "/free_feats", 100);

    this->_predictedfeats_pc_publisher = this->_measurements_node_handle.advertise<sensor_msgs::PointCloud2>(this->_namespace + "/predicted_feats", 100);
    this->_atbirthfeats_pc_publisher = this->_measurements_node_handle.advertise<sensor_msgs::PointCloud2>(this->_namespace + "/atbirth_feats", 100);

    // Initial constraint in the motion of the camera (initially constrained to no motion)
    int max_num_rb = -1;
    this->getROSParameter<int>(this->_namespace + std::string("/max_num_rb"), max_num_rb, max_num_rb);

    // Initial constraint in the motion of the camera (initially constrained to no motion)
    int initial_cam_motion_constraint = 6;
    this->getROSParameter<int>(this->_namespace + std::string("/cam_motion_constraint"), initial_cam_motion_constraint, initial_cam_motion_constraint);

    // Type of filter for the static environment (EKF based or ICP based)
    int static_environment_tracker_type_int;
    this->getROSParameter<int>(this->_namespace + std::string("/static_environment_type"), static_environment_tracker_type_int);
    this->_static_environment_tracker_type = (static_environment_tracker_t)static_environment_tracker_type_int;

    double prior_cov_pose;
    this->getROSParameter<double>(this->_namespace + std::string("/prior_cov_pose"), prior_cov_pose);
    double prior_cov_vel;
    this->getROSParameter<double>(this->_namespace + std::string("/prior_cov_vel"), prior_cov_vel);
    double cov_sys_acc_tx;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_tx"), cov_sys_acc_tx);
    double cov_sys_acc_ty;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_ty"), cov_sys_acc_ty);
    double cov_sys_acc_tz;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_tz"), cov_sys_acc_tz);
    double cov_sys_acc_rx;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_rx"), cov_sys_acc_rx);
    double cov_sys_acc_ry;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_ry"), cov_sys_acc_ry);
    double cov_sys_acc_rz;
    this->getROSParameter<double>(this->_namespace + std::string("/cov_sys_acc_rz"), cov_sys_acc_rz);

    int min_num_points_in_segment;
    this->getROSParameter<int>(this->_namespace + std::string("/min_num_points_in_segment"), min_num_points_in_segment);
    double min_probabilistic_value;
    this->getROSParameter<double>(this->_namespace + std::string("/min_probabilistic_value"), min_probabilistic_value);
    double max_fitness_score;
    this->getROSParameter<double>(this->_namespace + std::string("/max_fitness_score"), max_fitness_score);

    double min_amount_translation_for_new_rb;
    this->getROSParameter<double>(this->_namespace + std::string("/min_amount_translation_for_new_rb"), min_amount_translation_for_new_rb);
    double min_amount_rotation_for_new_rb;
    this->getROSParameter<double>(this->_namespace + std::string("/min_amount_rotation_for_new_rb"), min_amount_rotation_for_new_rb);

    int min_num_supporting_feats_to_correct;
    this->getROSParameter<int>(this->_namespace + std::string("/min_num_supporting_feats_to_correct"), min_num_supporting_feats_to_correct);

    this->getROSParameter<bool>(std::string("/omip/robot_interaction"), _robot_interaction);
    double max_feat_error_irb;
    this->getROSParameter<double>(this->_namespace + std::string("/max_feat_error_irb"), max_feat_error_irb);
    double max_feat_error_ee;
    this->getROSParameter<double>(this->_namespace + std::string("/max_feat_error_ee"), max_feat_error_ee);
    double max_distance_ee;
    this->getROSParameter<double>(this->_namespace + std::string("/max_distance_ee"), max_distance_ee);
    double max_distance_irb;
    this->getROSParameter<double>(this->_namespace + std::string("/max_distance_irb"), max_distance_irb);

    ROS_INFO_STREAM_NAMED( "MultiRBTrackerNode.ReadParameters",
                           "MultiRBTrackerNode Parameters: " <<
                           "\n\tmax_num_rb: " << max_num_rb <<
                           "\n\tprior_cov_pose: " << prior_cov_pose <<
                           "\n\tprior_cov_vel: " << prior_cov_vel <<
                           "\n\tcov_sys_acc_tx: " << cov_sys_acc_tx <<
                           "\n\tcov_sys_acc_ty: " << cov_sys_acc_ty <<
                           "\n\tcov_sys_acc_tz: " << cov_sys_acc_tz <<
                           "\n\tcov_sys_acc_rx: " << cov_sys_acc_rx <<
                           "\n\tcov_sys_acc_ry: " << cov_sys_acc_ry <<
                           "\n\tcov_sys_acc_rz: " << cov_sys_acc_rz <<
                           "\n\tmin_num_points_in_segment: " << min_num_points_in_segment <<
                           "\n\tmin_probabilistic_value: " << min_probabilistic_value <<
                           "\n\tmax_fitness_score: " << max_fitness_score <<
                           "\n\tmin_amount_translation_for_new_rb: " << min_amount_translation_for_new_rb <<
                           "\n\tmin_amount_rotation_for_new_rb: " << min_amount_rotation_for_new_rb <<
                           "\n\tmin_num_supporting_feats_to_correct: " << min_num_supporting_feats_to_correct <<
                           "\n\tcam_motion_constraint: " << initial_cam_motion_constraint <<
                           "\n\trobot_interaction: " << _robot_interaction <<
                           "\n\tonly_proprioception: " << _only_proprioception <<
                           "\n\tmax_feat_error_irb: " << max_feat_error_irb <<
                           "\n\tmax_feat_error_ee: " << max_feat_error_ee <<
                           "\n\tmax_distance_ee: " << max_distance_ee <<
                           "\n\tmax_distance_irb: " << max_distance_irb );

    // Create the RE filter
    this->_re_filter = new MultiRBTracker(max_num_rb, this->_loop_period_ns,
                                          this->_ransac_iterations,
                                          this->_estimation_error_threshold,
                                          this->_static_motion_threshold,
                                          this->_new_rbm_error_threshold,
                                          this->_max_error_to_reassign_feats,
                                          this->_supporting_features_threshold,
                                          this->_min_num_feats_for_new_rb,
                                          this->_min_num_frames_for_new_rb,
                                          initial_cam_motion_constraint,
                                          this->_static_environment_tracker_type
                                          );

    this->_re_filter->setPriorCovariancePose(prior_cov_pose);
    this->_re_filter->setPriorCovarianceVelocity(prior_cov_vel);
    this->_re_filter->setCovarianceSystemAccelerationTx(cov_sys_acc_tx);
    this->_re_filter->setCovarianceSystemAccelerationTy(cov_sys_acc_ty);
    this->_re_filter->setCovarianceSystemAccelerationTz(cov_sys_acc_tz);
    this->_re_filter->setCovarianceSystemAccelerationRx(cov_sys_acc_rx);
    this->_re_filter->setCovarianceSystemAccelerationRy(cov_sys_acc_ry);
    this->_re_filter->setCovarianceSystemAccelerationRz(cov_sys_acc_rz);
    this->_re_filter->setNumberOfTrackedFeatures(this->_num_tracked_feats);
    this->_re_filter->setMinNumPointsInSegment(min_num_points_in_segment);
    this->_re_filter->setMinProbabilisticValue(min_probabilistic_value);
    this->_re_filter->setMaxFitnessScore(max_fitness_score);
    this->_re_filter->setMinAmountTranslationForNewRB(min_amount_translation_for_new_rb);
    this->_re_filter->setMinAmountRotationForNewRB(min_amount_rotation_for_new_rb);
    this->_re_filter->setMinNumberOfSupportingFeaturesToCorrectPredictedState(min_num_supporting_feats_to_correct);
    this->_re_filter->setRobotInteraction(_robot_interaction);

    this->_re_filter->Init();

    if(_robot_interaction)
    {
        this->_re_filter->setInteractedRigidBodyMaxFeatureError(max_feat_error_irb);
        this->_re_filter->setEndEffectorEstimationThreshold(max_feat_error_ee);
        this->_re_filter->setInteractedRigidBodyMaxDistanceFeaturesToBody(max_distance_irb);
        this->_re_filter->setEndEffectorMaxDistanceFeaturesToBody(max_distance_ee);

        double max_feat_error_ee;
        this->getROSParameter<double>(this->_namespace + std::string("/max_feat_error_ee"), max_feat_error_ee);

        // If we only have proprioception as source of information we create a timed callback that queries the proprioception measurement and steps the loop
        if(_only_proprioception)
        {
            _only_pp_timer = this->_measurements_node_handle.createTimer(ros::Duration((double)_processing_factor/this->_sensor_fps),
                                                                         &MultiRBTrackerNode::measurementCallbackOnlyPP, this);
        }
    }
}

MultiRBTrackerNode::~MultiRBTrackerNode()
{
}

void MultiRBTrackerNode::DynamicReconfigureCallback(rb_tracker::RBTrackerDynReconfConfig &config, uint32_t level)
{
    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    this->_publishing_rbposes_with_cov = config.pub_rb_poses_with_cov;
    this->_publishing_clustered_pc = config.pub_clustered_pc;
    this->_publishing_tf = config.pub_tf;
    this->_printing_rb_poses = config.print_rb_poses;
    if(this->_re_filter)
    {
        this->_re_filter->setDynamicReconfigureValues(config);
    }
}

void MultiRBTrackerNode::ReadParameters()
{
    this->getROSParameter<int>(this->_namespace + std::string("/ransac_iterations"), this->_ransac_iterations);
    this->getROSParameter<double>(this->_namespace + std::string("/estimation_error_threshold"), this->_estimation_error_threshold);
    this->getROSParameter<double>(this->_namespace + std::string("/static_motion_threshold"), this->_static_motion_threshold);
    this->getROSParameter<double>(this->_namespace + std::string("/new_rbm_error_threshold"), this->_new_rbm_error_threshold);
    this->getROSParameter<double>(this->_namespace + std::string("/max_error_to_reassign_feats"), this->_max_error_to_reassign_feats);
    this->getROSParameter<int>(this->_namespace + std::string("/supporting_features_threshold"), this->_supporting_features_threshold);
    this->getROSParameter<int>(this->_namespace + std::string("/min_num_feats_for_new_rb"), this->_min_num_feats_for_new_rb);
    this->getROSParameter<int>(this->_namespace + std::string("/min_num_frames_for_new_rb"), this->_min_num_frames_for_new_rb);

    this->getROSParameter<bool>(this->_namespace + std::string("/pub_rb_poses_with_cov"), this->_publishing_rbposes_with_cov, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_clustered_pc"), this->_publishing_clustered_pc, true);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_tf"), this->_publishing_tf, true);
    this->getROSParameter<bool>(this->_namespace + std::string("/print_rb_poses"), this->_printing_rb_poses, true);

    this->getROSParameter<double>(std::string("/omip/sensor_fps"), this->_sensor_fps);
    this->getROSParameter<int>(std::string("/omip/processing_factor"), this->_processing_factor);
    this->_loop_period_ns = 1e9/(this->_sensor_fps/(double)this->_processing_factor);

    if(!_only_proprioception)
    {
        this->getROSParameter<int>(std::string("/feature_tracker/number_features"), this->_num_tracked_feats);
    }else{
        _num_tracked_feats = 1;
    }


    ROS_INFO_STREAM_NAMED( "MultiRBTrackerNode.ReadParameters",
                           "MultiRBTrackerNode Parameters: " << std::endl //
                           << "\tRANSAC iterations: " << this->_ransac_iterations << std::endl //
                           << "\tEstimation error threshold: " << this->_estimation_error_threshold<< std::endl //
                           << "\tStatic motion threshold: " << this->_static_motion_threshold << std::endl //
                           << "\tNew RB motion threshold: " << this->_new_rbm_error_threshold<< std::endl //
                           << "\tMax error to reassign features: " << this->_max_error_to_reassign_feats<< std::endl //
                           << "\tNumber of tracked features: " << this->_num_tracked_feats<< std::endl //
                           << "\tSupporting features threshold: " << this->_supporting_features_threshold<< std::endl //
                           << "\tMin num of feats for motion estimation: " << this->_min_num_feats_for_new_rb << std::endl //
                           << "\tMin age for creation: " << this->_min_num_frames_for_new_rb << std::endl //
                           << "\tMax framerate (factor): " << this->_sensor_fps << "(" << this->_processing_factor << ")" );
}

void MultiRBTrackerNode::MeasurementFromShapeTrackerCallback(const boost::shared_ptr<omip_msgs::ShapeTrackerStates const> &shape_tracker_states)
{
    this->_shape_tracker_meas = omip_msgs::ShapeTrackerStates(*shape_tracker_states);
    this->_re_filter->processMeasurementFromShapeTracker(this->_shape_tracker_meas);


    //    ROS_WARN_STREAM( "Current time minus refinement time: " << (this->_current_measurement_time - shape_tracker_states->header.stamp).toSec() );
    //    //Check if the result includes a valid refinement and updates
    //    if((this->_current_measurement_time - shape_tracker_states->header.stamp).toSec() < MAX_DELAY_BETWEEN_FT_AND_ST)
    //    {
    //        ROS_WARN_STREAM("Received measurement from shape tracker");
    //        this->_shape_tracker_meas = omip_msgs::ShapeTrackerStates(*shape_tracker_states);
    //        this->_shape_tracker_received = true;
    //    }else{
    //        ROS_ERROR_STREAM("Received measurement from shape tracker but too late. Time of the refinement: "
    //                         << shape_tracker_states->header.stamp <<  " Current time: " << this->_current_measurement_time);
    //    }
}

void MultiRBTrackerNode::measurementCallbackOnlyPP(const ros::TimerEvent& event)
{
    ros::Time tinit = ros::Time::now();

    ros::Time time_latest;
    std::string error_string;
    if(this->_tf_listener.getLatestCommonTime("/camera_rgb_optical_frame", "/ee", time_latest , &error_string))
    {
        ROS_ERROR("The latest commot time between ee and camera_rgb_optical_frame cannot be obtained! Are you connected to the robot?");
        ROS_ERROR_STREAM(error_string);
    }

    this->_current_measurement_time = time_latest;//ros::Time::now() - ros::Duration(5*(double)_processing_factor/this->_sensor_fps);// We compute one step behind because we need that TF contains the transformation

    // Measure the time interval between the previous measurement and the current measurement
    ros::Duration time_between_meas = this->_current_measurement_time - this->_previous_measurement_time;

    // Create empty visual measurement and pass it to the filter
    rbt_measurement_t features_pc_pcl = rbt_measurement_t(new FeatureCloudPCLwc());
    this->_re_filter->setMeasurement(features_pc_pcl, (double)this->_current_measurement_time.toNSec());

    // Get the proprioception measurement
    this->measurementCallbackProprioception();

    // Processing fps: ignore (this->_processing_factor - 1) frames and process one. This gives effectively the desired processing fps: _sensor_fps/_processing_factor
    if( this->_previous_measurement_time.toSec() != 0.)
    {
        // Use the predicted measurement and the received measurement to correct the state
        this->_re_filter->correctState();
    }

    this->_re_filter->ReflectState();

    // Publish the obtained new state
    this->_publishState();

    // Publish additional stuff
    this->_PrintResults();
    this->_PublishTF();
    this->_PublishPosesWithCovariance();

    // We add this pause so that predictions from the higher level have more than enough time to arrive
    // We need these predictions to move the interacted rigid body
    ros::Duration loop_duration(0.02);
    loop_duration.sleep();

    // Predict next RE state
    this->_re_filter->predictState(this->_loop_period_ns);

    // If the received prediction from the higher level is for the next time step we use it
    if(abs((_last_predictions_kh.header.stamp - (this->_current_measurement_time + ros::Duration(_loop_period_ns/1e9))).toNSec()) < _loop_period_ns/2 )
    {
        ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback", "The prediction from higher level can be used to predict next measurement after correction");
        this->_re_filter->addPredictedState(_last_predictions_kh, (double)_last_predictions_kh.header.stamp.toNSec());
    }else{
        ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback",
                               "The prediction from higher level can NOT be used to predict next measurment after correction. Delay: "
                               << (double)((this->_current_measurement_time + ros::Duration(_loop_period_ns/1e9)) - _last_predictions_kh.header.stamp).toNSec()/1e9);
    }

    // Predict next measurement based on the predicted state
    this->_re_filter->predictMeasurement();

    this->_previous_measurement_time = this->_current_measurement_time;

    ros::Time tend = ros::Time::now();
    ROS_WARN_STREAM("Time between meas pp: " << time_between_meas.toSec()*1000 << " ms");
    ROS_WARN_STREAM("Total meas processing time pp: " << (tend-tinit).toSec()*1000 << " ms");
}

void MultiRBTrackerNode::measurementCallback(const boost::shared_ptr<rbt_measurement_ros_t const> &features_pc)
{
    ros::Time tinit = ros::Time::now();

    // Get the time of the measurement as reference time
    this->_current_measurement_time = features_pc->header.stamp;

    // Convert point cloud of features to PCL pc
    rbt_measurement_t features_pc_pcl = rbt_measurement_t(new FeatureCloudPCLwc());
    pcl::fromROSMsg(*features_pc, *features_pc_pcl);

    this->_re_filter->setMeasurement(features_pc_pcl, (double)this->_current_measurement_time.toNSec());

    ///////PROPRIOCEPTION
    /// I need to do this update synchronized with the visual update because otherwise they could try to access the same parts of the
    /// filter concurrently (e.g. the filter update)
    if(_robot_interaction)
    {
        this->measurementCallbackProprioception();
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
        ROS_ERROR("First iteration. We predict after correct");
    }else{
        if( frames_between_meas != this->_processing_factor)
        {
            ROS_ERROR("Lost frames:%3d. Need to predict state and measurement again.", frames_between_meas - this->_processing_factor);

            // Predict next RE state
            this->_re_filter->predictState((double)time_between_meas.toNSec());

            // If the received prediction from the higher level is for this time step we use it
            if(abs((_last_predictions_kh.header.stamp - this->_current_measurement_time).toNSec()) < _loop_period_ns/2  )
            {
                ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback", "The prediction from higher level can be used to predict next measurement before correction");
                this->_re_filter->addPredictedState(_last_predictions_kh, (double)_last_predictions_kh.header.stamp.toNSec());
            }else{
                ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback",
                                       "The prediction from higher level can NOT be used to predict next measurement before correction. Delay: "
                                       << (double)(this->_current_measurement_time - _last_predictions_kh.header.stamp).toNSec()/1e9);
            }

            // Predict next measurement based on the predicted state
            this->_re_filter->predictMeasurement();
        }else{
            ROS_INFO("Frames between measurements:%3d.", frames_between_meas);
        }
    }

    // In the first iteration we cannot correct the state because we have only one measurement
    if(this->_previous_measurement_time.toSec() != 0.)
    {
        // Use the predicted measurement and the received measurement to correct the state
        this->_re_filter->correctState();
    }

    this->_re_filter->ReflectState();

    // Publish the obtained new state
    this->_publishState();

    // Publish additional stuff
    this->_PrintResults();
    this->_PublishTF();
    this->_PublishClusteredFeatures();
    this->_PublishPosesWithCovariance();

    // We add this pause so that predictions from the higher level have more than enough time to arrive
    // We need these predictions to move the interacted rigid body
    if(_robot_interaction)
    {
        ros::Duration loop_duration(0.02);
        loop_duration.sleep();
    }

    // Predict next RE state
    this->_re_filter->predictState(this->_loop_period_ns);

    // If the received prediction from the higher level is for the next time step we use it
    if(abs((_last_predictions_kh.header.stamp - (this->_current_measurement_time + ros::Duration(_loop_period_ns/1e9))).toNSec()) < _loop_period_ns/2 )
    {
        ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback", "The prediction from higher level can be used to predict next measurement after correction");
        this->_re_filter->addPredictedState(_last_predictions_kh, (double)_last_predictions_kh.header.stamp.toNSec());
    }else{
        ROS_ERROR_STREAM_NAMED("MultiRBTrackerNode.measurementCallback",
                               "The prediction from higher level can NOT be used to predict next measurment after correction. Delay: "
                               << (double)((this->_current_measurement_time + ros::Duration(_loop_period_ns/1e9)) - _last_predictions_kh.header.stamp).toNSec()/1e9);
    }

    // Predict next measurement based on the predicted state
    this->_re_filter->predictMeasurement();

    this->_publishPredictedMeasurement();

    this->_previous_measurement_time = this->_current_measurement_time;

    ros::Time tend = ros::Time::now();
    ROS_WARN_STREAM("Time between meas vision: " << time_between_meas.toSec()*1000 << " ms");
    ROS_WARN_STREAM("Total meas processing time vision: " << (tend-tinit).toSec()*1000 << " ms");
}

void MultiRBTrackerNode::measurementCallbackProprioception()
{
    tf::StampedTransform current_eef_wrt_camframe_transform;
    Eigen::Matrix4d current_eef_wrt_camframe_eigen;
    ros::Time time_latest;
    std::string error_string;
    // Returns non-zero if an error
    if(this->_tf_listener.getLatestCommonTime("/camera_rgb_optical_frame", "/ee", time_latest , &error_string))
    {
        ROS_ERROR("The current ee pose cannot be obtained! Are you connected to the robot?");
        ROS_ERROR_STREAM(error_string);
        current_eef_wrt_camframe_eigen = Eigen::Matrix4d::Identity();
    }else{
        if(_current_measurement_time - time_latest  < ros::Duration(1e-2))
        {
            try{
                // I should use _current_measurement_time_proprioception but it only works with the real robot!
                if(_current_measurement_time - time_latest  < ros::Duration(0.0))
                {
                    this->_tf_listener.lookupTransform( "/camera_rgb_optical_frame", "/ee", _current_measurement_time , current_eef_wrt_camframe_transform);
                }else{
                    this->_tf_listener.lookupTransform( "/camera_rgb_optical_frame", "/ee", time_latest , current_eef_wrt_camframe_transform);
                }
                current_eef_wrt_camframe_transform.getOpenGLMatrix(current_eef_wrt_camframe_eigen.data());
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                current_eef_wrt_camframe_eigen = Eigen::Matrix4d::Identity();
            }
        }else{
            ROS_ERROR("The current ee pose cannot be obtained! Are you connected to the robot?");
            std::cout << "Current meas time: " << _current_measurement_time << std::endl;
            std::cout << "Time of the last common tf: " << time_latest << std::endl;
            current_eef_wrt_camframe_eigen = Eigen::Matrix4d::Identity();
        }
    }

    this->_re_filter->setMeasurementProprioception(current_eef_wrt_camframe_eigen, (double)_current_measurement_time.toNSec());
}

void MultiRBTrackerNode::statePredictionCallback(const boost::shared_ptr<rbt_state_t const> &predicted_rb_poses)
{
    this->_last_predictions_kh = rbt_state_t(*predicted_rb_poses);
}

void MultiRBTrackerNode::_publishState() const
{
    rbt_state_t poses_and_vels_msg = this->_re_filter->getState();
    poses_and_vels_msg.header.stamp = this->_current_measurement_time;

    poses_and_vels_msg.header.frame_id = "camera_rgb_optical_frame";
    this->_state_publisher2.publish(poses_and_vels_msg);

    rbt_state_t poses_and_vels_msg_corrected = this->_re_filter->getState();
    poses_and_vels_msg_corrected.header.stamp = this->_current_measurement_time;

    poses_and_vels_msg_corrected.header.frame_id = "camera_rgb_optical_frame";

    this->_state_publisher.publish(poses_and_vels_msg_corrected);

    this->_shape_tracker_received = false;
}

void MultiRBTrackerNode::_publishPredictedMeasurement() const
{
    FeatureCloudPCLwc::Ptr predicted_locations = this->_re_filter->getPredictedMeasurement();
    predicted_locations->header.frame_id = "camera_rgb_optical_frame";

    sensor_msgs::PointCloud2 predicted_locations_ros;
    pcl::toROSMsg(*predicted_locations, predicted_locations_ros);

    this->_measurement_prediction_publisher.publish(predicted_locations_ros);
}

void MultiRBTrackerNode::_PrintResults() const
{
    if(this->_printing_rb_poses)
    {
        // Get the poses (twists) of each RB
        std::vector<Eigen::Matrix4d> tracked_bodies_motion = this->_re_filter->getPoses();

        Eigen::Twistd pose_in_ec;

        RB_id_t RB_id;
        int num_supporting_feats = 0;
        for (size_t poses_idx = 0; poses_idx < tracked_bodies_motion.size(); poses_idx++)
        {
            RB_id = this->_re_filter->getRBId(poses_idx);
            num_supporting_feats = this->_re_filter->getNumberSupportingFeatures(poses_idx);

            TransformMatrix2Twist(tracked_bodies_motion[poses_idx], pose_in_ec);
#define MULTIMODAL_EE_FILTER_ID 3
#define DEFORMED_END_EFFECTOR_FILTER_ID 4
#define INTERACTED_RB_FILTER_ID 5
            if(_robot_interaction && RB_id == MULTIMODAL_EE_FILTER_ID)
            {
                ROS_INFO_NAMED( "MultiRBTrackerNode::_PublishRBPoses", "RB%2d (MM EE): Supporting feats: %d, Pose (vx,vy,vz,rx,ry,rz): % 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f",
                                (int)RB_id,
                                num_supporting_feats,
                                pose_in_ec.vx(),
                                pose_in_ec.vy(),
                                pose_in_ec.vz(),
                                pose_in_ec.rx(),
                                pose_in_ec.ry(),
                                pose_in_ec.rz());
            }else if(_robot_interaction && RB_id == DEFORMED_END_EFFECTOR_FILTER_ID)
            {
                ROS_INFO_NAMED( "MultiRBTrackerNode::_PublishRBPoses", "RB%2d (DEF EE): Supporting feats: %d, Pose (vx,vy,vz,rx,ry,rz): % 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f",
                                (int)RB_id,
                                num_supporting_feats,
                                pose_in_ec.vx(),
                                pose_in_ec.vy(),
                                pose_in_ec.vz(),
                                pose_in_ec.rx(),
                                pose_in_ec.ry(),
                                pose_in_ec.rz());

            }else if(_robot_interaction && RB_id == INTERACTED_RB_FILTER_ID)
            {
                ROS_INFO_NAMED( "MultiRBTrackerNode::_PublishRBPoses", "RB%2d (IB): Supporting feats: %d, Pose (vx,vy,vz,rx,ry,rz): % 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f",
                                (int)RB_id,
                                num_supporting_feats,
                                pose_in_ec.vx(),
                                pose_in_ec.vy(),
                                pose_in_ec.vz(),
                                pose_in_ec.rx(),
                                pose_in_ec.ry(),
                                pose_in_ec.rz());
            }else{
                ROS_INFO_NAMED( "MultiRBTrackerNode::_PublishRBPoses", "RB%2d: Supporting feats: %d, Pose (vx,vy,vz,rx,ry,rz): % 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f",
                                (int)RB_id,
                                num_supporting_feats,
                                pose_in_ec.vx(),
                                pose_in_ec.vy(),
                                pose_in_ec.vz(),
                                pose_in_ec.rx(),
                                pose_in_ec.ry(),
                                pose_in_ec.rz());
            }
        }
    }
}

void MultiRBTrackerNode::_PublishPosesWithCovariance()
{
    if(this->_publishing_rbposes_with_cov)
    {
        // Get the poses and covariances on the poses (pose with cov) of each RB
        std::vector<geometry_msgs::PoseWithCovariancePtr> tracked_poses_with_cov = this->_re_filter->getPosesWithCovariance();

        RB_id_t RB_id;
        for (size_t poses_idx = 0; poses_idx < tracked_poses_with_cov.size(); poses_idx++)
        {
            RB_id = this->_re_filter->getRBId(poses_idx);
            std::ostringstream oss;
            oss << "ip/rb" << RB_id;

            // Publish the pose with covariance of each RB with a different publisher and on a different topic name
            if (this->_est_body_publishers.find(RB_id) == this->_est_body_publishers.end())
            {
                ros::Publisher* new_estimated_pose_publisher = new ros::Publisher(
                            this->_measurements_node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(oss.str(), 100));
                this->_est_body_publishers[RB_id] = new_estimated_pose_publisher;
            }
            geometry_msgs::PoseWithCovarianceStamped pose_with_cov_stamped;
            pose_with_cov_stamped.header.stamp = this->_current_measurement_time;
            pose_with_cov_stamped.header.frame_id = "camera_rgb_optical_frame";
            pose_with_cov_stamped.pose = *(tracked_poses_with_cov[poses_idx]);
            this->_est_body_publishers[this->_re_filter->getRBId(poses_idx)]->publish(pose_with_cov_stamped);
        }
    }
}

void MultiRBTrackerNode::_PublishTF()
{
    static tf::TransformBroadcaster tf_br;

    if(this->_publishing_tf)
    {
        // Get the poses and covariances on the poses (pose with cov) of each RB
        std::vector<geometry_msgs::PoseWithCovariancePtr> tracked_poses_with_cov = this->_re_filter->getPosesWithCovariance();
        RB_id_t RB_id;
        for (size_t poses_idx = 0; poses_idx < tracked_poses_with_cov.size(); poses_idx++)
        {
            RB_id = this->_re_filter->getRBId(poses_idx);
            std::ostringstream oss;
            oss << "ip/rb" << RB_id;

            // Publish the pose of the RBs on the TF tree
            tf::Transform transform_auxiliar;
            transform_auxiliar.setOrigin(tf::Vector3(tracked_poses_with_cov[poses_idx]->pose.position.x, tracked_poses_with_cov[poses_idx]->pose.position.y,
                                                     tracked_poses_with_cov[poses_idx]->pose.position.z));
            transform_auxiliar.setRotation(tf::Quaternion(tracked_poses_with_cov.at(poses_idx)->pose.orientation.x,tracked_poses_with_cov.at(poses_idx)->pose.orientation.y,
                                                          tracked_poses_with_cov.at(poses_idx)->pose.orientation.z, tracked_poses_with_cov.at(poses_idx)->pose.orientation.w));

            tf_br.sendTransform(tf::StampedTransform(transform_auxiliar, this->_current_measurement_time, "camera_rgb_optical_frame",
                                                     RB_id == 0 ? "static_environment" : oss.str().c_str()));
        }
    }
}

void MultiRBTrackerNode::_PublishClusteredFeatures()
{
    if(this->_publishing_clustered_pc)
    {
        FeatureCloudPCL clustered_pc = this->_re_filter->getLabelledSupportingFeatures();
        clustered_pc.header.frame_id = "camera_rgb_optical_frame";
        clustered_pc.header.stamp = pcl_conversions::toPCL(this->_current_measurement_time);

        sensor_msgs::PointCloud2 feature_set_ros;
        pcl::toROSMsg(clustered_pc, feature_set_ros);
        feature_set_ros.header.stamp = this->_current_measurement_time;
        this->_clustered_pc_publisher.publish(feature_set_ros);

        FeatureCloudPCL freefeats_pc = this->_re_filter->getFreeFeatures();
        freefeats_pc.header.frame_id = "camera_rgb_optical_frame";
        freefeats_pc.header.stamp = pcl_conversions::toPCL(this->_current_measurement_time);

        sensor_msgs::PointCloud2 freefeats_pc_ros;
        pcl::toROSMsg(freefeats_pc, freefeats_pc_ros);
        this->_freefeats_pc_publisher.publish(freefeats_pc_ros);

        FeatureCloudPCL predictedfeats_pc = this->_re_filter->getPredictedFeatures();
        predictedfeats_pc.header.frame_id = "camera_rgb_optical_frame";
        predictedfeats_pc.header.stamp = pcl_conversions::toPCL(this->_current_measurement_time);

        sensor_msgs::PointCloud2 predictedfeats_pc_ros;
        pcl::toROSMsg(predictedfeats_pc, predictedfeats_pc_ros);
        this->_predictedfeats_pc_publisher.publish(predictedfeats_pc_ros);

        FeatureCloudPCL atbirthfeats_pc = this->_re_filter->getAtBirthFeatures();
        atbirthfeats_pc.header.frame_id = "camera_rgb_optical_frame";
        atbirthfeats_pc.header.stamp = pcl_conversions::toPCL(this->_current_measurement_time);

        sensor_msgs::PointCloud2 atbirthfeats_pc_ros;
        pcl::toROSMsg(atbirthfeats_pc, atbirthfeats_pc_ros);
        this->_atbirthfeats_pc_publisher.publish(atbirthfeats_pc_ros);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rb_tracker");
    MultiRBTrackerNode rbtvn;
    rbtvn.run();

    return (0);
}
