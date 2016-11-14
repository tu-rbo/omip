#include "feature_tracker/FeatureTrackerNode.h"
#include <pcl/conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <rosgraph_msgs/Clock.h>

#include <std_msgs/Float64.h>

#include "feature_tracker/PointFeatureTracker.h"

#include <unistd.h>

#include <pcl_conversions/pcl_conversions.h>

#if ROS_VERSION_MINIMUM(1, 11, 1) // if current ros version is >= 1.10.1 (hydro)
#else
#include "feature_tracker/pcl_conversions_indigo.h"
#endif

#define LINES_OF_TERMINAL_FT 4

using namespace omip;

FeatureTrackerNode::FeatureTrackerNode() :
    RecursiveEstimatorNodeInterface(1),
    _image_transport(this->_measurements_node_handle),
    _synchronizer(NULL),
    _data_from_bag(false),
    _ci_initialized(false),
    _subscribe_to_pc(false),
    _publishing_depth_edges_img(false),
    _publishing_tracked_feats_img(false),
    _publishing_rgb_img(false),
    _publishing_depth_img(false),
    _publishing_tracked_feats_with_pred_msk_img(false),
    _publishing_full_pc(false),
    _publishing_predicting_msk_img(false),
    _publishing_tracking_msk_img(false),
    _publishing_detecting_msk_img(false),
    _publishing_predicted_and_past_feats_img(false),
    _republishing_predicted_feat_locs(false),
    _advance_frame_min_wait_time(0.0),
    _sensor_fps(0.0),
    _loop_period_ns(0.0),
    _processing_factor(0),
    _advance_frame_mechanism(0),
    _advance_frame_max_wait_time(60.),
    _advance_sub_returned_true(false),
    _attention_to_motion(false),
    _occlusion_mask_positive(true)
{
    this->_namespace = std::string("feature_tracker");

    this->_ReadParameters();
    
    this->_SubscribeAndAdvertiseTopics();
    
    this->_InitializeVariables();
}

FeatureTrackerNode::~FeatureTrackerNode()
{  
}

void FeatureTrackerNode::measurementCallback(const sensor_msgs::PointCloud2ConstPtr& full_pc_msg,
                                             const sensor_msgs::ImageConstPtr &depth_image_msg,
                                             const sensor_msgs::ImageConstPtr &rgb_image_msg)
{
    // Pass the full RGBD point cloud
    PointCloudPCL temp_cloud;
    pcl::fromROSMsg(*full_pc_msg, temp_cloud);
    this->_full_rgbd_pc = boost::make_shared<const PointCloudPCL>(temp_cloud);
    this->_re_filter->setFullRGBDPC(this->_full_rgbd_pc);

    // Call the reduced callback, so that we don't duplicate code
    this->measurementCallback(depth_image_msg, rgb_image_msg);
}

void FeatureTrackerNode::measurementCallback(const sensor_msgs::ImageConstPtr &depth_image_msg,
                                             const sensor_msgs::ImageConstPtr &rgb_image_msg)
{
    ros::Time first = ros::Time::now();

    // Set the current time to be the time of the depth image
    ros::Time current_measurement_time = depth_image_msg->header.stamp;

    ros::Duration time_between_meas;

    // If we are in the first iteration the prev meas time is zero and we don't check for the elapsed time
    if(this->_previous_measurement_time.toSec() != 0.)
    {
        // Measure the time interval between the previous measurement and the current measurement
        time_between_meas = current_measurement_time - this->_previous_measurement_time;

        // The time interval between frames is the inverse of the max_framerate
        double period_between_frames = 1.0/this->_sensor_fps;

        // The number of frames elapsed is the time elapsed divided by the period between frames
        int frames_between_meas = round((time_between_meas.toSec())/period_between_frames);

        // Processing fps: ignore (this->_processing_factor - 1) frames and process one. This gives effectively the desired processing fps: _sensor_fps/_processing_factor
        if( frames_between_meas < this->_processing_factor )
        {
            return;
        }
        else if(frames_between_meas == this->_processing_factor )
        {
            ROS_INFO("  0 frames lost. Processing at %d fps.", (int)this->_sensor_fps);
        }
        else if(frames_between_meas > this->_processing_factor)
        {
            // We show this message if we have lost frames
            ROS_ERROR("%3d frames lost! Consider setting a lower frame rate." , frames_between_meas - this->_processing_factor);
        }
    }

    // Get the time of the current measurement (reference time from depth maps channel)
    this->_current_measurement_time = current_measurement_time;

    // Create an ft_measurement object and pass it to the RE
    this->_cv_ptr_depth = cv_bridge::toCvCopy(depth_image_msg);
    this->_cv_ptr_rgb = cv_bridge::toCvCopy(rgb_image_msg);
    ft_measurement_t latest_measurement = ft_measurement_t( this->_cv_ptr_rgb, this->_cv_ptr_depth);

    this->_re_filter->setMeasurement(latest_measurement, (double)this->_current_measurement_time.toNSec());

    // Predict next RE state
    this->_re_filter->predictState(this->_loop_period_ns);

    // Predict next measurement based on the predicted state
    this->_re_filter->predictMeasurement();

    //TOBETESTED: We do not jump the first frame because we need to detect features
    // Use the predicted measurement and the received measurement to correct the state
    this->_re_filter->correctState();

    // Publish the obtained new state
    this->_publishState();
    
    // Publish additional stuff
    this->PublishRGBImg();
    this->PublishDepthImg();
    this->PublishDepthEdgesImg();
    this->PublishTrackedFeaturesImg();
    this->PublishTrackedFeaturesWithPredictionMaskImg();
    this->PublishDetectingMaskImg();
    this->PublishTrackingMaskImg();
    this->PublishPredictedAndLastFeaturesImg();
    this->PublishPredictionMaskImg();

    this->RepublishPredictedFeatLocation();

    if(this->_publishing_full_pc && this->_subscribe_to_pc)
    {
        this->PublishFullRGBDPC();
    }
    

    if (this->_data_from_bag)
    {
        static int advanced_frame_number = 0;
        switch(this->_advance_frame_mechanism)
        {
        case 0://0 -> Automatically advancing, no waiting
        {
            usleep((int)(this->_advance_frame_min_wait_time));
        }
            break;
        case 1://1 -> Manually advancing
        {
            std::cout << "Press enter to process the next frame" << std::endl;
            getchar();
        }
            break;
        case 2://2 -> Wait for signal from Shape Reconstruction
        case 3://3 -> Wait for signal from Shape Tracker
        {

            double max_time = _advance_frame_max_wait_time; //(seconds)

            if (max_time < 0) {
                std::cout << "WARNING: no time limit, waiting forever" << std::endl;
                max_time = std::numeric_limits<double>::max();
            }

            std::cout << "Waiting for advance response in FTracker" ;

            double wait_time=0.;
            while(advanced_frame_number > 0 && !this->_advance_sub_returned_true && wait_time < max_time && this->_active)
            {
                usleep(1e5);
                printf(" (%4.1fs)", wait_time);
                wait_time += 0.1;
            }
            if (wait_time >= max_time) {
                std::cout << " Exceeded time limit " << max_time ;
            } else {
                std::cout << " Refinement received in FTracker" ;
            }
            this->_advance_sub_returned_true = false;
        }
            break;
        }
        advanced_frame_number++;
        std::cout << ". Reading frame " << advanced_frame_number << std::endl;
    }

    //After processing, the time stamp of the current measurement becomes the previous time
    this->_previous_measurement_time = this->_current_measurement_time;

    ros::Time last = ros::Time::now();
    ROS_WARN_STREAM("Time between meas: " << time_between_meas.toSec()*1000 << " ms");
    ROS_WARN_STREAM("Total meas processing time: " << (last-first).toSec()*1000 << " ms");
}

void FeatureTrackerNode::statePredictionCallback(const boost::shared_ptr<ft_state_ros_t const> &predicted_locations_pc)
{
    //convert dataset
    FeatureCloudPCLwc temp_cloud;
    pcl::fromROSMsg(*predicted_locations_pc, temp_cloud);
    this->_predicted_locations_pc = boost::make_shared<FeatureCloudPCLwc>(temp_cloud);

    this->_re_filter->addPredictedState(this->_predicted_locations_pc, (double)predicted_locations_pc->header.stamp.toNSec());
}

void FeatureTrackerNode::DynamicReconfigureCallback(feature_tracker::FeatureTrackerDynReconfConfig &config, uint32_t level)
{
    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    this->_publishing_depth_edges_img = config.pub_depth_edges_img;
    this->_publishing_tracked_feats_img = config.pub_tracked_feats_img;
    this->_publishing_rgb_img = config.pub_rgb_img;
    this->_publishing_depth_img = config.pub_depth_img;
    this->_publishing_tracked_feats_with_pred_msk_img = config.pub_tracked_feats_with_pred_mask_img;
    //this->_publishing_full_pc = config.pub_full_pc;
    this->_publishing_predicting_msk_img = config.pub_predicting_msk_img;
    this->_publishing_tracking_msk_img = config.pub_tracking_msk_img;
    this->_publishing_detecting_msk_img = config.pub_detecting_msk_img;
    this->_publishing_predicted_and_past_feats_img = config.pub_predicted_and_past_feats_img;

    this->_republishing_predicted_feat_locs = config.repub_predicted_feat_locs;

    this->_advance_frame_min_wait_time = 1e6*(config.advance_frame_min_wait_time);
    if(this->_re_filter)
    {
        this->_re_filter->setDynamicReconfigureValues(config);
    }
}

void FeatureTrackerNode::ReadRosBag()
{
    rosbag::Bag bag;
    ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadRosBag", "Reading rosbag: " <<this->_bag_file_name);
    bag.open(this->_bag_file_name, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(this->_full_rgbd_pc_topic);
    topics.push_back(this->_depth_img_topic);
    topics.push_back(this->_rgb_img_topic);
    topics.push_back(this->_occlusion_mask_img_topic);
    topics.push_back(this->_ci_topic);
    topics.push_back("/tf");

    ROS_DEBUG_STREAM_NAMED( "FeatureTrackerNode.ReadRosBag",
                            "Topics: " << this->_full_rgbd_pc_topic << std::endl
                            << this->_depth_img_topic << std::endl
                            << this->_rgb_img_topic << std::endl
                            << this->_occlusion_mask_img_topic << std::endl
                            << this->_ci_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int messages_counter = 0;
    rosgraph_msgs::Clock time_now_msg;

    // When we read a rosbag we use the time of the bag
    // We get the time from the messages and we publish it, so that the rest of the ROS system gets the time of the bag
    ros::Time time_now;
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        if(!this->_active)
            break;

        if (m.getTopic() == this->_full_rgbd_pc_topic || ("/" + m.getTopic() == this->_full_rgbd_pc_topic))
        {
            sensor_msgs::PointCloud2::ConstPtr points_pc = m.instantiate<sensor_msgs::PointCloud2>();
            if (points_pc != NULL )
            {
                if(this->_subscribe_to_pc)
                {
                    this->_bag_full_rgbd_pc_sub.newMessage(points_pc);
                }
                time_now = points_pc->header.stamp;
            }
            this->_camera_info_pub.publish(this->_camera_info_msg);
            this->_camera_info_pub2.publish(this->_camera_info_msg);
        }

        if (m.getTopic() == this->_occlusion_mask_img_topic || ("/" + m.getTopic() == this->_occlusion_mask_img_topic))
        {
            sensor_msgs::Image::Ptr occlusion_mask = m.instantiate<sensor_msgs::Image>();
            if (occlusion_mask != NULL)
            {
                time_now = occlusion_mask->header.stamp;
                this->OcclusionMaskImgCallback(occlusion_mask);
            }
        }

        if (m.getTopic() == this->_depth_img_topic || ("/" + m.getTopic() == this->_depth_img_topic))
        {
            sensor_msgs::Image::Ptr depth_img =  m.instantiate<sensor_msgs::Image>();
            if (depth_img != NULL)
            {
                time_now = depth_img->header.stamp;
                this->_bag_depth_img_sub.newMessage(depth_img);
            }
        }

        if (m.getTopic() == this->_rgb_img_topic || ("/" + m.getTopic() == this->_rgb_img_topic))
        {
            sensor_msgs::Image::Ptr rgb_img = m.instantiate<sensor_msgs::Image>();
            if (rgb_img != NULL)
            {
                time_now = rgb_img->header.stamp;
                this->_bag_rgb_img_sub.newMessage(rgb_img);
            }

            // Spin here the queue of the measurements
            if (ros::ok())
            {
                this->_measurements_queue->callAvailable(ros::WallDuration(0.0));
            }
        }

        if ((m.getTopic() == this->_ci_topic || ("/" + m.getTopic() == this->_ci_topic)) && !this->_ci_initialized)
        {
            sensor_msgs::CameraInfo::ConstPtr ci_msg = m.instantiate<sensor_msgs::CameraInfo>();
            if (ci_msg != NULL)
            {
                time_now = ci_msg->header.stamp;
                this->_camera_info_msg = sensor_msgs::CameraInfo(*ci_msg);
                this->_re_filter->setCameraInfoMsg(&this->_camera_info_msg);
                this->_ci_initialized = true;
            }
        }


        static tf::TransformBroadcaster br;
        static bool first_ee_tf = true;

        if ((m.getTopic() == "/tf" || ("/" + m.getTopic() == "/tf")))
        {
            tf::tfMessage::Ptr tf_msg = m.instantiate<tf::tfMessage>();
            if (tf_msg != NULL)
            {
                time_now = tf_msg->transforms.at(0).header.stamp;
                for(int tf_transforms_idx = 0; tf_transforms_idx < tf_msg->transforms.size() ; tf_transforms_idx++)
                {
                    geometry_msgs::TransformStamped transform = tf_msg->transforms.at(tf_transforms_idx);
                    if(transform.child_frame_id == "/ee")
                    {
                        if(first_ee_tf)
                        {
                            tf::StampedTransform cam_link_to_camera_rgb_optical_frame;

                            tf::Vector3 translation = tf::Vector3(0.000, 0.020, 0.000);
                            cam_link_to_camera_rgb_optical_frame.setOrigin(translation);
                            tf::Quaternion rotation = tf::createQuaternionFromRPY(0.,0.0,-M_PI_2);
                            cam_link_to_camera_rgb_optical_frame.setRotation(rotation);

                            tf::transformMsgToTF(transform.transform, this->_initial_ee_tf);
                            this->_initial_ee_tf =  this->_initial_ee_tf * cam_link_to_camera_rgb_optical_frame;
                            first_ee_tf = false;
                        }

                        //transform.header.stamp = ros::Time::now();
                        br.sendTransform(transform);

                        geometry_msgs::TransformStamped initial_tf;
                        tf::transformTFToMsg(this->_initial_ee_tf, initial_tf.transform);
                        initial_tf.header = transform.header;
                        initial_tf.child_frame_id = "initial_transform";
                        br.sendTransform(initial_tf);
                    }
                }
            }
        }

        time_now_msg.clock = time_now;
        this->_time_repub.publish(time_now_msg);

        messages_counter++;
    }

    if(this->_active)
    {
        ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadRosBag", "End of the rosbag.");
    }
    bag.close();    
    this->_shutdown_publisher.publish(std_msgs::Empty());
}

void FeatureTrackerNode::OcclusionMaskImgCallback(const sensor_msgs::ImageConstPtr &occlusion_mask_img)
{
    this->_cv_ptr_occlusion_msk = cv_bridge::toCvCopy(occlusion_mask_img, "mono8");
    this->_re_filter->setOcclusionMaskImg(this->_cv_ptr_occlusion_msk);
}

void FeatureTrackerNode::PublishRGBImg()
{
    if(this->_publishing_rgb_img)
    {
        cv_bridge::CvImagePtr rgb_img = this->_re_filter->getRGBImg();
        if(rgb_img)
        {
            rgb_img->header.frame_id = "camera_rgb_optical_frame";
            rgb_img->header.stamp = this->_current_measurement_time;
            this->_rgb_img_pub.publish(rgb_img->toImageMsg());
        }
    }
}

void FeatureTrackerNode::PublishDepthImg()
{
    if(this->_publishing_depth_img)
    {
        cv_bridge::CvImagePtr depth_img = this->_re_filter->getDepthImg();
        if(depth_img)
        {
            depth_img->header.frame_id = "camera_rgb_optical_frame";
            depth_img->header.stamp = this->_current_measurement_time;
            this->_depth_img_pub.publish(depth_img->toImageMsg());
        }
    }
}

void FeatureTrackerNode::PublishFullRGBDPC()
{
    if(this->_publishing_full_pc && this->_subscribe_to_pc)
    {
        sensor_msgs::PointCloud2 full_rgbd_pc_ros;
        pcl::toROSMsg(*(this->_full_rgbd_pc), full_rgbd_pc_ros);
        full_rgbd_pc_ros.header.stamp = this->_current_measurement_time;
        full_rgbd_pc_ros.header.frame_id = "camera_rgb_optical_frame";
        this->_full_rgbd_pc_repub.publish(full_rgbd_pc_ros);
    }
}

void FeatureTrackerNode::PublishTrackedFeaturesImg()
{
    if(this->_publishing_tracked_feats_img)
    {
        cv_bridge::CvImagePtr tracked_features_img = this->_re_filter->getTrackedFeaturesImg();
        tracked_features_img->header.frame_id = "camera_rgb_optical_frame";
        tracked_features_img->header.stamp = this->_current_measurement_time;
        this->_tracked_feats_img_pub.publish(tracked_features_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishTrackedFeaturesWithPredictionMaskImg()
{
    if(this->_publishing_tracked_feats_with_pred_msk_img)
    {
        cv_bridge::CvImagePtr tracked_features_with_pm_img = this->_re_filter
                ->getTrackedFeaturesWithPredictionMaskImg();
        tracked_features_with_pm_img->header.frame_id = "camera_rgb_optical_frame";
        tracked_features_with_pm_img->header.stamp = this->_current_measurement_time;
        this->_tracked_feats_with_pm_img_pub.publish(tracked_features_with_pm_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishDepthEdgesImg()
{
    if(this->_publishing_depth_edges_img)
    {
        cv_bridge::CvImagePtr depth_edges_img = this->_re_filter
                ->getDepthEdgesImg();
        depth_edges_img->header.frame_id = "camera_rgb_optical_frame";
        depth_edges_img->header.stamp = this->_current_measurement_time;
        this->_depth_edges_img_pub.publish(depth_edges_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishPredictionMaskImg()
{
    if(this->_publishing_predicting_msk_img)
    {
        cv_bridge::CvImagePtr predicting_mask_img = this->_re_filter->getPredictingMaskImg();
        predicting_mask_img->header.frame_id = "camera_rgb_optical_frame";
        predicting_mask_img->header.stamp = this->_current_measurement_time;
        this->_prediction_mask_img_pub.publish(predicting_mask_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishTrackingMaskImg()
{
    if(this->_publishing_tracking_msk_img)
    {
        cv_bridge::CvImagePtr tracking_mask_img = this->_re_filter->getTrackingMaskImg();
        tracking_mask_img->header.frame_id = "camera_rgb_optical_frame";
        tracking_mask_img->header.stamp = this->_current_measurement_time;
        this->_tracking_mask_img_pub.publish(tracking_mask_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishDetectingMaskImg()
{
    if(this->_publishing_detecting_msk_img)
    {
        cv_bridge::CvImagePtr detecting_mask_img = this->_re_filter->getDetectingMaskImg();
        detecting_mask_img->header.frame_id = "camera_rgb_optical_frame";
        detecting_mask_img->header.stamp = this->_current_measurement_time;
        this->_detecting_mask_img_pub.publish(detecting_mask_img->toImageMsg());
    }
}

void FeatureTrackerNode::PublishPredictedAndLastFeaturesImg()
{
    if(this->_publishing_predicted_and_past_feats_img)
    {
        cv_bridge::CvImagePtr predicted_and_last_feats_img = this->_re_filter->getPredictedAndLastFeaturesImg();
        predicted_and_last_feats_img->header.frame_id = "camera_rgb_optical_frame";
        predicted_and_last_feats_img->header.stamp = this->_current_measurement_time;
        this->_predicted_and_past_feats_img_pub.publish(predicted_and_last_feats_img->toImageMsg());
    }
}

void FeatureTrackerNode::RepublishPredictedFeatLocation()
{
    if(this->_republishing_predicted_feat_locs && this->_predicted_locations_pc)
    {
        //convert dataset
        sensor_msgs::PointCloud2 temp_cloud;
        pcl::toROSMsg(*this->_predicted_locations_pc, temp_cloud);
        temp_cloud.header.stamp = this->_current_measurement_time;
        temp_cloud.header.frame_id = "camera_rgb_optical_frame";
        _pred_feat_locs_repub.publish(temp_cloud);
    }
}

void FeatureTrackerNode::_publishState() const
{
    ft_state_t tracked_features_pc = this->_re_filter->getState();

    sensor_msgs::PointCloud2 tracked_features_pc_ros;
    pcl::toROSMsg(*tracked_features_pc, tracked_features_pc_ros);
    tracked_features_pc_ros.header.stamp = this->_current_measurement_time;
    tracked_features_pc_ros.header.frame_id = "camera_rgb_optical_frame";

    this->_state_publisher.publish(tracked_features_pc_ros);
}

void FeatureTrackerNode::_publishPredictedMeasurement() const
{
    ROS_ERROR_STREAM_NAMED("FeatureTrackerNode._publishPredictedMeasurement", "FeatureTrackerNode::_publishPredictedMeasurement has not been implemented for the Feature Tracker. It could publish the predicted 2D location of the features using both generative models: the internal model (no motion between frames) and the model that uses the higher RE level (velocity of the features");
}

void FeatureTrackerNode::_ReadParameters()
{
    this->getROSParameter<bool>(this->_namespace  + std::string("/data_from_bag"),this->_data_from_bag);
    this->getROSParameter<std::string>(this->_namespace  + std::string("/ft_type"),this->_tracker_type);
    this->getROSParameter<std::string>(this->_namespace  + std::string("/bag_file"),this->_bag_file_name);
    this->getROSParameter<std::string>(this->_namespace + std::string("/rgbd_pc_topic"),this->_full_rgbd_pc_topic);
    this->getROSParameter<std::string>(this->_namespace + std::string("/depth_img_topic"),this->_depth_img_topic);
    this->getROSParameter<std::string>(this->_namespace + std::string("/rgb_img_topic"),this->_rgb_img_topic);
    this->getROSParameter<std::string>(this->_namespace + std::string("/camera_info_topic"),this->_ci_topic);

    this->getROSParameter<bool>(this->_namespace + std::string("/pub_depth_edges_img"),this->_publishing_depth_edges_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_rgb_img"),this->_publishing_rgb_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_depth_img"),this->_publishing_depth_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_tracked_feats_img"),this->_publishing_tracked_feats_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_tracked_feats_with_pred_mask_img"),this->_publishing_tracked_feats_with_pred_msk_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_predicting_msk_img"),this->_publishing_predicting_msk_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_tracking_msk_img"),this->_publishing_tracking_msk_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_detecting_msk_img"),this->_publishing_detecting_msk_img, false);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_predicted_and_past_feats_img"),this->_publishing_predicted_and_past_feats_img, false);

    this->getROSParameter<bool>(this->_namespace + std::string("/repub_predicted_feat_locs"),this->_republishing_predicted_feat_locs, false);

    this->getROSParameter<bool>(this->_namespace + std::string("/attention_to_motion"),this->_attention_to_motion);


    this->getROSParameter<bool>(this->_namespace + std::string("/subscribe_to_pc"),this->_subscribe_to_pc);
    this->getROSParameter<bool>(this->_namespace + std::string("/pub_full_pc"),this->_publishing_full_pc);

    this->getROSParameter<int>(this->_namespace + std::string("/advance_frame_mechanism"),this->_advance_frame_mechanism);
    this->getROSParameter<double>(this->_namespace + std::string("/advance_frame_max_wait_time"),this->_advance_frame_max_wait_time, 60.);
    this->getROSParameter<double>(this->_namespace + std::string("/advance_frame_min_wait_time"),this->_advance_frame_min_wait_time, -1);

    this->_advance_frame_min_wait_time *= 1e6;

    this->getROSParameter<std::string>(this->_namespace + std::string("/selfocclusionfilter_img_topic"),this->_occlusion_mask_img_topic);
    this->_predicted_locations_pc_topic = std::string(this->_namespace + "/predicted_measurement");
    this->getROSParameter<bool>(this->_namespace + std::string("/selfocclusionfilter_positive"),this->_occlusion_mask_positive);

    this->getROSParameter<double>(std::string("/omip/sensor_fps"), this->_sensor_fps);
    this->getROSParameter<int>(std::string("/omip/processing_factor"), this->_processing_factor);
    this->_loop_period_ns = 1e9/(this->_sensor_fps/(double)this->_processing_factor);

    // Defined also in feature_tracker_cfg.yaml
    std::map<int, std::string > mechanism_codes;
    mechanism_codes[0] = std::string("Automatically advancing, no waiting");
    mechanism_codes[1] = std::string("Manually advancing");
    mechanism_codes[2] = std::string("Wait for signal from Shape Reconstruction");
    mechanism_codes[3] = std::string("Wait for signal from Shape Tracker");

    if(this->_data_from_bag)
    {
        ROS_ERROR_STREAM_NAMED("FeatureTrackerNode.ReadParameters","Reading a rosbag!");
        ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadParameters",
                               "FeatureTrackerNode Parameters: " <<
                               "\n\tRosbag file: " << this->_bag_file_name <<
                               "\n\tType of advancing mechanism: " << mechanism_codes[this->_advance_frame_mechanism] <<
                               "\n\tMaximum time to wait for the signal to advance a frame (only used if advance_frame_mechanism is 2 or 3): " << this->_advance_frame_max_wait_time <<
                               "\n\tMinimum time to wait to advance a frame (only used if advance_frame_mechanism is 0): " << this->_advance_frame_min_wait_time/1e6 <<
                               "\n\tSubscribe to point cloud topic: " << this->_subscribe_to_pc <<
                               "\n\tSensor framerate: " << this->_sensor_fps << " fps (" << 1.f/this->_sensor_fps << " s)");
    }else{
        ROS_ERROR_STREAM_NAMED("FeatureTrackerNode.ReadParameters","Data from sensor!");
        ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadParameters",
                               "FeatureTrackerNode Parameters: " <<
                               "\n\tSubscribe to point cloud topic: " << this->_subscribe_to_pc <<
                               "\n\tSensor framerate: " << this->_sensor_fps << " fps (" << 1.f/this->_sensor_fps << " s)");
    }


    if(this->_subscribe_to_pc)
    {
        ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadParameters",
                               "FeatureTrackerNode subscribes to these topics: " <<
                               "\n\t" << this->_full_rgbd_pc_topic <<
                               "\n\t" << this->_depth_img_topic <<
                               "\n\t" << this->_rgb_img_topic <<
                               "\n\t" << this->_ci_topic <<
                               "\n\t" << this->_occlusion_mask_img_topic);
    }else{
        ROS_INFO_STREAM_NAMED( "FeatureTrackerNode.ReadParameters",
                               "FeatureTrackerNode subscribes to these topics: " <<
                               "\n\t" << this->_depth_img_topic <<
                               "\n\t" << this->_rgb_img_topic <<
                               "\n\t" << this->_ci_topic <<
                               "\n\t" << this->_occlusion_mask_img_topic);
    }
}

void FeatureTrackerNode::_InitializeVariables()
{
    if (this->_tracker_type == std::string("PointFeatureTracker"))
    {
        this->_re_filter = new PointFeatureTracker(this->_loop_period_ns,
                                                  this->_subscribe_to_pc,
                                                  this->_namespace);
        ((PointFeatureTracker*)this->_re_filter)->setSelfOcclusionPositive(_occlusion_mask_positive);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED( "FeatureTrackerNode.getROSParameter",
                                "This type of Visual Tracker (" << this->_tracker_type << ") is not defined");
        throw(std::string(
                    "[FeatureTrackerNode::getROSParameter] This type of Visual Tracker is not defined"));
    }

    //Set the previous time to zero
    this->_previous_measurement_time.fromSec(0.);

    // Get the CameraInfo message from the sensor (when not reading from a bag)
    if (!this->_data_from_bag)
    {
        // Waits a second to read one camera_info message from the camera driver, if not, returns an empty vector
        sensor_msgs::CameraInfoConstPtr camera_info = ros::topic::waitForMessage<
                sensor_msgs::CameraInfo>(this->_ci_topic, this->_measurements_node_handle,
                                         ros::Duration(2.0));
        if (camera_info)
        {
            this->_camera_info_msg = sensor_msgs::CameraInfo(*camera_info);
            this->_re_filter->setCameraInfoMsg(&this->_camera_info_msg);
            this->_ci_initialized = true;
            this->_camera_info_pub2.publish(this->_camera_info_msg);
        }
        else
        {
            ROS_ERROR_NAMED(
                        "FeatureTrackerNode.getROSParameter",
                        "CameraInfo message could not get read. Using default values to initialize the camera info!");
            this->_camera_info_msg.width = 640;
            this->_camera_info_msg.height = 480;
            this->_camera_info_msg.distortion_model = std::string("plumb_bob");
            for(int i=0; i<5; i++)
                this->_camera_info_msg.D.push_back(0.0);
            this->_camera_info_msg.K[0] = 520;
            this->_camera_info_msg.K[2] = 320;
            this->_camera_info_msg.K[4] = 520;
            this->_camera_info_msg.K[5] = 240;
            this->_camera_info_msg.K[8] = 1;
            this->_camera_info_msg.R[0] = 1;
            this->_camera_info_msg.R[4] = 1;
            this->_camera_info_msg.R[8] = 1;
            this->_camera_info_msg.P[0] = 520;
            this->_camera_info_msg.P[2] = 320;
            this->_camera_info_msg.P[5] = 520;
            this->_camera_info_msg.P[6] = 240;
            this->_camera_info_msg.P[10] = 1;
            this->_re_filter->setCameraInfoMsg(&this->_camera_info_msg);
            this->_ci_initialized = false;
            this->_camera_info_pub2.publish(this->_camera_info_msg);
        }
    }
}

void FeatureTrackerNode::_SubscribeAndAdvertiseTopics()
{
    // Setup the callback for the dynamic reconfigure
    this->_dr_callback = boost::bind(&FeatureTrackerNode::DynamicReconfigureCallback, this, _1, _2);

    //this->_dr_srv = new dynamic_reconfigure::Server<feature_tracker::feature_tracker_paramsConfig>();
    this->_dr_srv.setCallback(this->_dr_callback);

    // If we get the data from the kinect
    if (!this->_data_from_bag)
    {
        this->_depth_img_sub.subscribe(this->_measurements_node_handle, this->_depth_img_topic, 10);
        this->_rgb_img_sub.subscribe(this->_measurements_node_handle, this->_rgb_img_topic, 10);
        
        if (this->_subscribe_to_pc)
        {
            ROS_ERROR_STREAM_NAMED("FeatureTrackerNode._SubscribeAndAdvertiseTopics", "Using the HEAVY synchronization in FT (depth maps, RGB images and point clouds).");
            this->_full_rgbd_pc_sub.subscribe(this->_measurements_node_handle, this->_full_rgbd_pc_topic, 1);
            this->_synchronizer = new message_filters::Synchronizer<FTrackerSyncPolicy>(FTrackerSyncPolicy(1), this->_full_rgbd_pc_sub,
                                                                                        this->_depth_img_sub, this->_rgb_img_sub);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED("FeatureTrackerNode._SubscribeAndAdvertiseTopics", "Using the LIGHT synchronization in FT (depth maps and RGB images).");
            this->_light_synchronizer = new message_filters::Synchronizer<FTrackerLightSyncPolicy>(FTrackerLightSyncPolicy(10),
                                                                                                   this->_depth_img_sub, this->_rgb_img_sub);
        }
    }
    // If we get the data from a bag
    else
    {
        this->_time_repub = this->_measurements_node_handle.advertise<rosgraph_msgs::Clock>("clock", 100);
        if (this->_subscribe_to_pc)
        {
            this->_synchronizer =new message_filters::Synchronizer<FTrackerSyncPolicy>(FTrackerSyncPolicy(10),
                                                                                       this->_bag_full_rgbd_pc_sub,
                                                                                       this->_bag_depth_img_sub,
                                                                                       this->_bag_rgb_img_sub);
        }
        else
        {
            this->_light_synchronizer = new message_filters::Synchronizer<FTrackerLightSyncPolicy>(FTrackerLightSyncPolicy(10),
                                                                                                   this->_bag_depth_img_sub,
                                                                                                   this->_bag_rgb_img_sub);
        }
    }
    
    if (this->_subscribe_to_pc)
    {
        this->_synchronizer->registerCallback(boost::bind(&FeatureTrackerNode::measurementCallback, this, _1, _2, _3));
    }
    else
    {
        this->_light_synchronizer->registerCallback(boost::bind(&FeatureTrackerNode::measurementCallback, this, _1, _2));
    }
    
    this->_occlusion_mask_img_sub = this->_measurements_node_handle.subscribe(this->_occlusion_mask_img_topic, 1,
                                                                              &FeatureTrackerNode::OcclusionMaskImgCallback, this);
    
    this->_state_prediction_subscriber = this->_state_prediction_node_handles.at(0).subscribe(
                this->_predicted_locations_pc_topic,1,&FeatureTrackerNode::statePredictionCallback, this);
    
    this->_state_publisher = this->_measurements_node_handle.advertise<ft_state_ros_t>(this->_namespace + "/state", 100);
    this->_rgb_img_pub = this->_image_transport.advertise(this->_namespace + "/rgb_img", 100);
    this->_depth_img_pub = this->_image_transport.advertise(this->_namespace + "/depth_img", 100);
    this->_tracked_feats_img_pub = this->_image_transport.advertise(this->_namespace + "/tracked_feats_img", 100);
    this->_tracked_feats_with_pm_img_pub = this->_image_transport.advertise(this->_namespace + "/tracked_feats_with_pm_img", 100);
    this->_depth_edges_img_pub = this->_image_transport.advertise(this->_namespace + "/depth_edges_img", 100);
    this->_tracking_mask_img_pub = this->_image_transport.advertise(this->_namespace + "/tracking_mask_img", 100);
    this->_detecting_mask_img_pub = this->_image_transport.advertise(this->_namespace + "/detecting_mask_img", 100);
    this->_predicted_and_past_feats_img_pub = this->_image_transport.advertise(this->_namespace + "/pred_and_past_feats_img", 100);
    this->_prediction_mask_img_pub = this->_image_transport.advertise(this->_namespace + "/predicting_msk_img", 100);
    this->_camera_info_pub2 =this->_measurements_node_handle.advertise<sensor_msgs::CameraInfo>(this->_namespace + "/camera_info",100);

    this->_pred_feat_locs_repub = this->_measurements_node_handle.advertise<ft_state_ros_t>(this->_namespace + "/pred_feat_locs_repub" ,100);

    this->_camera_info_pub =this->_measurements_node_handle.advertise<sensor_msgs::CameraInfo>(this->_ci_topic,100);
    
    if(this->_data_from_bag )
    {
        this->_full_rgbd_pc_repub = this->_measurements_node_handle.advertise<sensor_msgs::PointCloud2>("camera/depth_registered/points",100);
        this->_tf_repub = this->_measurements_node_handle.advertise<tf::tfMessage>("tf",100);
        this->_shutdown_publisher = this->_measurements_node_handle.advertise<std_msgs::Empty>("/omip/shutdown",100);
    }


    // Wait for refinements of the pose coming from a shape tracker node
    if(this->_advance_frame_mechanism == 2)
    {
        ROS_INFO("Waiting for ShapeReconstruction to advance");
        this->_advance_sub = this->_state_prediction_node_handles.at(0).subscribe("segmentation_info_msg", 1,
                                                                                  &FeatureTrackerNode::AdvanceBagCallbackFromShapeReconstruction, this);
    }else if(this->_advance_frame_mechanism == 3){
        ROS_INFO("Waiting for ShapeTracker to advance");
        this->_advance_sub = this->_state_prediction_node_handles.at(0).subscribe("/shape_tracker/state", 1,
                                                                                  &FeatureTrackerNode::AdvanceBagCallbackFromShapeTracker, this);
    }
}

void FeatureTrackerNode::AdvanceBagCallbackFromShapeReconstruction(const boost::shared_ptr<std_msgs::Bool const> &flag)
{
    ROS_DEBUG_STREAM("Advance flag from ShapeReconstruction received in FeatureTracker!");
    this->_advance_sub_returned_true = true;
}

void FeatureTrackerNode::AdvanceBagCallbackFromShapeTracker(const boost::shared_ptr<omip_msgs::ShapeTrackerStates const> &st_states)
{
    ROS_DEBUG_STREAM("Advance flag from ShapeTracker received in FeatureTracker!");
    this->_advance_sub_returned_true = true;
}

void FeatureTrackerNode::run()
{
    if (this->_data_from_bag)
    {
        for(int idx = 0; idx < this->_num_external_state_predictors; idx++)
        {
            // Create a thread that spins on the callback queue of the predictions
            this->_state_predictor_listener_threads.push_back(new boost::thread(boost::bind(&RecursiveEstimatorNodeInterface::spinStatePredictorQueue, this, idx)));
        }
        this->ReadRosBag();
    }else{
        RecursiveEstimatorNodeInterface::run();
    }
}

// Main program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_tracker");
    FeatureTrackerNode ft_node;

    bool press_enter_to_start = false;
    ft_node.getROSParameter<bool>("/omip/press_enter_to_start", press_enter_to_start);
    if(press_enter_to_start)
    {
        std::cout << "************************************************************************" << std::endl;
        std::cout << "Press enter to start Online Interactive Perception" << std::endl;
        std::cout << "************************************************************************" << std::endl;
        getchar();
    }else{
        std::cout << "Starting Online Interactive Perception" << std::endl;
    }

    ft_node.run();
    
    return (0);
}
