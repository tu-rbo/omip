/*
 * FeatureTrackerNode.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014).
 * This implementation can be used to reproduce the results of the paper and to be applied to new research.
 * The implementation allows also to be extended to perceive different information/models or to use additional sources of information.
 * A detail explanation of the method and the system can be found in our paper.
 *
 * If you are using this implementation in your research, please consider citing our work:
 *
@inproceedings{martinmartin_ip_iros_2014,
Title = {Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors},
Author = {Roberto {Mart\'in-Mart\'in} and Oliver Brock},
Booktitle = {Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems},
Pages = {2494-2501},
Year = {2014},
Location = {Chicago, Illinois, USA},
Note = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Url = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Projectname = {Interactive Perception}
}
 * If you have questions or suggestions, contact us:
 * roberto.martinmartin@tu-berlin.de
 *
 * Enjoy!
 */

#ifndef FEATURE_TRACKER_NODE_H_
#define FEATURE_TRACKER_NODE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <tf/tf.h>

#include "feature_tracker/FeatureTracker.h"

#include <boost/thread.hpp>

#include <omip_common/RecursiveEstimatorNodeInterface.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <feature_tracker/FeatureTrackerDynReconfConfig.h>

#include <std_msgs/Bool.h>
#include "omip_msgs/ShapeTrackerStates.h"

namespace omip
{
/**
 * BagSubscriber class
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 * This class is used to synchronize messages from a bag
 */
template<class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        this->signalMessage(msg);
    }
};

/**
 * @brief FeatureTrackerNode class
 * Connects to the ROS communication system to get messages from the sensors and publish results
 * Templated in the state of to be estimated by this RE level (ROS type)
 * The received messages are passed to the FeatureTracker object to be processed
 *
 */
class FeatureTrackerNode :
        public RecursiveEstimatorNodeInterface<ft_measurement_ros_t, ft_state_ros_t, FeatureTracker>
{
    // Policies to synchorize point clouds, depth and RGB images from the sensor
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> FTrackerSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> FTrackerLightSyncPolicy;

public:

    /**
     * Constructor
     */
    FeatureTrackerNode();

    virtual void AdvanceBagCallbackFromShapeReconstruction(const boost::shared_ptr<std_msgs::Bool const> &flag);

    virtual void AdvanceBagCallbackFromShapeTracker(const boost::shared_ptr<omip_msgs::ShapeTrackerStates const> &st_states);

    /**
     * Destructor
     */
    virtual ~FeatureTrackerNode();

    /**
     * @brief Combined callback for the measurements of this RE level: point cloud, depth image and rgb image
     *
     * @param pc_msg Input full RGBD point cloud
     * @param depth_image_msg Input depth image
     * @param rgb_image_msg Input RGB image
     */
    virtual void measurementCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg,
                          const sensor_msgs::ImageConstPtr &depth_image_msg,
                          const sensor_msgs::ImageConstPtr &rgb_image_msg);

    /**
     * @brief Lighter version of the combined callback for the measurements of this RE level: depth image and rgb image
     *
     * @param depth_image_msg Input depth image
     * @param rgb_image_msg Input RGB image
     */
    virtual void measurementCallback(const sensor_msgs::ImageConstPtr &depth_image_msg,
                               const sensor_msgs::ImageConstPtr &rgb_image_msg);

    /**
     * @brief Empty measurement callback, just to implement correctly the RENode interface
     *
     * @param ft_measurement_ros Empty message. Only to implement the interface
     */
    virtual void measurementCallback(const boost::shared_ptr<ft_measurement_ros_t const> &ft_measurement_ros){}

    /**
     * @brief Callback for the predictions about the state of this RE level coming from the higher level of the hierarchy
     *
     * @param predicted_next_state Predictions about the next state (next feature 3D locations) from the RBT
     */
    virtual void statePredictionCallback(const boost::shared_ptr<ft_state_ros_t const> &predicted_next_state);

    /**
     * @brief Callback for the Dynamic Reconfigure parameters
     *
     * @param config Values from the Dynamic Reconfigure server
     * @param level
     */
    virtual void DynamicReconfigureCallback(feature_tracker::FeatureTrackerDynReconfConfig &config, uint32_t level);

    /**
     * @brief Read messages from a rosbag instead of receiving them from a sensor
     * Internally it "publishes" the messages that are then received by the measurement callback
     *
     */
    virtual void ReadRosBag();

    /**
     * @brief Callback for the mask of the self occlusion of the robot. Only for the WAM
     *
     * @param occlusion_mask_img Mask image with the area of the visual field covered by the WAM robot
     */
    virtual void OcclusionMaskImgCallback(const sensor_msgs::ImageConstPtr &occlusion_mask_img);

    /**
     * @brief Publish an image with the last tracked features on it
     *
     */
    virtual void PublishTrackedFeaturesImg();

    /**
     * @brief Publish an image with the last tracked features together with the mask of the predictions
     *
     */
    virtual void PublishTrackedFeaturesWithPredictionMaskImg();

    /**
     * @brief Publish an image with the edges detected on the depth map
     *
     */
    virtual void PublishDepthEdgesImg();

    /**
     * @brief Publish the mask of the predictions
     *
     */
    virtual void PublishPredictionMaskImg();

    /**
     * @brief Publish the mask used to track features. Combines edges, predictions mask and max depth
     *
     */
    virtual void PublishTrackingMaskImg();

    /**
     * @brief Publish the mask used to detect new features. Combines edges, max depth and mask of current features (to not detect them)
     *
     */
    virtual void PublishDetectingMaskImg();

    /**
     * @brief Publish the full RGB-D point cloud used in the last iteration (used to synchronize the PC to the other published messages)
     *
     */
    virtual void PublishFullRGBDPC();

    virtual void PublishRGBImg();

    virtual void PublishDepthImg();

    /**
     * @brief Publish an image with the last feature locations and the predicted feature locations
     *
     */
    virtual void PublishPredictedAndLastFeaturesImg();

    virtual void RepublishPredictedFeatLocation();

    virtual void run();

protected:

    /**
     * @brief Publish the current state of this RE level
     *
     */
    virtual void _publishState() const;

    /**
     * @brief Publish the prediction about the next measurement by this RE level
     *
     */
    virtual void _publishPredictedMeasurement() const;

    /**
     * Read parameters from configuration file
     */
    virtual void _ReadParameters();

    /**
     * Initialize variables
     */
    virtual void _InitializeVariables();

    /**
     * Subscribe and advertise topics for this node
     */
    virtual void _SubscribeAndAdvertiseTopics();

    image_transport::ImageTransport _image_transport;

    // Subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::PointCloud2> _full_rgbd_pc_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_img_sub;
    message_filters::Subscriber<sensor_msgs::Image> _rgb_img_sub;
    message_filters::Synchronizer<FTrackerSyncPolicy> *_synchronizer;
    message_filters::Synchronizer<FTrackerLightSyncPolicy> *_light_synchronizer;
    ros::Subscriber _occlusion_mask_img_sub;

    // Fake subscribers to capture images
    BagSubscriber<sensor_msgs::Image> _bag_rgb_img_sub;
    BagSubscriber<sensor_msgs::Image> _bag_depth_img_sub;
    BagSubscriber<sensor_msgs::PointCloud2> _bag_full_rgbd_pc_sub;
    ros::Publisher _shutdown_publisher;

    // Publishers
    ros::Publisher _full_rgbd_pc_repub;
    ros::Publisher _camera_info_pub;
    ros::Publisher _camera_info_pub2;
    ros::Publisher _tf_repub;

    ros::Publisher _time_repub;

    ros::Publisher _pred_feat_locs_repub;


    image_transport::Publisher _depth_img_pub;
    image_transport::Publisher _rgb_img_pub;
    image_transport::Publisher _tracked_feats_img_pub;
    image_transport::Publisher _tracked_feats_with_pm_img_pub;
    image_transport::Publisher _depth_edges_img_pub;
    image_transport::Publisher _prediction_mask_img_pub;
    image_transport::Publisher _tracking_mask_img_pub;
    image_transport::Publisher _detecting_mask_img_pub;
    image_transport::Publisher _predicted_and_past_feats_img_pub;

    sensor_msgs::CameraInfo _camera_info_msg;

    // PCL variables
    PointCloudPCL::ConstPtr _full_rgbd_pc;
    FeatureCloudPCLwc::Ptr _predicted_locations_pc;

    // OpenCV variables
    cv_bridge::CvImagePtr _cv_ptr_depth;
    cv_bridge::CvImagePtr _cv_ptr_rgb;
    cv_bridge::CvImagePtr _cv_ptr_occlusion_msk;

    // Flags
    bool _data_from_bag;
    bool _subscribe_to_pc;
    bool _ci_initialized;
    bool _publishing_depth_edges_img;
    bool _publishing_rgb_img;
    bool _publishing_depth_img;
    bool _publishing_tracked_feats_img;
    bool _publishing_tracked_feats_with_pred_msk_img;
    bool _publishing_full_pc;
    bool _publishing_predicting_msk_img;
    bool _publishing_tracking_msk_img;
    bool _publishing_detecting_msk_img;
    bool _publishing_predicted_and_past_feats_img;
    bool _republishing_predicted_feat_locs;

    bool _occlusion_mask_positive;

    bool _attention_to_motion;

    double _sensor_fps;
    double _loop_period_ns;
    int _processing_factor;

    std::string _ft_ns;

    std::string _tracker_type;
    std::string _bag_file_name;
    std::string _full_rgbd_pc_topic;
    std::string _depth_img_topic;
    std::string _rgb_img_topic;
    std::string _occlusion_mask_img_topic;
    std::string _ci_topic;
    std::string _predicted_locations_pc_topic;

    tf::Transform  _initial_ee_tf;

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<feature_tracker::FeatureTrackerDynReconfConfig> _dr_srv;
    dynamic_reconfigure::Server<feature_tracker::FeatureTrackerDynReconfConfig>::CallbackType _dr_callback;

    double _advance_frame_min_wait_time;
    double _advance_frame_max_wait_time;

    ros::Subscriber _advance_sub;
    bool _advance_sub_returned_true;

    int _advance_frame_mechanism;
};
}

#endif /* FEATURE_TRACKER_NODE_H_ */
