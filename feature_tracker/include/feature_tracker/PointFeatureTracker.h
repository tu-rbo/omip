/*
 * PointFeatureTracker.h
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

#ifndef REAL_FEATURE_TRACKER_H_
#define REAL_FEATURE_TRACKER_H_

#include "feature_tracker/FeatureTracker.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/gpumat.hpp>
//#include <opencv2/gpu/gpu.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <fstream>

namespace omip
{

/**
 * Class PointFeatureTracker
 * It implements the interface FeatureTracker
 * Tracks features in the RGB stream and returns a PCL point cloud with their new 3D coordinates
 */
class PointFeatureTracker : public FeatureTracker
{
public:

    /**
   * Constructor
   */
    PointFeatureTracker(double loop_period_ns, bool using_pc = false, std::string ft_ns = std::string(""));

    /**
   * Destructor
   */
    virtual ~PointFeatureTracker();

    /**
     * @brief First step when updating the filter. The next state is predicted from current state and system model (part of the RecursiveEstimatorInterace)
     * In the FeatureTracker the system model is the most simple one: no change in the state. So the predicted state is just the current state.
     *
     */
    virtual void predictState(double time_interval_ns);

    /**
     * @brief Second step when updating the filter. The next measurement is predicted from the predicted next state (part of the RecursiveEstimatorInterface)
     *
     * In the FeatureTracker it is not clear what are the measurements:
     *      - From one side, the measurements are the RGB-D frames coming from the sensor. But we cannot predict entire RGB-D frames from 3D feature locations!
     *      - From the other side, the measurements are the 2D locations of the tracked features because we can predict them by projecting the predicted state(s)
     *      into the image plane.
     */
    virtual void predictMeasurement();

    /**
     * @brief Corrects the predicted state(s). First we track the features using both the prediction with the system model (copy of the previous feature locations)
     * and if there is a prediction from the higher level (RBT) we track features using it too. We then decide which prediction is the "most likely" based on the
     * quality measurement that the feature tracker returns and use the new location to update the belief state
     *
     */
    virtual void correctState();

    /**
     * @brief Add a new prediction about the state generated in the higher level (as prediction about the
     * next measurement)
     *
     * @param predicted_state Predicted next state
     */
    virtual void addPredictedState(const ft_state_t& predicted_state, const double& predicted_state_timestamp_ns);

    /**
     * @brief Set the messages that have to be estimated and published
     *
     * @param config Value from Dynamic Reconfigure
     */
    virtual void setDynamicReconfigureValues(feature_tracker::FeatureTrackerDynReconfConfig &config);


    /**
   * Set a new RGBD point cloud to use for tracking in 3D. The RGBD point cloud is used
   * to retrieve the 3D coordinates of the features tracked in 2D
   * @param full_rgb_pc - Shared pointer to the new RGBD point cloud
   */
    virtual void setFullRGBDPC(PointCloudPCL::ConstPtr full_rgb_pc);

    /**
   * Set a new occlusion mask image for tracking in 3D. The occlusion mask predicts the
   * area of the RGB image that will be occluded (where it will be visible) the arm of the
   * robot when it executes an interaction. Features on this area will be rejected.
   * @param occ_mask_img - Shared pointer to the new occlusion mask image
   */
    virtual void setOcclusionMaskImg(cv_bridge::CvImagePtr occ_mask_img);

    /**
   * Set the camera parameters to be used to project the 3D predicted Feature locations to the image plane and
   * convert them into image coordinates
   * @param camera_info - Pointer to the CameraInfo message containing the camera parameters
   */
    virtual void setCameraInfoMsg(const sensor_msgs::CameraInfo* camera_info);

    /**
   * Get a point cloud with the 3D locations and the unique Feature Id (L value) of the last
   * tracked features
   * @return - Shared pointer to the 3D XYZL point cloud of tracked Features
   */
    virtual ft_state_t getState() const;

    virtual cv_bridge::CvImagePtr getRGBImg();

    virtual cv_bridge::CvImagePtr getDepthImg();

    /**
    * Get the last tracked Features as marks in the RGB image
    * @return - Shared pointer to the image with the 2D tracked Feature locations marked
    */
    virtual cv_bridge::CvImagePtr getTrackedFeaturesImg();

    /**
    * Get a pointer to a RGB image with markers where the Features have been tracked in the last step
    * and only showing the RGB area where the features are searched
    * @return - Shared pointer to the RGB image with markers and searching areas
    */
    virtual cv_bridge::CvImagePtr getTrackedFeaturesWithPredictionMaskImg();

    /**
    * Get the image with the depth edges, used to reject Features that are on edges
    * @return - Shared pointer to the image of depth edges
    */
    virtual cv_bridge::CvImagePtr getDepthEdgesImg();

    /**
    * Get the image with the depth edges, used to reject Features that are on edges
    * @return - Shared pointer to the image of depth edges
    */
    virtual cv_bridge::CvImagePtr getPredictingMaskImg();

    /**
    * Get the mask used to track features. Combines edges, predictions mask and max depth
    * @return - Shared pointer to the image of tracking mask
    */
    virtual cv_bridge::CvImagePtr getTrackingMaskImg();

    /**
    * Get the mask used to detect new features. Combines edges, max depth and mask of current features (to not detect them)
    * @return - Shared pointer to the image of detecting mask
    */
    virtual cv_bridge::CvImagePtr getDetectingMaskImg();

    /**
    * Get the image with the last feature locations and the predicted feature locations
    * @return - Shared pointer to the image of the predicted and last features
    */
    virtual cv_bridge::CvImagePtr getPredictedAndLastFeaturesImg();

    /**
     * @brief Set if the self occlusion image will be black in the background and white for the occluded area (so_positive = true, new)
     * or white in the background and black in the occluded area (so_positive = false, old)
     *
     * @param so_positive Flag value
     */
    virtual void setSelfOcclusionPositive(bool so_positive);


protected:

    /**
   * Read parameters from ROS
   */
    void _ReadParameters();

    /**
   * Initialize matrices and variables used during the execution
   */
    void _InitializeVariables();

    void _EstimateDetectionMask();

    void _DetectNewFeatures();

    /**
   * Track the features that were detected previously and detect new features if needed
   */
    void _TrackFeatures();

    /**
   * Process the previously received depth img
   */
    void _ProcessDepthImg();

    /**
   * Process the previously received rgb img
   */
    void _ProcessRGBImg();

    /**
   * Process the previously received occlusion mask img
   */
    void _ProcessOcclusionMaskImg();

    /**
   * Process the previously received predicted locations pointcloud
   */
    void _ProcessPredictedLocationsPC();

    /**
     * @brief _estimateColorUncertainty Estimates the uncertainty of the color channel (and the tracker)
     */
    void _estimateColorUncertainty();

    /**
   * Retrieve the 3D coordinates of a set of 2D Features
   * @param points_in_2d - 2D locations of the Features in image plane
   * @param points_ids - ID of the Features
   * @param points_status - Status of the Features: If the 3D coordinates of a Feature cannot be retrieved, its status
   * changes to false
   * @param points_in_3d - Returning vector with the 3D coordinates and the Id of the Features
   * @return - Number of 2D Features whose 3D coordinates couldn't be retrieved (rejected)
   */
    int _Retrieve3DCoords(
            std::vector<cv::Point2f>& points_in_2d, std::vector<bool>& points_status,
            std::vector<cv::Point3d>& points_in_3d);

    /**
   * Compare current and previous 3D coordinates of the tracked Features to reject jumping ones. This
   * limits the maximum velocity of a pixel in 3D space.
   * 3D locations is too large) its status changes to false
   * @return - Number of 3D Features that jumped (rejected)
   */
    int _Compare3DCoords();

    ros::NodeHandle _node_handle;
    sensor_msgs::CameraInfo::Ptr _camera_info_ptr;

    // PCL variables
    PointCloudPCL::ConstPtr _received_point_cloud;

    // OpenCV variables
    cv_bridge::CvImagePtr _depth_img_ptr, _rgb_img_ptr, _current_bw_img_ptr, _previous_bw_img_ptr, _occlusion_msk_img_ptr;
    cv_bridge::CvImagePtr _tracked_features_img, _tracked_features_with_pred_msk_img, _depth_edges_img, _predicting_mask_img;
    cv_bridge::CvImagePtr _tracking_mask_img, _detecting_mask_img, _predicted_and_previous_features_img;
    cv::Mat _predicted_feats_msk_mat, _depth_mat, _canny_edges_mat, _feats_detection_msk_mat, _feats_tracking_msk_mat;
    cv::Mat _erosion_element_detecting_mat, _erosion_element_tracking_mat, _occlusion_msk_mat, _aux1_mat, _previous_depth_mat, _depth_difference_mask;
    cv::Mat _hsv_current_img, _downsampled_bgr;
    double _current_mean_sat, _previous_mean_sat;
    double _current_mean_val, _previous_mean_val;
    int _color_uncertainty_decay;

    std::vector<std::vector<cv::Point2f> >  _predicted_measurements;
    std::vector<cv::Point2f> _previous_measurement, _corrected_measurement, _combined_previous, _combined_predictions;
    std::vector<cv::Point3d> _previous_belief_state, _corrected_belief_state;
    std::vector<bool> _feat_status_v;
    std::vector<int> _previous_feat_ids_v, _current_feat_ids_v;
    std::vector<int> _received_prediction_ids;

    ros::Time _predicted_state_rcvd_time, _previous_time_of_motion_detection;
    double _predicted_state_rcvd_time_ns;

    // Flags
    bool _using_pc, _first_frame, _occlusion_msk_img_rcvd, _predicted_state_rcvd, _predicted_state_processed, _camera_info_msg_rcvd;
    bool _publishing_predicted_and_past_feats_img, _publishing_tracked_feats_img, _publishing_tracked_feats_with_pred_msk_img;
    bool _attention_to_motion, _use_motion_mask;

    // Parameters
    int _erosion_size_detecting, _erosion_size_tracking, _canny_kernel_size;
    int _canny_ratio, _number_of_features, _min_number_of_features, _frame_counter, _processing_factor, _min_area_size_pixels;
    double _max_allowed_depth, _min_allowed_depth, _canny_low_threshold, _max_interframe_jump, _sensor_fps, _min_time_to_detect_motion, _min_depth_difference, _min_feat_quality;

    Feature::Id _feature_id_counter;
    std::string _ft_ns;

    std::vector<float> _feat_quality;

    std::ofstream _features_file;

    bool _so_positive;

    double _min_sensor_noise_x, _min_sensor_noise_y, _min_sensor_noise_z;
    double _sensor_noise_model_a, _sensor_noise_model_b, _sensor_noise_model_c;
    double _color_uncertainty;
};
}

#endif /* REAL_FEATURE_TRACKER_H_ */
