#include "feature_tracker/PointFeatureTracker.h"

#include "feature_tracker/FeatureTracker.h"

#include <pcl_ros/publisher.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <pcl_conversions/pcl_conversions.h>

#if ROS_VERSION_MINIMUM(1, 11, 1) // if current ros version is >= 1.10.1 (hydro)
#else
#include "feature_tracker/pcl_conversions_indigo.h"
#endif

#define EROSION_SIZE_DETECTING 15
#define EROSION_SIZE_TRACKING 3

#define MAXIMUM_TIME_TO_USE_A_PREDICTION 0.05

using namespace omip;

PointFeatureTracker::PointFeatureTracker(double loop_period_ns, bool using_pc, std::string ft_ns) :
    FeatureTracker(loop_period_ns),
    _using_pc(using_pc),
    _first_frame(true),
    _occlusion_msk_img_rcvd(false),
    _predicted_state_rcvd(false),
    _predicted_state_processed(true),
    _camera_info_msg_rcvd(false),
    _erosion_size_detecting(-1),
    _erosion_size_tracking(-1),
    _canny_kernel_size(-1),
    _canny_ratio(-1),
    _number_of_features(-1),
    _min_number_of_features(-1),
    _frame_counter(1),
    _feature_id_counter(1),
    _max_allowed_depth(-1.f),
    _min_allowed_depth(-1.f),
    _canny_low_threshold(-1),
    _max_interframe_jump(0.f),
    _ft_ns(ft_ns),
    _publishing_predicted_and_past_feats_img(false),
    _publishing_tracked_feats_img(false),
    _publishing_tracked_feats_with_pred_msk_img(false),
    _sensor_fps(0.0),
    _processing_factor(0),
    _attention_to_motion(false),
    _use_motion_mask(false),
    _min_feat_quality(0.001),
    _predicted_state_rcvd_time_ns(0),
    _so_positive(false)
{
    this->_ReadParameters();
    this->_InitializeVariables();

    _features_file.open("features.txt");
}

PointFeatureTracker::~PointFeatureTracker()
{
    _features_file.close();
}

void PointFeatureTracker::setDynamicReconfigureValues(feature_tracker::FeatureTrackerDynReconfConfig &config)
{
    this->_publishing_predicted_and_past_feats_img = config.pub_predicted_and_past_feats_img;
}

void PointFeatureTracker::predictState(double time_interval_ns)
{
    // The predicted state based on current state is the current state itself (based on continuity prior: the features in
    // current step will be very close to where they were in the step before)
    this->_predicted_states[0] = this->_state;
}

void PointFeatureTracker::predictMeasurement()
{
    // We have two predictions for the measurements, one using each predicted state

    // First predicted measurement:
    // Uses the first predicted state, which is the same that the previous state (no feature motion)
    // Therefore, we could project the 3D features back to the image, but for the sake of accuracy and computation time, we just take
    // the previous "refined" measurement
    this->_predicted_measurements[0] = this->_corrected_measurement;

    // Second predicted measurement:
    // If we received from the higher level (RBT) a prediction about the next feature 3D locations, we project them to the image plane

    // By copying the previous vector of 2D feature locations we assure an initial value for tracking the features
    this->_predicted_measurements[1] = this->_corrected_measurement;

    if (!this->_first_frame && this->_camera_info_msg_rcvd && this->_predicted_states.size() > 1 && this->_predicted_states[1] &&
            pcl_conversions::fromPCL((double)this->_predicted_states[1]->header.stamp).toNSec() - this->_measurement_timestamp_ns < this->_loop_period_ns/2.0)
    {
        // Initialize variables
        this->_received_prediction_ids.clear();
        this->_predicted_feats_msk_mat = cv::Scalar(0);
        Eigen::Vector4d predicted_feat_3d_eigen = Eigen::Vector4d(0., 0., 0., 1.);
        Eigen::Vector3d predicted_feat_2d_eigen = Eigen::Vector3d(0., 0., 0.);
        cv::Point predicted_feat_2d_cv_int = cvPoint(0, 0);
        cv::Point2f predicted_feat_2d_cv_f = cvPoint2D32f(0.f, 0.f);
        double predicted_distance_to_sensor = 0.;
        int correction_search_area_radius = 0;

        BOOST_FOREACH(FeaturePCLwc predicted_feat_3d_pcl, this->_predicted_states[1]->points)
        {
            // Find the index of this feature in the vector of current features
            int predicted_feat_idx = std::find(this->_current_feat_ids_v.begin(), this->_current_feat_ids_v.end(), predicted_feat_3d_pcl.label) - this->_current_feat_ids_v.begin();

            // There is a current point with this idx
            if (predicted_feat_idx < this->_number_of_features)
            {
                this->_received_prediction_ids.push_back(predicted_feat_3d_pcl.label);
                // Convert the predicted feature from PCL point into Eigen vector homogeneous coordinates
                predicted_feat_3d_eigen.x() = predicted_feat_3d_pcl.x;
                predicted_feat_3d_eigen.y() = predicted_feat_3d_pcl.y;
                predicted_feat_3d_eigen.z() = predicted_feat_3d_pcl.z;

                // Estimate the distance of the predicted feature to the sensor
                predicted_distance_to_sensor = sqrt(predicted_feat_3d_pcl.x * predicted_feat_3d_pcl.x +
                                                    predicted_feat_3d_pcl.y * predicted_feat_3d_pcl.y +
                                                    predicted_feat_3d_pcl.z * predicted_feat_3d_pcl.z);

                // Project the predicted feature from 3D space into the image plane
                predicted_feat_2d_eigen = this->_image_plane_proj_mat_eigen * predicted_feat_3d_eigen;

                // Convert the predicted feature from image coordinates to pixel coordinates
                predicted_feat_2d_cv_f.x = predicted_feat_2d_eigen.x() / predicted_feat_2d_eigen.z();
                predicted_feat_2d_cv_f.y = predicted_feat_2d_eigen.y() / predicted_feat_2d_eigen.z();

                // Estimate the pixel (round the float pixel coordinates to the closest integer value)
                predicted_feat_2d_cv_int.x = cvRound(predicted_feat_2d_cv_f.x);
                predicted_feat_2d_cv_int.y = cvRound(predicted_feat_2d_cv_f.y);

                // If the predicted feature is inside the image limits
                // 1: We store this feature as prediction to help the tracker (it will use this locations as starting point)
                // 2: We clear a circular area around its location in the tracking mask. The tracker (as part of the correction of the predicted state)
                // will search for the feature only in this area of the image
                if (predicted_feat_2d_cv_int.x >= 0 && predicted_feat_2d_cv_int.x < this->_camera_info_ptr->width
                        && predicted_feat_2d_cv_int.y >= 0 && predicted_feat_2d_cv_int.y < this->_camera_info_ptr->height)
                {
                    this->_predicted_measurements[1][predicted_feat_idx] = predicted_feat_2d_cv_int;
                    // The radius of the circle in the mask is inversely proportional to the depth of the predicted location
                    // Motion of close 3-D points (small depth) will translate in large motion in image plane -> large RGB searching area for
                    // the feature
                    // Motion of distanced 3-D points (large depth) will translate in smaller motion in image plane -> small RGB searching area
                    // for the feature
                    // TODO: There are two other variables that should be taken into account when defining the searching area:
                    //        1: Uncertainty in the RBM: If the motion is very uncertain, the searching area should be larger. If the motion is
                    //        very certain, the feature location should be very close to the predicted one
                    //        2: Velocity of the RB: Large velocities are more difficult to track, and thus, the searching area should be larger.
                    correction_search_area_radius = std::max( 3, (int)(8.0 / predicted_distance_to_sensor));
                    cv::circle(this->_predicted_feats_msk_mat, predicted_feat_2d_cv_int, correction_search_area_radius, CV_RGB(255, 255, 255), -1, 8, 0);

                }
            }
        }
    }
}

void PointFeatureTracker::correctState()
{
    this->_ProcessOcclusionMaskImg();

    this->_ProcessDepthImg();

    this->_ProcessRGBImg();

    if (this->_first_frame)
    {
        // Detect new features in the image
        this->_DetectNewFeatures();
        this->_first_frame = false;
    }
    else
    {
        this->_TrackFeatures();
    }

    // Copy the result
    this->_previous_measurement = this->_corrected_measurement;

    this->_previous_feat_ids_v = this->_current_feat_ids_v;
    this->_previous_bw_img_ptr = this->_current_bw_img_ptr;
    this->_previous_belief_state = this->_corrected_belief_state;

    for (size_t num_feats = 0; num_feats < this->_number_of_features; num_feats++)
    {
        this->_state->points[num_feats].label = this->_current_feat_ids_v[num_feats];
        this->_state->points[num_feats].x = this->_corrected_belief_state[num_feats].x;
        this->_state->points[num_feats].y = this->_corrected_belief_state[num_feats].y;
        this->_state->points[num_feats].z = this->_corrected_belief_state[num_feats].z;

        // Covariance
        //
    }
    this->_frame_counter++;
}

void PointFeatureTracker::addPredictedState(const ft_state_t &predicted_state, const double& predicted_state_timestamp_ns)
{
    this->_predicted_states[1] = predicted_state;
    this->_predicted_state_rcvd_time_ns = predicted_state_timestamp_ns;
}

void PointFeatureTracker::setFullRGBDPC(PointCloudPCL::ConstPtr full_rgb_pc)
{
    this->_received_point_cloud = full_rgb_pc;
}

void PointFeatureTracker::setOcclusionMaskImg(cv_bridge::CvImagePtr occ_mask_img)
{
    this->_occlusion_msk_img_ptr = occ_mask_img;
    this->_occlusion_msk_img_rcvd = true;
}

void PointFeatureTracker::_ProcessDepthImg()
{
    this->_depth_img_ptr = this->_measurement.second;

    // Convert from sensor_msg to cv image
    // 1) Create matrices
    this->_depth_mat = cv::Mat(this->_depth_img_ptr->image.rows, this->_depth_img_ptr->image.cols, CV_8UC1);
    this->_aux1_mat = cv::Mat(this->_depth_img_ptr->image.rows, this->_depth_img_ptr->image.cols, CV_8UC1);

    // 2) Find the minimum and maximum distances
    // Exclude points that contain infinity values
    double minVal, maxVal;
    cv::Point minpt,maxpt;

    cv::minMaxLoc(this->_depth_img_ptr->image, &minVal, &maxVal, &minpt, &maxpt,  this->_depth_img_ptr->image == this->_depth_img_ptr->image);

    // If we set a ROI we use it
    if(this->_max_allowed_depth > 0)
    {
        maxVal = std::min(maxVal, this->_max_allowed_depth);
    }

    //TODO: Sometimes there are noisy points very close to the sensor. If we use the minipum in the image we will span
    // the values to this minimum value -> ignore it
    // If we set a ROI we use it
    if(this->_min_allowed_depth > 0)
    {
        minVal = std::max(this->_min_allowed_depth, minVal);
    }

    this->_depth_img_ptr->image.convertTo(this->_aux1_mat, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    this->_aux1_mat.setTo(0, this->_depth_img_ptr->image > maxVal);

    this->_aux1_mat.setTo(0, this->_depth_img_ptr->image < minVal);


    // Filter OUT large distances and disappeared points and edges
    // _depth_map contains 255 for the values that are not 0
    cv::threshold(this->_aux1_mat, this->_depth_mat, 0, 255, cv::THRESH_BINARY);

    // Reduce noise with a kernel 3x3
    cv::blur(this->_aux1_mat, this->_aux1_mat, cv::Size(3, 3));


    // Detect Canny edges
    cv::Canny(this->_aux1_mat, this->_canny_edges_mat,
              this->_canny_low_threshold,
              this->_canny_low_threshold * this->_canny_ratio,
              this->_canny_kernel_size, false);


    // Invert the resulting edges: edges are 0, not edges are 255
    cv::threshold(this->_canny_edges_mat, this->_canny_edges_mat, 0, 255, cv::THRESH_BINARY_INV);

    cv::bitwise_and(this->_depth_mat, this->_canny_edges_mat,this->_feats_detection_msk_mat);

    // If the occlusion mask was received, we merge it with the estimated mask
    if (this->_occlusion_msk_img_rcvd)
    {
        this->_feats_detection_msk_mat = this->_feats_detection_msk_mat & _occlusion_msk_mat;
    }else{
    }


    // Erode EROSION_SIZE_TRACKING pixel for the tracking mask
    cv::erode(this->_feats_detection_msk_mat, this->_feats_tracking_msk_mat, this->_erosion_element_tracking_mat);

    for (int n = 0; n < this->_corrected_measurement.size(); n++)
    {
        if (this->_feat_status_v[n])
        {
            cv::circle(this->_feats_detection_msk_mat, this->_corrected_measurement[n], 2, CV_RGB(0, 0, 0), -1, 8, 0);
        }
    }

    // Erode EROSION_SIZE_DETECTING pixel for the detecting mask
    cv::erode(this->_feats_detection_msk_mat, this->_feats_detection_msk_mat, this->_erosion_element_detecting_mat);

    // Focus the detection of new features into the areas of the image where the depth changed
    // TODO: Detect also areas of color changes
    if(this->_attention_to_motion)
    {
        // First time
        if(this->_previous_time_of_motion_detection == ros::Time(0))
        {
            this->_previous_depth_mat = this->_depth_img_ptr->image;
            this->_previous_time_of_motion_detection = ros::Time::now();
        }
        else
        {
            if((ros::Time::now() - this->_previous_time_of_motion_detection).toSec() > this->_min_time_to_detect_motion)
            {
                cv::Mat depth_difference;

                // Absolute difference between the previous and the current depth maps
                cv::absdiff(this->_depth_img_ptr->image, this->_previous_depth_mat, depth_difference);

                // Threshold the absolute difference to the minimum depth to consider motion
                if(this->_depth_img_ptr->encoding == "32FC1")
                {
                    this->_depth_difference_mask = (depth_difference > (double)(this->_min_depth_difference));
                }else{
                    this->_depth_difference_mask = (depth_difference > (uint16_t)(this->_min_depth_difference*1000));
                }


                // Set to zero areas close to edges
                cv::bitwise_and(this->_feats_tracking_msk_mat, this->_depth_difference_mask, this->_depth_difference_mask);

                int area_size = cv::countNonZero(this->_depth_difference_mask);

                if(area_size > this->_min_area_size_pixels)
                {
                    cv::bitwise_and(this->_feats_detection_msk_mat, this->_depth_difference_mask,this->_feats_detection_msk_mat);
                    this->_use_motion_mask = true;
                }
                else
                {
                    this->_use_motion_mask = false;
                }

                this->_previous_time_of_motion_detection = ros::Time::now();
                this->_previous_depth_mat = this->_depth_img_ptr->image;
            }
            else if(this->_use_motion_mask)
            {
                cv::bitwise_and(this->_feats_detection_msk_mat, this->_depth_difference_mask,this->_feats_detection_msk_mat);
            }
        }
    }
}

void PointFeatureTracker::_ProcessRGBImg()
{
    this->_rgb_img_ptr = this->_measurement.first;
    this->_current_bw_img_ptr = cv_bridge::cvtColor(this->_rgb_img_ptr, "mono8");
    _measurement_timestamp_ns = (double)this->_rgb_img_ptr->header.stamp.toNSec();
}

void PointFeatureTracker::_ProcessOcclusionMaskImg()
{
    if (this->_occlusion_msk_img_rcvd)
    {
        // Create a mask by thresholding the image
        if(this->_so_positive)
            _occlusion_msk_mat = this->_occlusion_msk_img_ptr->image == 0;
        else
            _occlusion_msk_mat = this->_occlusion_msk_img_ptr->image != 0;
    }
}

void PointFeatureTracker::setCameraInfoMsg(const sensor_msgs::CameraInfo* camera_info)
{
    this->_camera_info_ptr = sensor_msgs::CameraInfo::Ptr(new sensor_msgs::CameraInfo(*camera_info));

    if(this->_predicted_feats_msk_mat.cols != this->_camera_info_ptr->width || this->_predicted_feats_msk_mat.rows != this->_camera_info_ptr->height)
    {
        ROS_ERROR_STREAM_NAMED("PointFeatureTracker.setCameraInfoMsg", "Resizing predicted features mask!");
        this->_predicted_feats_msk_mat = cv::Mat::zeros(this->_camera_info_ptr->height, this->_camera_info_ptr->width, CV_8U);
    }

    this->_image_plane_proj_mat_eigen(0, 0) = this->_camera_info_ptr->P[0]; // For carmine + 32;
    this->_image_plane_proj_mat_eigen(0, 1) = this->_camera_info_ptr->P[1];
    this->_image_plane_proj_mat_eigen(0, 2) = this->_camera_info_ptr->P[2];
    this->_image_plane_proj_mat_eigen(0, 3) = this->_camera_info_ptr->P[3];
    this->_image_plane_proj_mat_eigen(1, 0) = this->_camera_info_ptr->P[4];
    this->_image_plane_proj_mat_eigen(1, 1) = this->_camera_info_ptr->P[5]; // For carmine + 32;
    this->_image_plane_proj_mat_eigen(1, 2) = this->_camera_info_ptr->P[6];
    this->_image_plane_proj_mat_eigen(1, 3) = this->_camera_info_ptr->P[7];
    this->_image_plane_proj_mat_eigen(2, 0) = this->_camera_info_ptr->P[8];
    this->_image_plane_proj_mat_eigen(2, 1) = this->_camera_info_ptr->P[9];
    this->_image_plane_proj_mat_eigen(2, 2) = this->_camera_info_ptr->P[10];
    this->_image_plane_proj_mat_eigen(2, 3) = this->_camera_info_ptr->P[11];
    this->_camera_info_msg_rcvd = true;
}

ft_state_t PointFeatureTracker::getState() const
{
    return this->_state;
}

cv_bridge::CvImagePtr PointFeatureTracker::getTrackedFeaturesImg()
{
    if(this->_publishing_tracked_feats_img && this->_rgb_img_ptr)
    {
        this->_rgb_img_ptr->image.copyTo(this->_tracked_features_img->image);
        this->_tracked_features_img->encoding = this->_rgb_img_ptr->encoding;
        this->_tracked_features_img->header.stamp = ros::Time::now();

        for (size_t tracked_feats_idx = 0; tracked_feats_idx < this->_corrected_measurement.size(); tracked_feats_idx++)
        {
            cv::Point p = cvPoint(cvRound(this->_corrected_measurement[tracked_feats_idx].x),
                                  cvRound(this->_corrected_measurement[tracked_feats_idx].y));

            // If the feature has been correctly tracked we paint a yellow cross
            if (this->_feat_status_v[tracked_feats_idx])
            {
                //cv::circle(this->_cv_ptr_rgb->image, p,3, CV_RGB(255, 255, 255), -1, 8, 0);
                cv::line(this->_tracked_features_img->image, cv::Point(p.x, p.y + 3),
                         cv::Point(p.x, p.y - 3), CV_RGB(0, 255, 255), 1, 8);
                cv::line(this->_tracked_features_img->image, cv::Point(p.x + 3, p.y),
                         cv::Point(p.x - 3, p.y), CV_RGB(0, 255, 255), 1, 8);
            }
            // If the feature has been lost, we paint a red circle
            else
            {
                cv::circle(this->_tracked_features_img->image, p, 5, CV_RGB(0, 0, 255),-1, 8, 0);
            }
        }
    }
    return this->_tracked_features_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getRGBImg()
{
    return this->_rgb_img_ptr;
}

cv_bridge::CvImagePtr PointFeatureTracker::getDepthImg()
{
    return this->_depth_img_ptr;
}

cv_bridge::CvImagePtr PointFeatureTracker::getTrackedFeaturesWithPredictionMaskImg()
{
    if(this->_publishing_tracked_feats_with_pred_msk_img)
    {
        this->_tracked_features_with_pred_msk_img->image.setTo(0);
        this->_tracked_features_with_pred_msk_img->header.stamp = ros::Time::now();
        this->_tracked_features_with_pred_msk_img->encoding = this->_tracked_features_img ->encoding;
        this->_tracked_features_img->image.copyTo(this->_tracked_features_with_pred_msk_img->image, this->_predicted_feats_msk_mat);
    }
    return this->_tracked_features_with_pred_msk_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getDepthEdgesImg()
{
    this->_depth_edges_img->header.stamp = ros::Time::now();
    this->_depth_edges_img->encoding = "mono8";
    this->_depth_edges_img->image = this->_canny_edges_mat;
    return  this->_depth_edges_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getPredictingMaskImg()
{
    this->_predicting_mask_img->header.stamp = ros::Time::now();
    this->_predicting_mask_img->encoding = "mono8";
    this->_predicting_mask_img->image = this->_predicted_feats_msk_mat;
    return  this->_predicting_mask_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getTrackingMaskImg()
{
    this->_tracking_mask_img->header.stamp = ros::Time::now();
    this->_tracking_mask_img->encoding = "mono8";
    this->_tracking_mask_img->image = this->_feats_tracking_msk_mat;
    return  this->_tracking_mask_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getDetectingMaskImg()
{
    this->_detecting_mask_img->header.stamp = ros::Time::now();
    this->_detecting_mask_img->encoding = "mono8";
    this->_detecting_mask_img->image = this->_feats_detection_msk_mat;
    return  this->_detecting_mask_img;
}

cv_bridge::CvImagePtr PointFeatureTracker::getPredictedAndLastFeaturesImg()
{
    if(this->_publishing_predicted_and_past_feats_img && this->_rgb_img_ptr)
    {
        this->_rgb_img_ptr->image.copyTo(this->_predicted_and_previous_features_img->image);
        this->_predicted_and_previous_features_img->encoding = this->_rgb_img_ptr->encoding;
        this->_predicted_and_previous_features_img->header.stamp = ros::Time::now();

        //
        for (size_t tracked_feats_idx = 0;tracked_feats_idx < this->_predicted_measurements[0].size(); tracked_feats_idx++)
        {
            cv::Point previous = cvPoint(cvRound(this->_predicted_measurements[0][tracked_feats_idx].x),
                    cvRound(this->_predicted_measurements[0][tracked_feats_idx].y));
            cv::Point predicted = cvPoint(cvRound(this->_predicted_measurements[1][tracked_feats_idx].x),
                    cvRound(this->_predicted_measurements[1][tracked_feats_idx].y));

            if (this->_feat_status_v[tracked_feats_idx])
            {
                //cv::circle(this->_cv_ptr_rgb->image, p,3, CV_RGB(255, 255, 255), -1, 8, 0);
                cv::line(this->_predicted_and_previous_features_img->image, cv::Point(previous.x, previous.y + 4),
                         cv::Point(previous.x, previous.y - 4), CV_RGB(0, 255, 255), 1, 8);
                cv::line(this->_predicted_and_previous_features_img->image, cv::Point(previous.x + 4, previous.y),
                         cv::Point(previous.x - 4, previous.y), CV_RGB(0, 255, 255), 1, 8);

                // The next two lines draw a blue cross at the predicted feature location
                cv::line(this->_predicted_and_previous_features_img->image,cv::Point(predicted.x, predicted.y + 4),
                         cv::Point(predicted.x, predicted.y - 4),CV_RGB(255, 0, 0), 1, 8);
                cv::line(this->_predicted_and_previous_features_img->image,cv::Point(predicted.x + 4, predicted.y),
                         cv::Point(predicted.x - 4, predicted.y),CV_RGB(255, 0, 0), 1, 8);
            }
            else
            {
                cv::circle(this->_predicted_and_previous_features_img->image, previous, 5, CV_RGB(0, 0, 255),
                           -1, 8, 0);
            }
        }
    }
    return this->_predicted_and_previous_features_img;
}

void PointFeatureTracker::setSelfOcclusionPositive(bool so_positive)
{
    this->_so_positive = so_positive;
}

void PointFeatureTracker::_ReadParameters()
{
    this->_node_handle.getParam(this->_ft_ns + std::string("/max_distance"), this->_max_allowed_depth);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_distance"), this->_min_allowed_depth);
    this->_node_handle.getParam(this->_ft_ns + std::string("/number_features"), this->_number_of_features);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_number_features"), this->_min_number_of_features);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_feat_quality"), this->_min_feat_quality);
    this->_node_handle.getParam(this->_ft_ns + std::string("/max_interframe_jump"), this->_max_interframe_jump);
    this->_node_handle.getParam(this->_ft_ns + std::string("/max_distance"), this->_max_allowed_depth);

    this->_node_handle.getParam(this->_ft_ns + std::string("/pub_tracked_feats_with_pred_mask_img"), this->_publishing_tracked_feats_with_pred_msk_img);
    this->_node_handle.getParam(this->_ft_ns + std::string("/pub_tracked_feats_img"), this->_publishing_tracked_feats_img);
    this->_node_handle.getParam(this->_ft_ns + std::string("/pub_predicted_and_past_feats_img"), this->_publishing_predicted_and_past_feats_img);

    this->_node_handle.getParam(std::string("/omip/sensor_fps"), this->_sensor_fps);
    this->_node_handle.getParam(std::string("/omip/processing_factor"), this->_processing_factor);
    this->_loop_period_ns = 1e9/(this->_sensor_fps/(double)this->_processing_factor);

    this->_node_handle.getParam(this->_ft_ns + std::string("/erosion_size_detect"), this->_erosion_size_detecting);
    this->_node_handle.getParam(this->_ft_ns + std::string("/erosion_size_track"), this->_erosion_size_tracking);

    this->_node_handle.getParam(this->_ft_ns + std::string("/attention_to_motion"),this->_attention_to_motion);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_time_to_detect_motion"),this->_min_time_to_detect_motion);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_depth_difference"),this->_min_depth_difference);
    this->_node_handle.getParam(this->_ft_ns + std::string("/min_area_size_pixels"),this->_min_area_size_pixels);

    ROS_INFO_STREAM_NAMED(
                "PointFeatureTracker::ReadParameters",
                "PointFeatureTracker Parameters: " << std::endl <<
                "\tMaximum distance: " << this->_max_allowed_depth << std::endl <<
                "\tMinimum distance: " << this->_min_allowed_depth << std::endl <<
                "\tMaximum allowed 3D motion of features between consecutive frames: " << this->_max_interframe_jump << std::endl <<
                "\tNumber of tracked features: " << this->_number_of_features << std::endl <<
                "\tMinimum number of tracked features: " << this->_min_number_of_features << std::endl <<
                "\tMinimum feature quality: " << this->_min_feat_quality << std::endl <<
                "\tMax framerate (factor): " << this->_sensor_fps << "(" << this->_processing_factor << ")" << std::endl <<
                "\tSize of the erosion for detecting: " << this->_erosion_size_detecting << std::endl <<
                "\tSize of the erosion for tracking: " << this->_erosion_size_tracking << std::endl <<
                "\tFocussing attention into depth-changing areas: " << this->_attention_to_motion << std::endl <<
                "\tMinimum time between frames to detect depth-changing areas: " << this->_min_time_to_detect_motion << std::endl <<
                "\tMinimum depth change to consider a pixel as changing: " << this->_min_depth_difference << std::endl <<
                "\tMinimum area size to use the attention mechanism: " << this->_min_area_size_pixels << std::endl <<
                "\tPublishing image of the tracked feats: " << this->_publishing_tracked_feats_img << std::endl <<
                "\tPublishing image of the tracked feats with prediction mask: " << this->_publishing_tracked_feats_with_pred_msk_img << std::endl <<
                "\tPublishing image of the predicted and the past features: " << this->_publishing_predicted_and_past_feats_img);
}

void PointFeatureTracker::_InitializeVariables()
{
    this->_filter_name = "FTFilter";
    this->_camera_info_ptr = sensor_msgs::CameraInfo::Ptr(new sensor_msgs::CameraInfo());
    this->_image_plane_proj_mat_eigen = Eigen::Matrix<double, 3, 4>::Zero();
    this->_erosion_element_detecting_mat = cv::getStructuringElement(cv::MORPH_RECT,
                                                                     cv::Size(2 * this->_erosion_size_detecting + 1,
                                                                              2 * this->_erosion_size_detecting + 1),
                                                                     cv::Point(this->_erosion_size_detecting, this->_erosion_size_detecting));

    this->_erosion_element_tracking_mat = cv::getStructuringElement(cv::MORPH_RECT,
                                                                    cv::Size(2 * this->_erosion_size_tracking + 1,
                                                                             2 * this->_erosion_size_tracking + 1),
                                                                    cv::Point(this->_erosion_size_tracking, this->_erosion_size_tracking));

    this->_canny_low_threshold = 20;
    this->_canny_kernel_size = 3;
    this->_canny_ratio = 3;

    // Initialize all vectors
    this->_corrected_measurement.resize(this->_number_of_features,cv::Point2f(-1., -1.));
    this->_corrected_belief_state.resize(this->_number_of_features,cv::Point3d(-1., -1., -1.));
    this->_previous_belief_state.resize(this->_number_of_features,cv::Point3d(-1., -1., -1.));
    this->_current_feat_ids_v.resize(this->_number_of_features, 0);
    this->_feat_status_v.resize(this->_number_of_features, false);
    this->_previous_feat_ids_v = this->_current_feat_ids_v;

    // Initialize pcl point clouds
    FeaturePCLwc init_feature = FeaturePCLwc();
    init_feature.x = -1.f;
    init_feature.y = -1.f;
    init_feature.z = -1.f;
    init_feature.label = 0;
    this->_state = ft_state_t(new  FeatureCloudPCLwc());
    this->_state->points.resize(this->_number_of_features, init_feature);

    // The predicted states are always 2
    // The first element of the predicted states is obtained from the internal state model that predicts that the locations of the
    // features in the next step will be the locations of the features in the last step
    this->_predicted_states.resize(2);
    this->_predicted_states[0] = this->_state;
    // The second element of the predicted states comes from the higher level (RBT)

    this->_predicted_measurements.resize(2);

    this->_tracked_features_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_tracked_features_with_pred_msk_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_depth_edges_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_predicting_mask_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_tracking_mask_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_detecting_mask_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    this->_predicted_and_previous_features_img = cv_bridge::CvImagePtr(new cv_bridge::CvImage());

    this->_previous_time_of_motion_detection = ros::Time(0.0);
}

void PointFeatureTracker::_EstimateDetectionMask()
{
    for (int n = 0; n < this->_corrected_measurement.size(); n++)
    {
        if (this->_feat_status_v[n])
        {
            cv::circle(this->_feats_detection_msk_mat, this->_corrected_measurement[n], 2*_erosion_size_detecting, CV_RGB(0, 0, 0), -1, 8, 0);
        }
    }
}

void PointFeatureTracker::_DetectNewFeatures()
{
    int detected_good_features = 0;
    double min_distance = 30;
    double min_feat_quality = _min_feat_quality;
    int num_new_features_to_detect = std::count(this->_feat_status_v.begin(), this->_feat_status_v.end(), false);

    // Repeat the detection until the desired minimum number of features is detected (maximum 5 times)
    int repetition_counter = 0;
    do
    {
        _min_feat_quality = min_feat_quality;
        this->_EstimateDetectionMask();

        //If we try to detect very few features (1 or 2) the probability that we cannot retrieve their 3D location
        //is high, and then even if we reduce the neccessary quality level, we do not get valid features
        num_new_features_to_detect = std::max(2*num_new_features_to_detect, 40);
        std::vector<cv::Point2f> new_features;

        // DETECTION /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        cv::goodFeaturesToTrack(this->_current_bw_img_ptr->image, new_features, num_new_features_to_detect, min_feat_quality, min_distance, this->_feats_detection_msk_mat);

        std::vector<bool> new_feats_status;
        new_feats_status.resize(num_new_features_to_detect, true);
        std::vector<cv::Point3d> new_features_3d;
        new_features_3d.resize(num_new_features_to_detect, cv::Point3d(-1., -1., -1.));

        this->_Retrieve3DCoords(new_features, new_feats_status, new_features_3d);

        for (int old_feats_idx = 0; old_feats_idx < this->_number_of_features; old_feats_idx++)
        {
            if (!this->_feat_status_v[old_feats_idx])
            {
                for (size_t new_feats_idx = 0; new_feats_idx < new_features.size();new_feats_idx++)
                {
                    if (new_feats_status[new_feats_idx])
                    {
                        this->_current_feat_ids_v[old_feats_idx] = this->_feature_id_counter++;
                        this->_corrected_measurement[old_feats_idx] = new_features[new_feats_idx];
                        this->_corrected_belief_state[old_feats_idx] = new_features_3d[new_feats_idx];
                        this->_feat_status_v[old_feats_idx] = true;
                        new_feats_status[new_feats_idx] = false;
                        detected_good_features++;
                        break;
                    }
                }
            }
        }
        min_feat_quality /= 2.0;
        min_feat_quality = std::max(min_feat_quality, 0.0001);
        num_new_features_to_detect = std::count(this->_feat_status_v.begin(), this->_feat_status_v.end(), false);
        repetition_counter++;
    }
    while (num_new_features_to_detect > (this->_number_of_features - this->_min_number_of_features) && repetition_counter < 5);
}

void PointFeatureTracker::_TrackFeatures()
{
    std::vector<std::vector<uchar> > feat_tracked;
    feat_tracked.resize(2);

    _feat_quality.clear();
    _feat_quality.resize(2);

    // Track using the previous locations
    cv::calcOpticalFlowPyrLK(
                this->_previous_bw_img_ptr->image,
                this->_current_bw_img_ptr->image,
                this->_previous_measurement,
                this->_predicted_measurements[0],
            feat_tracked[0],
            _feat_quality[0],
            cv::Size(7, 7),
            1,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS,
            _min_feat_quality/2.0);

    int num_lost_features_tracking = 0;

    // If we had a recent prediction about the next feature locations
    bool received_useful_prediction = false;
    std::vector<bool> using_predicted_location;
    using_predicted_location.resize(this->_number_of_features, false);
    double time_since_last_prediction = this->_measurement_timestamp_ns - _predicted_state_rcvd_time_ns;
    if(true && time_since_last_prediction < this->_loop_period_ns/2.0)
    {
        received_useful_prediction = true;
        // Track using the predictions
        cv::calcOpticalFlowPyrLK(
                    this->_previous_bw_img_ptr->image,
                    this->_current_bw_img_ptr->image,
                    this->_previous_measurement,
                    this->_predicted_measurements[1],
                feat_tracked[1],
                _feat_quality[1],
                cv::Size(7, 7),
                1,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                 30,
                                 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW + cv::OPTFLOW_LK_GET_MIN_EIGENVALS,
                _min_feat_quality/2.0);

        // Select the tracked location with lower error
        int using_pred = 0;
        int using_prev = 0;
        for (size_t feat_idx = 0; feat_idx < this->_number_of_features; feat_idx++)
        {
            // Both trackers found the point -> Select the one with lower error
            if (feat_tracked[0][feat_idx] && feat_tracked[1][feat_idx])
            {
                this->_corrected_measurement[feat_idx] =
                        (_feat_quality[1][feat_idx] <= _feat_quality[0][feat_idx] ? this->_predicted_measurements[1][feat_idx] : this->_predicted_measurements[0][feat_idx]);

                // Count the tracked point used
                _feat_quality[1][feat_idx] <= _feat_quality[0][feat_idx] ? using_pred++ : using_prev++;
            }
            // Only the tracker with predictions could track it
            else if (feat_tracked[1][feat_idx])
            {
                this->_corrected_measurement[feat_idx] = this->_predicted_measurements[1][feat_idx];
                using_predicted_location[feat_idx] = true;
            }
            // Only the tracker with previous could track it
            else if (feat_tracked[0][feat_idx])
            {
                this->_corrected_measurement[feat_idx] = this->_predicted_measurements[0][feat_idx];
            }
            // Both lost the feature
            else
            {
                this->_current_feat_ids_v[feat_idx] = 0;
                this->_feat_status_v[feat_idx] = false;
                num_lost_features_tracking++;
            }
        }
        ROS_INFO_STREAM_NAMED("PointFeatureTracker._TrackFeatures","Using " << using_pred << " predicted and " << using_prev << " previous Locations.");
    }else{

        ROS_ERROR_STREAM_NAMED("PointFeatureTracker.measurementCallback",
                               "The prediction from higher level can NOT be used to predict next measurement before correction. Delay: "
                               << (double)(time_since_last_prediction)/1e9);
        for (size_t feat_idx = 0; feat_idx < this->_number_of_features; feat_idx++)
        {
            // If the tracker with previous could track it
            if (feat_tracked[0][feat_idx])
            {
                this->_corrected_measurement[feat_idx] = this->_predicted_measurements[0][feat_idx];
            }
            // Both lost the feature
            else
            {
                this->_current_feat_ids_v[feat_idx] = 0;
                this->_feat_status_v[feat_idx] = false;
                num_lost_features_tracking++;
            }
        }

    }

    // 1) Check if the tracked features are in a tracking allowed region and if they could be tracked
    int num_lost_features_mask = 0;
    for (size_t feat_idx = 0; feat_idx < this->_number_of_features; feat_idx++)
    {
        if (this->_feat_status_v[feat_idx])
        {
            if (!this->_feats_tracking_msk_mat.at<uchar>(this->_corrected_measurement[feat_idx].y,this->_corrected_measurement[feat_idx].x))
            {
                this->_current_feat_ids_v[feat_idx] = 0;
                this->_feat_status_v[feat_idx] = false;
                num_lost_features_mask++;
            }
        }
    }

    // 2) If we are using predictions: check if the tracked features are in a predicted region
    int num_lost_features_prediction_mask = 0;
    if(received_useful_prediction)
    {
        for (size_t feat_idx = 0; feat_idx < this->_number_of_features; feat_idx++)
        {
            if (this->_feat_status_v[feat_idx]
                    && std::find(this->_received_prediction_ids.begin(), this->_received_prediction_ids.end(), this->_current_feat_ids_v[feat_idx])!= this->_received_prediction_ids.end()
                    && using_predicted_location[feat_idx])
            {
                if (!this->_predicted_feats_msk_mat.at<uchar>(this->_corrected_measurement[feat_idx].y, this->_corrected_measurement[feat_idx].x))
                {
                    this->_current_feat_ids_v[feat_idx] = 0;
                    this->_feat_status_v[feat_idx] = false;
                    num_lost_features_prediction_mask++;
                }
            }
        }
    }

    int num_lost_features_retrieve_3d = this->_Retrieve3DCoords(this->_corrected_measurement, this->_feat_status_v, this->_corrected_belief_state);

    int num_lost_features_compare_3d = this->_Compare3DCoords();

    ROS_INFO_NAMED("FeatureTracker._TrackFeatures",
                   "Lost features: %3d (%3d because of tracking,%3d because of masking,%3d because of out of prediction,%3d because of not having 3D data,%3d because of jumping in 3D)",
                   num_lost_features_tracking + num_lost_features_mask + num_lost_features_prediction_mask + num_lost_features_retrieve_3d + num_lost_features_compare_3d,
                   num_lost_features_tracking,
                   num_lost_features_mask,
                   num_lost_features_prediction_mask,
                   num_lost_features_retrieve_3d ,
                   num_lost_features_compare_3d);

    this->_DetectNewFeatures();
}

int PointFeatureTracker::_Retrieve3DCoords(std::vector<cv::Point2f>& points_in_2d, std::vector<bool>& points_status, std::vector<cv::Point3d>& points_in_3d)
{
    int features_without_3d_ctr = 0;
    double x_coord, y_coord, z_coord;
    for (size_t features_idx = 0; features_idx < points_in_2d.size();
         features_idx++)
    {
        if (points_status[features_idx])
        {

            if (this->_using_pc)
            {
                // Get the pixel index
                int idx = (cvRound(points_in_2d[features_idx].y) * this->_received_point_cloud->width + cvRound(points_in_2d[features_idx].x));

                x_coord = this->_received_point_cloud->points[idx].x;
                y_coord = this->_received_point_cloud->points[idx].y;
                z_coord = this->_received_point_cloud->points[idx].z;
            }
            else
            {
                cv::Point p = cvPoint(cvRound(points_in_2d[features_idx].x),cvRound(points_in_2d[features_idx].y));

                // get raw z value
                float z_raw_float = 0.0;
                uint16_t z_raw_uint = 0;
                if(this->_depth_img_ptr->encoding == "32FC1")
                {
                    z_raw_float = this->_depth_img_ptr->image.at<float>(p);
                }else{
                    z_raw_uint = this->_depth_img_ptr->image.at<uint16_t>(p);
                }

                if ((z_raw_float != 0 && std::isfinite(z_raw_float))
                        || ( z_raw_uint != 0 && std::isfinite(z_raw_uint)))
                {
                    /* K = Intrinsic camera matrix for the raw (distorted) images.
                        [fx  0 cx]
                    K = [ 0 fy cy]
                        [ 0  0  1]
                    */

                    if(this->_depth_img_ptr->encoding == "32FC1")
                    {
                        z_coord = (double)z_raw_float;
                    }else{
                        z_coord = (double)z_raw_uint/1000.0;
                    }
                    double umcx = points_in_2d[features_idx].x - this->_camera_info_ptr->K[2];
                    double vmcy = points_in_2d[features_idx].y - this->_camera_info_ptr->K[5];

                    // calculate x and y
                    x_coord = z_coord * umcx / (double)this->_camera_info_ptr->K[0];
                    y_coord = z_coord * vmcy / (double)this->_camera_info_ptr->K[4];
                }
                else
                {
                    z_coord = 0.0 / 0.0;
                }

            }

            // If the data from Kinect for this feature is RIGHT
            if (!isnan(x_coord) && !isnan(y_coord) && !isnan(z_coord))
            {
                points_in_3d[features_idx].x = x_coord;
                points_in_3d[features_idx].y = y_coord;
                points_in_3d[features_idx].z = z_coord;
            }
            else
            {
                points_in_3d[features_idx].x = -1.f;
                points_in_3d[features_idx].y = -1.f;
                points_in_3d[features_idx].z = -1.f;
                points_status[features_idx] = false;
                features_without_3d_ctr++;
            }
        }
    }
    return features_without_3d_ctr;
}

int PointFeatureTracker::_Compare3DCoords()
{
    int jumping_features_ctr = 0;
    double interframe_distance = 0.f;
    double current_x_coord = 0.f, current_y_coord = 0.f, current_z_coord = 0.f;
    double previous_x_coord = 0.f, previous_y_coord = 0.f, previous_z_coord = 0.f;
    for (int features_idx = 0; features_idx < this->_number_of_features; features_idx++)
    {
        if (this->_previous_feat_ids_v[features_idx] == this->_current_feat_ids_v[features_idx] && this->_feat_status_v[features_idx])
        {
            current_x_coord = this->_corrected_belief_state[features_idx].x;
            current_y_coord = this->_corrected_belief_state[features_idx].y;
            current_z_coord = this->_corrected_belief_state[features_idx].z;

            previous_x_coord = this->_previous_belief_state[features_idx].x;
            previous_y_coord = this->_previous_belief_state[features_idx].y;
            previous_z_coord = this->_previous_belief_state[features_idx].z;

            //std::cout << "Current: " << current_x_coord << " " << current_y_coord << " " << current_z_coord << std::endl;
            //std::cout << "Previous: "<< previous_x_coord << " " << previous_y_coord << " " << previous_z_coord << std::endl;

            interframe_distance = sqrt( pow(current_x_coord - previous_x_coord, 2) + pow(current_y_coord - previous_y_coord, 2)
                                        + pow(current_z_coord - previous_z_coord, 2));

            if (interframe_distance >= this->_max_interframe_jump)
            {
                this->_corrected_belief_state[features_idx].x = -1.f;
                this->_corrected_belief_state[features_idx].y = -1.f;
                this->_corrected_belief_state[features_idx].z = -1.f;
                this->_current_feat_ids_v[features_idx] = 0;
                this->_feat_status_v[features_idx] = false;
                jumping_features_ctr++;
            }
        }
    }
    return jumping_features_ctr;
}
