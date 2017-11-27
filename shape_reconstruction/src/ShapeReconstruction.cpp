#include "shape_reconstruction/ShapeReconstruction.h"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter_indices.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/package.h>

#include <cmath>

#include <ros/console.h>

#include "shape_reconstruction/RangeImagePlanar.hpp"

#include "shape_reconstruction/Passthrough.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include <boost/lexical_cast.hpp>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types_conversion.h>

#include <pcl/registration/icp.h>

#include <pcl/common/centroid.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>

#include <math.h>

using namespace omip;

ShapeReconstruction::ShapeReconstruction()
    :
      _accumulate_change_candidates(true),
      _min_depth_change(0.08),
      _remove_inconsistent_points(true),
      _t(0),
      _extend_to_neighbor_sv(false),
      _acc_candidates_of_current(IMG_HEIGHT, IMG_WIDTH, CV_8UC1),
      _similarity_in_normal(0),
      _similarity_in_h(0)
{
    // Create auxiliary depth map image
    this->_acc_candidates_of_current.setTo(0);

    // Create range image planar object
    this->_rip_temp = pcl::RangeImagePlanar::Ptr(new pcl::RangeImagePlanar());

    // Create indices
    this->_moving_pts_of_current_idx_depth = pcl::PointIndices::Ptr(new pcl::PointIndices ());
    this->_moving_pts_of_previous_idx_depth = pcl::PointIndices::Ptr(new pcl::PointIndices ());
    this->_out_of_view_pts_of_current_idx= pcl::PointIndices::Ptr(new pcl::PointIndices ());
    this->_occluded_pts_of_current_idx= pcl::PointIndices::Ptr(new pcl::PointIndices ());

    this->_moving_pts_of_current_idx_color = pcl::PointIndices::Ptr(new pcl::PointIndices ());
    this->_moving_pts_of_previous_idx_color = pcl::PointIndices::Ptr(new pcl::PointIndices ());

    this->_knn = pcl::search::Search<omip::SRPoint>::Ptr(new pcl::search::KdTree<omip::SRPoint>);

    this->_rb_shape  =  omip::SRPointCloud::Ptr(new omip::SRPointCloud);
    this->_rb_shape->height=1;

    this->_supporting_features = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>());

    this->_candidates = SRPointCloud::Ptr(new SRPointCloud);
    this->_candidates->height=1;

    this->_candidates_in_current = SRPointCloud::Ptr(new SRPointCloud);

    this->_rb_segment.reset(new SRPointCloud);

    /// Mesh generation and storage as STL file
    ///
    /// For RB shape
    this->_rb_polygon_data_ptr = vtkSmartPointer<vtkPolyData>::New ();
    this->_rb_normal_estimator_ptr.reset(new pcl::NormalEstimationOMP<omip::SRPoint, pcl::Normal>());
    this->_rb_normal_estimator_ptr->setKSearch (50);
    this->_rb_estimated_normals_pc_ptr.reset(new pcl::PointCloud<pcl::Normal>());
    this->_rb_tree_for_normal_estimation_ptr.reset(new pcl::search::KdTree<omip::SRPoint>());
    this->_rb_position_color_and_normals_pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    this->_rb_tree_for_triangulation_ptr.reset(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    this->_rb_greedy_projection_triangulator_ptr.reset(new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>());
    // Set typical values for the parameters
    this->_rb_greedy_projection_triangulator_ptr->setMu (2.5);
    this->_rb_greedy_projection_triangulator_ptr->setMaximumNearestNeighbors (100);
    this->_rb_greedy_projection_triangulator_ptr->setMaximumSurfaceAngle(M_PI); // 180 degrees
    this->_rb_greedy_projection_triangulator_ptr->setMinimumAngle(0.0); // 5 degrees
    this->_rb_greedy_projection_triangulator_ptr->setMaximumAngle(M_PI); // 180 degrees
    this->_rb_greedy_projection_triangulator_ptr->setNormalConsistency(true);
    this->_rb_greedy_projection_triangulator_ptr->setConsistentVertexOrdering(true);
    // Set the maximum distance between connected points (maximum edge length)
    this->_rb_greedy_projection_triangulator_ptr->setSearchRadius (0.1);
    this->_rb_polygon_writer_ptr = vtkSmartPointer<vtkSTLWriter>::New ();
    this->_rb_polygon_writer_ptr->SetFileTypeToBinary();
    this->_rb_triangulated_mesh_ptr.reset(new pcl::PolygonMesh());
}

void ShapeReconstruction::setInitialFullRGBDPCAndRBT(const SRPointCloud::Ptr &initial_pc_msg,
                                                     const geometry_msgs::TwistWithCovariance &rb_transformation_initial)
{
    // Copy the initial point cloud
    pcl::copyPointCloud(*initial_pc_msg, *this->_initial_ffs._pc);

    // Get time stamp of the initial point cloud
    this->_initial_ffs._time.fromNSec(initial_pc_msg->header.stamp);

    // Clean the point cloud of nans
    pcl::removeNaNFromPointCloud<SRPoint>(*this->_initial_ffs._pc,*this->_initial_ffs._pc_without_nans, this->_initial_ffs._not_nan_indices);

    // Get the RGB image from the point cloud
    pcl::toROSMsg (*initial_pc_msg, this->_initial_ffs._rgb); //convert the cloud

    // Get the initial transformation
    this->_initial_ffs._transformation = geometry_msgs::TwistWithCovariance(rb_transformation_initial);

    // Get the depth map from the point cloud
    OrganizedPC2DepthMap(this->_initial_ffs._pc, this->_initial_ffs._dm->image);

    if(this->_depth_filling)
    {
        fillNaNsCBF(this->_initial_ffs._rgb, this->_initial_ffs._dm->image, this->_initial_ffs._dm_filled, this->_initial_ffs._time);
    }
}

void ShapeReconstruction::initialize()
{
    std::stringstream pub_topic_name_ext_d_and_c;
    pub_topic_name_ext_d_and_c << "/shape_recons/shape_rb" << this->_rb_id;
    this->_rb_shape_pub = this->_node_handle.advertise<sensor_msgs::PointCloud2>(pub_topic_name_ext_d_and_c.str(), 1);

    std::stringstream pub_segment_topic_name;
    pub_segment_topic_name << "/shape_recons/segment_rb" << this->_rb_id;
    this->_rb_segment_pub = this->_node_handle.advertise<sensor_msgs::PointCloud2>(pub_segment_topic_name.str(), 1);

    std::stringstream pub_occlusion_topic_name;
    pub_occlusion_topic_name << "/shape_recons/pc_occlusions_rb" << this->_rb_id;
    this->_occlusions_pub = this->_node_handle.advertise<sensor_msgs::PointCloud2>(pub_occlusion_topic_name.str(), 1);

    std::stringstream pub_occlusion_t_topic_name;
    pub_occlusion_t_topic_name << "/shape_recons/pc_occlusions_transf_rb" << this->_rb_id;
    this->_occlusions_transformed_pub = this->_node_handle.advertise<sensor_msgs::PointCloud2>(pub_occlusion_t_topic_name.str(), 1);

    this->_approximate_voxel_grid_filter.setLeafSize(this->_leaf_size,
                                                     this->_leaf_size,
                                                     this->_leaf_size);

    // This filter should delete single super voxels. If the properties of the super voxels
    // change, or the leaf size of the voxel grid filter change, the radius and/or the min neighbors of
    // the outlier removal should change
    // set radius for neighbor search
    this->_radius_outlier_removal.setRadiusSearch (this->_ror_radius_search);
    // set threshold for minimum required neighbors neighbors
    this->_radius_outlier_removal.setMinNeighborsInRadius (this->_ror_min_neighbors);

    if(this->_record_videos)
    {
        std::string videos_path = ros::package::getPath("shape_reconstruction") + std::string("/videos/");

        // create folder if does not exist
        boost::filesystem::path videos_folder(videos_path);
        if (!boost::filesystem::exists(videos_folder)) {
            if (!boost::filesystem::create_directories(videos_folder)) {
                ROS_ERROR_NAMED("ShapeReconstruction.ShapeReconstruction", "Output directory for video bag does not exist and cannot be created!");
                return;
            }
        }

        this->_videos.open(videos_path + std::string("videos_rb") + boost::lexical_cast<std::string>(this->_rb_id)+std::string(".bag"),rosbag::bagmode::Write);

        sensor_msgs::ImagePtr initial_dm_msg;
        DepthImage2CvImage(this->_initial_ffs._dm->image, initial_dm_msg);
        this->_videos.write("original_dm", this->_initial_ffs._time, initial_dm_msg);

        if(this->_depth_filling)
        {
            sensor_msgs::ImagePtr initial_dm_filled_msg;
            DepthImage2CvImage(this->_initial_ffs._dm_filled, initial_dm_filled_msg);
            this->_videos.write("filled_dm", this->_initial_ffs._time, initial_dm_filled_msg);
        }
    }
}

ShapeReconstruction::ShapeReconstruction(const ShapeReconstruction &sr)
{
    ROS_ERROR_STREAM_NAMED("ShapeReconstruction.ShapeReconstruction", "Do not use this copy constructor! It is not complete!");

    this->_rb_id = sr._rb_id;

    this->_to_initial = sr._to_initial;
    this->_depth_filling = sr._depth_filling;

    this->_initial_ffs = sr._initial_ffs.clone();
    this->_previous_ffs = sr._previous_ffs.clone();
    this->_current_ffs = sr._current_ffs.clone();

    this->_rip_temp = pcl::RangeImagePlanar::Ptr(new pcl::RangeImagePlanar(*sr._rip_temp));

    this->_moving_pts_of_current_idx_depth = pcl::PointIndices::Ptr(new pcl::PointIndices(*sr._moving_pts_of_current_idx_depth));
    this->_moving_pts_of_previous_idx_depth = pcl::PointIndices::Ptr(new pcl::PointIndices(*sr._moving_pts_of_previous_idx_depth));
    this->_out_of_view_pts_of_current_idx = pcl::PointIndices::Ptr(new pcl::PointIndices(*sr._out_of_view_pts_of_current_idx));
    this->_occluded_pts_of_current_idx = pcl::PointIndices::Ptr(new pcl::PointIndices(*sr._occluded_pts_of_current_idx));

}

ShapeReconstruction::~ShapeReconstruction()
{
    if(this->_record_videos)
    {
        _videos.close();
    }
}

void ShapeReconstruction::setCameraInfo(const sensor_msgs::CameraInfo& camera_info)
{
    this->_ci = sensor_msgs::CameraInfo(camera_info);
}

void ShapeReconstruction::setFullRGBDPCandRBT(const SRPointCloud::Ptr &pc_msg,
                                              const geometry_msgs::TwistWithCovariance &rb_transformation)
{
    ros::Time t1 = ros::Time::now();

    _previous_k_indices.clear();
    _current_k_indices.clear();
    // t=0 means first point cloud AFTER initial received
    ROS_WARN_STREAM_NAMED("ShapeReconstruction.setFullRGBDPCandRBT", "[RB" << this->_rb_id << "]: t = " << _t);


    if(this->_current_ffs._pc->points.size() > 0 )
    {
        this->_previous_ffs = this->_current_ffs;
    }else{
        this->_previous_ffs = this->_initial_ffs;
    }
    this->_current_ffs.reset();

    ros::Time t2 = ros::Time::now();

    pcl::copyPointCloud(*pc_msg, *this->_current_ffs._pc);

    ros::Time t3 = ros::Time::now();

    // Clean the point cloud of nans
    this->_current_ffs._not_nan_indices.clear();
    pcl::removeNaNFromPointCloud<SRPoint>(*this->_current_ffs._pc,*this->_current_ffs._pc_without_nans, this->_current_ffs._not_nan_indices);

    ros::Time t4 = ros::Time::now();

    ROS_WARN_STREAM_NAMED("ShapeReconstruction.setFullRGBDPCandRBT", "[RB" << this->_rb_id << "]: " << "Point cloud time stamp: "
                          <<this->_current_ffs._pc->header.stamp <<  ", not nan points "
                          << this->_current_ffs._not_nan_indices.size() << "/" << this->_current_ffs._pc_without_nans->size());


    this->_current_ffs._time = pcl_conversions::fromPCL(pc_msg->header.stamp);
    pcl::toROSMsg (*pc_msg, this->_current_ffs._rgb); //convert the cloud
    this->_current_ffs._transformation = geometry_msgs::TwistWithCovariance(rb_transformation);

    ros::Time t5 = ros::Time::now();

    // Create a depth map from the current organized depth map
    OrganizedPC2DepthMap(this->_current_ffs._pc, this->_current_ffs._dm->image);

    ros::Time t6 = ros::Time::now();

    // -----------------------------------------------------------
    // main algorithm

    if(this->_depth_filling)
    {
        fillNaNsCBF(this->_current_ffs._rgb, this->_current_ffs._dm->image, this->_current_ffs._dm_filled, this->_current_ffs._time);
        this->_current_ffs._dm->image.copyTo(this->_current_ffs._dm_filled, this->_current_ffs._dm->image == this->_current_ffs._dm->image);
    }

    ros::Time t7 = ros::Time::now();

    if(this->_record_videos)
    {
        sensor_msgs::ImagePtr current_dm_msg;
        DepthImage2CvImage(this->_current_ffs._dm->image, current_dm_msg);
        this->_videos.write("original_dm", this->_current_ffs._time, current_dm_msg);

        if(this->_depth_filling)
        {
            sensor_msgs::ImagePtr current_dm_filled_msg;
            DepthImage2CvImage(this->_current_ffs._dm_filled, current_dm_filled_msg);
            this->_videos.write("filled_dm", this->_current_ffs._time, current_dm_filled_msg);
        }
    }

    // Segmentation method 1: motion segmentation based on DEPTH (memoryless)
    this->_DetectImageLocationsWhereDepthChanges();
    ros::Time t8 = ros::Time::now();
    this->_TestMotionCoherencyOfPointsInImageLocationsWhereDepthChanged();
    ros::Time t9 = ros::Time::now();

    // Segmentation method 2: motion segmentation based on COLOR (memoryless)
    this->_DetectImageLocationsWhereColorChanges();
    ros::Time t10 = ros::Time::now();
    this->_TestMotionCoherencyOfPointsInImageLocationsWhereColorChanged();
    ros::Time t11 = ros::Time::now();

    // Merge the segmentation results into the model(s)
    this->_MergeValidPointsIntoModels();

    ros::Time t12 = ros::Time::now();

    this->_RemoveInconsistentPointsFromModelsAndExtendToRegions();

    ros::Time t12b = ros::Time::now();

    this->_FilterModel();

    ros::Time t13 = ros::Time::now();

    _t++;

//    std::cout << "Preprocessing: " << (t7-t1).toSec() << std::endl;
//    std::cout << "Depth-based segmentation: " << (t9-t7).toSec() << std::endl;
//    std::cout << "Color-based segmentation: " << (t11-t9).toSec() << std::endl;
//    std::cout << "Remove inconsistent points and extend to regions: " << (t12b-t12).toSec() << std::endl;
//    std::cout << "Filter model: " << (t13-t12b).toSec() << std::endl;
//    std::cout << "Total processing RGBD frame (one RB): " << (t13-t1).toSec() << std::endl;
}

void ShapeReconstruction::_DetectImageLocationsWhereDepthChanges()
{
    // Find the difference
    if(this->_depth_filling)
    {
        this->_difference_in_depth = this->_current_ffs._dm_filled - this->_previous_ffs._dm_filled;
    }else{
        this->_difference_in_depth = this->_current_ffs._dm->image - this->_previous_ffs._dm->image;
    }

    if(this->_record_videos)
    {
        sensor_msgs::ImagePtr dm_msg;
        DepthImage2CvImage(_difference_in_depth, dm_msg);
        this->_videos.write("changes_in_depth", this->_current_ffs._time, dm_msg);
    }


    // If the difference is negative (under some threshold) that means that the object occludes now what was behind and visible before
    // This part of the CURRENT depth map (of the CURRENT point cloud) should be added to the candidates of the RB
    cv::threshold(this->_difference_in_depth, this->_candidates_of_current, -_min_depth_change, 255, cv::THRESH_BINARY_INV);

    if(this->_record_videos)
    {
        sensor_msgs::ImagePtr dm_msg;
        DepthImage2CvImage(_candidates_of_current, dm_msg);
        this->_videos.write("candidates_of_current", this->_current_ffs._time, dm_msg);
    }

    // If the difference is positive (over some threshold) that means that the object is now not occluding what it was occluding before
    // This part of the PREVIOUS depth map (of the PREVIOUS point cloud) should be added to the candidates of the RB
    cv::threshold(this->_difference_in_depth, this->_candidates_of_previous, _min_depth_change, 255, cv::THRESH_BINARY);

    if(this->_record_videos)
    {
        sensor_msgs::ImagePtr dm_msg;
        DepthImage2CvImage(_candidates_of_previous, dm_msg);
        this->_videos.write("candidates_of_previous", this->_current_ffs._time, dm_msg);
    }

    this->_previous_depth_mask = this->_previous_ffs._dm->image > 1.5 | this->_previous_ffs._dm->image < 0.3 | this->_previous_ffs._dm->image != this->_previous_ffs._dm->image;
    this->_current_depth_mask = this->_current_ffs._dm->image > 1.5 | this->_current_ffs._dm->image < 0.3 | this->_current_ffs._dm->image != this->_current_ffs._dm->image;

    this->_candidates_of_current.convertTo(this->_candidates_of_current_8u, CV_8U);
    this->_candidates_of_previous.convertTo(this->_candidates_of_previous_8u, CV_8U);

    // accumulate candidates
    this->_acc_candidates_of_current = _acc_candidates_of_current | this->_candidates_of_current_8u;
    this->_acc_candidates_of_current.setTo(0,this->_candidates_of_previous_8u);

    if(this->_record_videos)
    {
        sensor_msgs::ImagePtr dm_msg;
        DepthImage2CvImage(_acc_candidates_of_current, dm_msg);
        this->_videos.write("acc_candidates_of_current", this->_current_ffs._time, dm_msg);
    }

    if (!this->_depth_filling)
    {
        ROS_INFO_STREAM_NAMED("ShapeReconstruction._DetectImageLocationsWhereDepthChanges",  "[RB" << this->_rb_id << "]: Removing nan points from accumulator");
        this->_acc_candidates_of_current.setTo(0, this->_current_ffs._dm->image != this->_current_ffs._dm->image);
    }

    if(this->_accumulate_change_candidates)
    {
        this->_acc_candidates_of_current.copyTo(this->_candidates_of_current_8u);
    }

    this->_candidates_of_current_8u.setTo(0, this->_current_depth_mask);
    this->_candidates_of_previous_8u.setTo(0, this->_previous_depth_mask);

    this->_moving_pts_of_current_idx_depth->indices.clear();
    this->_moving_pts_of_previous_idx_depth->indices.clear();

    Image8u2Indices(this->_candidates_of_current_8u, this->_moving_pts_of_current_idx_depth);
    Image8u2Indices(this->_candidates_of_previous_8u, this->_moving_pts_of_previous_idx_depth);
}

void ShapeReconstruction::_TestMotionCoherencyOfPointsInImageLocationsWhereDepthChanged()
{
    // Estimate the transformations from the twists
    this->_EstimateTransformations();

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._TestMotionCoherencyOfPointsInImageLocationsWhereDepthChanged",
                          "[RB" << this->_rb_id << "]: Estimating which of the points that changed DEPTH moved coherently with the RB motion.");

    // Filter the previous point cloud, transform only the filtered points and find the nearest neighbors in the current point cloud
    this->_sqrt_dist.clear();
    this->_FindCandidatesInPreviousPC(this->_previous_ffs,
                                      this->_current_ffs,
                                      this->_moving_pts_of_previous_idx_depth,
                                      this->_current_to_previous_HTransform,
                                      this->_current_k_indices,
                                      this->_sqrt_dist);

    // Filter the current point cloud, transform only the filtered points and find the nearest neighbors in the previous point cloud
    this->_sqrt_dist.clear();
    this->_FindCandidatesInCurrentPC(this->_previous_ffs,
                                     this->_current_ffs,
                                     this->_moving_pts_of_current_idx_depth,
                                     this->_previous_to_current_HTransform,
                                     this->_previous_k_indices,
                                     this->_sqrt_dist);

    std::vector<int > invalid_indices;
    // Add the valid points of the current point cloud to the model
    for (unsigned int i = 0; i < _previous_k_indices.size(); i++)
    {
        if (_previous_k_indices[i].size() == 0)
        {
            invalid_indices.push_back(_moving_pts_of_current_idx_depth->indices.at(i));
        }
    }

    cv::Mat not_coherent_points = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
    Indices2Image8u(invalid_indices, not_coherent_points);

    this->_acc_candidates_of_current.setTo(0, not_coherent_points);
}

//#define DISPLAY_IMAGES_DILWCC
void ShapeReconstruction::_DetectImageLocationsWhereColorChanges()
{
    // Create matrices
    cv::Mat current_hsv_image, current_h_channel, current_s_channel, current_v_channel;
    cv::Mat previous_hsv_image, previous_h_channel, previous_s_channel, previous_v_channel;
    cv::Mat difference_h_channel, circular_difference_h_channel, thresholded_difference_h_channel, difference_h_channel_wo_black, thresholded_difference_h_channel_wo_black;

    // Convert ROS msg to opencv
    cv_bridge::CvImagePtr previous_rgb_cv;
    previous_rgb_cv = cv_bridge::toCvCopy(this->_previous_ffs._rgb);

    cv_bridge::CvImagePtr current_rgb_cv;
    current_rgb_cv = cv_bridge::toCvCopy(this->_current_ffs._rgb);

#ifdef DISPLAY_IMAGES_DILWCC
    // Display original color images
    cv::imshow("color current", current_rgb_cv->image);
    cv::imshow("color previous", previous_rgb_cv->image);
#endif

    // Convert color to HSV images
    cv::cvtColor(previous_rgb_cv->image, previous_hsv_image, CV_BGR2HSV);
    cv::cvtColor(current_rgb_cv->image, current_hsv_image, CV_BGR2HSV);

    // Smooth images
    //cv::GaussianBlur(previous_hsv_image, previous_hsv_image,cv::Size(5,5), 0,0);
    //cv::GaussianBlur(current_hsv_image, current_hsv_image,cv::Size(5,5), 0,0);

    // Extract channels
    cv::extractChannel(current_hsv_image,current_h_channel,0);
    cv::extractChannel(current_hsv_image,current_s_channel,1);
    cv::extractChannel(current_hsv_image,current_v_channel,2);

    cv::extractChannel(previous_hsv_image,previous_h_channel,0);
    cv::extractChannel(previous_hsv_image,previous_s_channel,1);
    cv::extractChannel(previous_hsv_image,previous_v_channel,2);

    // Difference of hue -> HUE values are between 0 and 179 and represent the angle
    // of a cylindrical coordinates system. That implies that 180=0.
    cv::absdiff(current_h_channel,  previous_h_channel, difference_h_channel );
    circular_difference_h_channel = 180.0 - difference_h_channel;
    circular_difference_h_channel.copyTo(difference_h_channel, difference_h_channel >89);

#ifdef DISPLAY_IMAGES_DILWCC
    // Show difference of HUE
    cv::imshow("Circular difference of HUE", difference_h_channel);
#endif

    // We do not consider pixels that change between different black hue colors, because black identification if really bad
    difference_h_channel.copyTo(difference_h_channel_wo_black);
    //difference_h_channel_wo_black.setTo(cv::Scalar(0), current_s_channel < 90 & current_v_channel < 50 & previous_s_channel < 90 & previous_v_channel < 50);
    difference_h_channel_wo_black.setTo(cv::Scalar(0), current_s_channel < 90 | previous_s_channel < 90 );

#ifdef DISPLAY_IMAGES_DILWCC
    // Show pixels with low saturation or value
    cv::imshow("Low current_s_channel", current_s_channel < 90);
    //cv::imshow("Low current_v_channel", current_v_channel < 10);
    cv::imshow("Low previous_s_channel", previous_s_channel < 90);
    //cv::imshow("Low previous_v_channel", previous_v_channel < 10);
#endif

    // Threshold the circular hue difference to find points that change enough
    cv::threshold(difference_h_channel, thresholded_difference_h_channel, 20, 255, cv::THRESH_BINARY);

#ifdef DISPLAY_IMAGES_DILWCC
    cv::imshow("Thresholded circular difference of HUE", thresholded_difference_h_channel);
#endif

    // Threshold the circular hue difference without black pixels to find points that change enough
    cv::threshold(difference_h_channel_wo_black, thresholded_difference_h_channel_wo_black, 15, 255, cv::THRESH_BINARY);

#ifdef DISPLAY_IMAGES_DILWCC
    cv::imshow("Thresholded circular difference of HUE without black pixels", thresholded_difference_h_channel_wo_black);
#endif

    cv::Mat thresholded_difference_h_channel_wo_black_wo_noisy_pixels;
    int openning_size = 1;
    // Element: 0: Rect - 1: Cross - 2: Ellipse
    int openning_element_type = 2;
    cv::Mat openning_element = cv::getStructuringElement( openning_element_type, cv::Size( 2*openning_size + 1, 2*openning_size+1 ), cv::Point( openning_size, openning_size ) );
    cv::morphologyEx(thresholded_difference_h_channel_wo_black, thresholded_difference_h_channel_wo_black_wo_noisy_pixels, cv::MORPH_OPEN, openning_element);

#ifdef DISPLAY_IMAGES_DILWCC
    cv::imshow("Thresholded circular difference of HUE without black pixels nor NOISY pixels", thresholded_difference_h_channel_wo_black_wo_noisy_pixels);
#endif

    // We use color to detect things that move and do not change depth. Therefore, we set to zero
    // points that changed their color AND their depth
    cv::Mat changing_depth_points;
    if(this->_depth_filling)
    {
        cv::absdiff(this->_current_ffs._dm_filled, this->_previous_ffs._dm_filled, changing_depth_points);
    }else{
        cv::absdiff(this->_current_ffs._dm->image, this->_previous_ffs._dm->image, changing_depth_points);
    }

    // The mask of moving points in current frame are the points that changed color and that are valid
    cv::Mat motion_mask_current= thresholded_difference_h_channel_wo_black_wo_noisy_pixels.clone();
    motion_mask_current.setTo(cv::Scalar(0), this->_current_depth_mask);

    // The mask of moving points in previous frame are the points that changed color and that are valid
    cv::Mat motion_mask_previous = thresholded_difference_h_channel_wo_black_wo_noisy_pixels.clone();
    motion_mask_previous.setTo(cv::Scalar(0), this->_previous_depth_mask);

#ifdef DISPLAY_IMAGES_DILWCC
    cv::imshow("Current color candidates", motion_mask_current);
    cv::imshow("Previous color candidates", motion_mask_previous);
#endif

#ifdef DISPLAY_IMAGES_DILWCC
    cv::waitKey(-1);
#endif

    Image8u2Indices(motion_mask_current, this->_moving_pts_of_current_idx_color);
    Image8u2Indices(motion_mask_previous, this->_moving_pts_of_previous_idx_color);

    if(this->_record_videos)
    {
        _videos.write("original_color",ros::Time::now(), current_rgb_cv->toImageMsg());
        sensor_msgs::ImagePtr dm_msg;
        DepthImage2CvImage(thresholded_difference_h_channel, dm_msg);
        this->_videos.write("changes_in_color", this->_current_ffs._time, dm_msg);
    }
}

void ShapeReconstruction::_TestMotionCoherencyOfPointsInImageLocationsWhereColorChanged()
{
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._TestMotionCoherencyOfPointsInImageLocationsWhereColorChanged",
                          "[RB" << this->_rb_id << "]: Estimating which of the points that changed COLOR moved coherently with the RB motion.");

    // Filter the previous point cloud, transform only the filtered points and find the nearest neighbors in the current point cloud
    this->_FindCandidatesInPreviousPC(this->_previous_ffs,
                                      this->_current_ffs,
                                      this->_moving_pts_of_previous_idx_color,
                                      this->_current_to_previous_HTransform,
                                      this->_current_k_indices,
                                      this->_sqrt_dist);

    // Filter the current point cloud, transform only the filtered points and find the nearest neighbors in the previous point cloud
    this->_FindCandidatesInCurrentPC(this->_previous_ffs,
                                     this->_current_ffs,
                                     this->_moving_pts_of_current_idx_color,
                                     this->_previous_to_current_HTransform,
                                     this->_previous_k_indices,
                                     this->_sqrt_dist);
}

void ShapeReconstruction::_FindCandidatesInPreviousPC(FrameForSegmentation& _previous_ffs,
                                                      const FrameForSegmentation& _current_ffs,
                                                      pcl::PointIndices::Ptr& _moving_pts_of_previous_idx,
                                                      Eigen::Matrix4d& _current_to_previous_HTransform,
                                                      std::vector<std::vector<int > >& _previous_k_indices,
                                                      std::vector<std::vector<float > >& _previous_sqrt_dist)
{
    _previous_ffs._pc_moving_pts =  omip::SRPointCloud::Ptr(new omip::SRPointCloud);
    this->_extractor.setInputCloud(_previous_ffs._pc);
    this->_extractor.setIndices(_moving_pts_of_previous_idx);
    this->_extractor.setNegative(false);
    this->_extractor.filter(*_previous_ffs._pc_moving_pts);

    std::vector<std::vector<int > > previous_k_indices;
    if(_previous_ffs._pc_moving_pts->points.size())
    {
        pcl::transformPointCloud<SRPoint>(*_previous_ffs._pc_moving_pts, *_previous_ffs._pc_moving_pts_transf, _current_to_previous_HTransform.cast<float>());

        this->_knn->setInputCloud(_current_ffs._pc_without_nans);
        std::vector<int> empty;

        this->_knn->radiusSearch(*_previous_ffs._pc_moving_pts_transf, empty, this->_knn_min_radius, previous_k_indices, _previous_sqrt_dist);

        ROS_INFO_STREAM_NAMED("ShapeReconstruction._FindCandidatesInPreviousPC", "[RB" << this->_rb_id << "]: After radius search for finding candidates in previous PC. Queries: "
                              << _previous_ffs._pc_moving_pts_transf->points.size() );

    }

    _previous_k_indices.insert(_previous_k_indices.end(), previous_k_indices.begin(), previous_k_indices.end());
}

void ShapeReconstruction::_FindCandidatesInCurrentPC(const FrameForSegmentation& _previous_ffs,
                                                     FrameForSegmentation& _current_ffs,
                                                     pcl::PointIndices::Ptr& _moving_pts_of_current_idx,
                                                     Eigen::Matrix4d& _previous_to_current_HTransform,
                                                     std::vector<std::vector<int > >& _current_k_indices,
                                                     std::vector<std::vector<float > >& _current_sqrt_dist)
{
    _current_ffs._pc_moving_pts =  omip::SRPointCloud::Ptr(new omip::SRPointCloud);
    this->_extractor.setInputCloud(_current_ffs._pc);
    this->_extractor.setIndices(_moving_pts_of_current_idx);
    this->_extractor.setNegative(false);
    this->_extractor.filter(*_current_ffs._pc_moving_pts);

    std::vector<std::vector<int > > current_k_indices;
    if(_current_ffs._pc_moving_pts->points.size())
    {
        pcl::transformPointCloud<SRPoint>(*_current_ffs._pc_moving_pts, *_current_ffs._pc_moving_pts_transf, _previous_to_current_HTransform.cast<float>());

        this->_knn->setInputCloud(_previous_ffs._pc_without_nans);

        std::vector<int> empty;
        this->_knn->radiusSearch(*_current_ffs._pc_moving_pts_transf, empty, this->_knn_min_radius, current_k_indices, _current_sqrt_dist);

        ROS_INFO_STREAM_NAMED("ShapeReconstruction._FindCandidatesInPreviousPC", "[RB" << this->_rb_id << "]: After radius search for finding candidates in current PC. Queries: "
                              << _current_ffs._pc_moving_pts_transf->points.size() );
    }

    _current_k_indices.insert(_current_k_indices.end(), current_k_indices.begin(), current_k_indices.end());
}


void ShapeReconstruction::_MergeValidPointsIntoModels()
{
    // We create these transformed point clouds in the general function to avoid repeating it in the subfunctions

    // Transform the points of the current point cloud that belong to the model to the object frame (corresponds to the frame of the first point cloud)
    omip::SRPointCloud::Ptr points_of_current_in_origin(new omip::SRPointCloud);
    pcl::transformPointCloud<SRPoint>(*this->_current_ffs._pc_without_nans, *points_of_current_in_origin, this->_current_HTransform_inv.cast<float>());

    // Transform the points of the previous point cloud that belong to the model into the object frame (corresponds to the frame of the first point cloud)
    omip::SRPointCloud::Ptr points_of_previous_in_origin(new omip::SRPointCloud);
    pcl::transformPointCloud<SRPoint>(*this->_previous_ffs._pc_without_nans, *points_of_previous_in_origin, this->_previous_HTransform_inv.cast<float>());

    //Now we have all the indices together
    _MergeValidPointsIntoModels("DEPTHANDCOLOR",
                                    points_of_current_in_origin,
                                    points_of_previous_in_origin,
                                    this->_rb_shape,
                                    this->_current_k_indices,
                                    this->_previous_k_indices);

    this->_candidates->points.clear();
    this->_candidates->width = 0;
}

void ShapeReconstruction::_MergeValidPointsIntoModels(const std::string& logger_name,
                                                      omip::SRPointCloud::Ptr& points_of_current_in_origin,
                                                      omip::SRPointCloud::Ptr& points_of_previous_in_origin,
                                                      omip::SRPointCloud::Ptr& rb_model,
                                                      const std::vector<std::vector<int > >& current_k_indices,
                                                      const std::vector<std::vector<int > >& previous_k_indices)
{
    int model_size_before_adding = rb_model->width;

    // Add the valid points of the current point cloud to the model
    for (unsigned int i = 0; i < current_k_indices.size(); i++) {
        if (current_k_indices[i].size() > 0){
            rb_model->points.push_back(points_of_current_in_origin->points[current_k_indices[i].at(0)]);
            rb_model->width++;
        }
    }

    // Add the valid points of the previous point cloud to the model
    for (unsigned int i = 0; i < previous_k_indices.size(); i++) {
        if (previous_k_indices[i].size() > 0){
            rb_model->points.push_back(points_of_previous_in_origin->points[previous_k_indices[i].at(0)]);
            rb_model->width++;
        }
    }

    int number_of_points_added = rb_model->width - model_size_before_adding;
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._MergeValidPointsIntoModels", "[RB" << this->_rb_id << "] "
                          << ": Added " << number_of_points_added << " points to the model based on changes in " << logger_name);
}

void ShapeReconstruction::_RemoveInconsistentPointsFromModelsAndExtendToRegions()
{
    // clean up models by removing points inconsistent with the current view
    this->_RemoveInconsistentPointsFromRBModel("BeforeExtendingToRegions", this->_current_HTransform,
                                               this->_rb_shape,this->_current_ffs._dm->image,this->_current_ffs._pc,this->_rb_segment);

    if(_estimate_supervoxels)
    {
        // After cleaning up the models we extend to regions
        this->_ExtendPointsToRegions();

        this->_RemoveInconsistentPointsFromRBModel("AfterExtendingToRegions", this->_current_HTransform,
                                                   this->_rb_shape,this->_current_ffs._dm->image,this->_current_ffs._pc,this->_rb_segment);
    }
}

void ShapeReconstruction::_FilterModel()
{
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._FilterModel", "[RB" << this->_rb_id << "]: Before approximate voxel grid filter. Num of points: "
                           << this->_rb_shape->points.size() );
    this->_approximate_voxel_grid_filter.setInputCloud (this->_rb_shape);
    this->_approximate_voxel_grid_filter.filter (*this->_rb_shape);
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._FilterModel", "[RB" << this->_rb_id << "]: After approximate voxel grid filter. Num of points: "
                           << this->_rb_shape->points.size() );

    // Apply the outlier removal filter to keep the model clean of small noisy clusters
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._FilterModel", "[RB" << this->_rb_id << "] " << ": Before outlier removal. Num of points: "
                          << this->_rb_shape->points.size() );
    this->_radius_outlier_removal.setInputCloud(this->_rb_shape);
    this->_radius_outlier_removal.filter (*this->_rb_shape);
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._FilterModel", "[RB" << this->_rb_id << "] " << ": After outlier removal. Num of points: "
                          << this->_rb_shape->points.size() );

}

void ShapeReconstruction::_RemoveInconsistentPointsFromRBModel(const std::string logger_name,
                                                               const Eigen::Matrix4d& HTransform,
                                                               SRPointCloud::Ptr& rb_shape,
                                                               cv::Mat& current_dm,
                                                               SRPointCloud::Ptr current_pc,
                                                               SRPointCloud::Ptr& rb_segment)
{
    // are self-occlusions handled?
    // yes they should be accounted for implicitly because if the "front" & the "back" are in the model, only
    // the front (which should be visible in current) will be projected to rb_rip.

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsFromRBModel", "[RB" << this->_rb_id << "] " << logger_name <<
                          ": Removing inconsistent points");

    // move the rb_shape into the current frame
    omip::SRPointCloud::Ptr rb_shape_current(new omip::SRPointCloud);
    pcl::transformPointCloud<SRPoint>(*rb_shape, *rb_shape_current, HTransform.cast<float>());

    // move the rb_shape into the current frame (new variant)
    omip::SRPointCloud::Ptr rb_shape_current_new(new omip::SRPointCloud);
    *rb_shape_current_new = *rb_shape_current;

    std::stringstream rb_id_str;
    rb_id_str << _rb_id;

    pcl::PointIndicesPtr indices_to_remove(new pcl::PointIndices);

    // these are the points in the current image that are consistent
    // with the model -> the actual segment in the image
    pcl::PointIndicesPtr indices_matching_in_model(new pcl::PointIndices);
    pcl::PointIndicesPtr indices_matching_in_current(new pcl::PointIndices);

    // find points that are inconsistent between moved rb_shape
    // and current depth image
    _FindInconsistentPoints(rb_shape_current_new, current_dm, // input
                               indices_to_remove, // output
                               indices_matching_in_model, // output
                               indices_matching_in_current // output
                               );

    // remove indices
    SRPointCloud::Ptr rb_new_clean(new SRPointCloud);
    if (_remove_inconsistent_points)
    {
        // move the rb_shape into the current frame
        this->_extractor.setNegative(true);
        this->_extractor.setInputCloud(rb_shape);
        this->_extractor.setIndices(indices_to_remove);
        this->_extractor.setKeepOrganized(false);
        this->_extractor.filter(*rb_new_clean);
    } else {
        indices_to_remove->indices.clear();
    }

    // create segment (always in current frame)
    SRPointCloud::Ptr rb_segment_new(new SRPointCloud);
    this->_extractor.setNegative(false);
    this->_extractor.setInputCloud(current_pc);
    this->_extractor.setIndices(indices_matching_in_current);
    this->_extractor.setKeepOrganized(true);
    this->_extractor.filter(*rb_segment_new);
    this->_extractor.setKeepOrganized(false);

    cv::Mat rb_segment_new_dm(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
    OrganizedPC2DepthMap(rb_segment_new, rb_segment_new_dm);

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsAndComputeSegment", "[RB" << this->_rb_id << "] " << logger_name << ":" << std::endl
                          << "\tPoints of the original model: " << rb_shape->points.size() << "," << std::endl
                          << "\tPoints matching in model: " << indices_matching_in_model->indices.size() << "," << std::endl
                          << "\tPoints matching in model: " << rb_new_clean->points.size()  << "," << std::endl
                          << "\tPoints matching in current point cloud: " << indices_matching_in_current->indices.size() << std::endl
                          << "\tPoints removed: " << indices_to_remove->indices.size() << (indices_to_remove->indices.empty() ? " (NO POINTS REMOVED) " : "")
                          );

    if (_remove_inconsistent_points)
    {
        rb_shape = rb_new_clean;
    }
    rb_segment = rb_segment_new;
}

void ShapeReconstruction::_FindInconsistentPoints(const omip::SRPointCloud::Ptr& pc_source,
                                                     const cv::Mat & dm_true,
                                                     pcl::PointIndicesPtr& indices_to_remove,
                                                     pcl::PointIndicesPtr& indices_matching_in_true,
                                                     pcl::PointIndicesPtr& indices_matching_in_dm,
                                                     const double min_depth_error) {

    indices_to_remove->indices.clear();

    using ::shape_reconstruction::RangeImagePlanar;
    RangeImagePlanar::Ptr dm_source_rip(new RangeImagePlanar);

    Eigen::Affine3f sensor_pose;
    sensor_pose.matrix() = Eigen::Matrix4f::Identity(); // this->_current_HTransform_inv.cast<float>();
    pcl::RangeImagePlanar::CoordinateFrame coordinate_frame = pcl::RangeImagePlanar::CAMERA_FRAME;

    int width = dm_true.cols, height = dm_true.rows;
    dm_source_rip->matchPointCloudAndImage (
                *pc_source,
                width,
                height,
                this->_ci.P[2], //width/2 -0.5, //319.5, // 329.245223575443,
            this->_ci.P[6], //height/2 - 0.5, // 239.5, //  239.458
            this->_ci.P[0], // fx
            this->_ci.P[5], // fy
            sensor_pose,
            coordinate_frame,
            dm_true,
            min_depth_error,
            indices_matching_in_true,
            indices_matching_in_dm,
            indices_to_remove
            );

}

void ShapeReconstruction::growSVRecursively(uint32_t label_sv_seed,
                                            std::vector<uint32_t>& labels_extension,
                                            std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                                            std::vector<uint32_t>& sv_containing_supporting_features,
                                            int& distance_to_feature_sv,
                                            std::map<uint32_t, std::unordered_set<int> >& labels_of_segments_to_extend)
{
    std::pair< std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator > neighbors;

    // Get the adjacency map of the sv
    neighbors =supervoxel_adjacency.equal_range(label_sv_seed);

    // Iterate over the pairs of sv-neighbor
    for(std::multimap<uint32_t, uint32_t>::iterator neighbor_it = neighbors.first; neighbor_it!= neighbors.second; ++neighbor_it)
    {
        bool add_neighbor = false;
        // Check if the neighbor sv was already added
        if(std::find(labels_extension.begin(), labels_extension.end(), neighbor_it->second)  != labels_extension.end())
        {
            add_neighbor = false;
        }else{
            // Check if the neighbor sv should be added
            // if it contains a feature OR if (it contains enough points that changed (color OR depth) AND the distance to a feature-containing SV is small)
            if(std::find(sv_containing_supporting_features.begin(), sv_containing_supporting_features.end(), neighbor_it->second)  != sv_containing_supporting_features.end())
            {
                add_neighbor = true;
                distance_to_feature_sv = 0;
            }else{
                std::map<uint32_t, std::unordered_set<int> >::iterator set_it = labels_of_segments_to_extend.find(neighbor_it->second);
                if(set_it != labels_of_segments_to_extend.end())
                {
                    if(set_it->second.size() > this->_min_number_model_pixels_in_sv)
                    {
                        if(distance_to_feature_sv <= 2)
                        {
                            add_neighbor = true;
                            distance_to_feature_sv++;
                        }
                    }
                }

            }
        }

        if(add_neighbor)
        {
            labels_extension.push_back(neighbor_it->second);
            growSVRecursively(neighbor_it->second,
                              labels_extension,
                              supervoxel_adjacency,
                              sv_containing_supporting_features,
                              distance_to_feature_sv,
                              labels_of_segments_to_extend);
        }
    }
}

void ShapeReconstruction::_ExtendPointsToRegions()
{
    std::vector<uint32_t> labels_to_extend;

    //Retrieve the labeled point cloud. Labels == Supervoxel ids
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = (*_supervoxelizer_ptr_ptr)->getLabeledCloud();

    //Remove NaNs from the labeled point cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud_not_nans(new pcl::PointCloud<pcl::PointXYZL>());
    std::vector<int> unused;
    pcl::removeNaNFromPointCloud< pcl::PointXYZL >(*labeled_cloud, *labeled_cloud_not_nans, unused);

    //Copy labeled point cloud without NaNs to a point cloud of XYZRGB points
    omip::SRPointCloud::Ptr labeled_cloud_not_nans_xyzrgb(new omip::SRPointCloud);
    pcl::copyPointCloud(*labeled_cloud_not_nans, *labeled_cloud_not_nans_xyzrgb);

    //Pass the new point cloud as input to a KNN. We will use it to find neighbors of the features and the points of the model
    this->_knn->setInputCloud(labeled_cloud_not_nans_xyzrgb);

    //Copy the supporting features of this RB into a point cloud without labels (only xyz)
    pcl::PointCloud<pcl::PointXYZ>::Ptr supporting_feats_no_labels(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*this->_supporting_features, *supporting_feats_no_labels);

    //Find the neighbor of each supporting feature in the point cloud of supervoxels -> find the supervoxel that contains each supporting feature
    //and add the supervoxels to the candidates
    for(int idx_sf =0; idx_sf<supporting_feats_no_labels->points.size(); idx_sf++)
    {
        std::vector<int> neighbors;
        std::vector<float> distances;
        this->_knn->nearestKSearchT<pcl::PointXYZ>(supporting_feats_no_labels->points[idx_sf], 1, neighbors, distances);
        if(std::find(labels_to_extend.begin(), labels_to_extend.end(), labeled_cloud_not_nans->points[neighbors[0]].label) == labels_to_extend.end())
        {
            if(labeled_cloud_not_nans->points[neighbors[0]].label != 0 )
            {
                labels_to_extend.push_back(labeled_cloud_not_nans->points[neighbors[0]].label);
            }
        }
    }

    // THIS PART COULD BE IMPROVED TO ACCELERATE THE PROCESS!
    // Move the model to the current location
    // and find neighbors in the labeled point cloud (supervoxels)
    omip::SRPointCloud::Ptr model_in_current_frame(new omip::SRPointCloud);
    std::vector<std::vector<int > > indices_model;
    std::vector<std::vector<float > > unused2;
    unused.clear();
    if(this->_rb_shape->points.size())
    {
        pcl::transformPointCloud<SRPoint>(*this->_rb_shape, *model_in_current_frame, this->_current_HTransform.cast<float>());
        this->_knn->radiusSearch(*model_in_current_frame, unused, this->_knn_min_radius, indices_model, unused2);
    }

    // Build a set that contains all the labels of the supervoxels containing points of the depth-based
    // and the indices of the points of the supervoxels that are neighbors of the models (this avoids counting the same point several times)
    std::map<uint32_t, std::unordered_set<int> > labels_of_segments_to_extend;
    for (unsigned int i = 0; i < indices_model.size(); i++)
    {
        for(unsigned int k=0; k < indices_model[i].size(); k++)
        {
            if(indices_model[i].at(k))
            {
                // For the first point we need to create the entry of the map
                if(labels_of_segments_to_extend.find(labeled_cloud_not_nans->points.at(indices_model[i].at(k)).label) ==  labels_of_segments_to_extend.end())
                {
                    std::unordered_set<int> initial_set_of_indices = {indices_model[i].at(k)};
                    labels_of_segments_to_extend[labeled_cloud_not_nans->points.at(indices_model[i].at(k)).label] = initial_set_of_indices;
                }
                else{
                    // If this point was not counted before as neighbor of another point we add its label
                    if(labels_of_segments_to_extend.at(labeled_cloud_not_nans->points.at(indices_model[i].at(k)).label).find(indices_model[i].at(k))
                            == labels_of_segments_to_extend.at(labeled_cloud_not_nans->points.at(indices_model[i].at(k)).label).end())
                    {
                        labels_of_segments_to_extend.at(labeled_cloud_not_nans->points.at(indices_model[i].at(k)).label).insert(indices_model[i].at(k));
                    }
                }
            }
        }
    }

    std::map<uint32_t, std::unordered_set<int> >::iterator it = labels_of_segments_to_extend.begin();
    std::map<uint32_t, std::unordered_set<int> >::iterator itend = labels_of_segments_to_extend.end();
    // Iterate over the map-counter and add the supervoxels that contain enough points
    for(; it != itend; it++)
    {
        if(it->second.size() > _min_number_model_pixels_in_sv && std::find(labels_to_extend.begin(), labels_to_extend.end(), it->first) == labels_to_extend.end())
        {
            if(it->first != 0)
            {
                labels_to_extend.push_back(it->first);
            }
        }
    }
    // END OF (THIS PART COULD BE IMPROVED TO ACCELERATE THE PROCESS!)

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    (*_supervoxelizer_ptr_ptr)->getSupervoxelAdjacency (supervoxel_adjacency);

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._ExtendPointsToRegions",
                          "[RB" << this->_rb_id << "]: Superpixels to extend the results: " << labels_to_extend.size());

    if(this->_extend_to_neighbor_sv)
    {
        this->_ExtendToNeighborSV(supervoxel_adjacency, labels_to_extend);

        ROS_INFO_STREAM_NAMED("ShapeReconstruction._ExtendPointsToRegions",
                              "[RB" << this->_rb_id << "]: Superpixels to extend the results (after extending to SV of similar properties): " << labels_to_extend.size());
    }

    // Add the points of the supervoxels to extend
    shape_reconstruction::PassThrough<pcl::PointXYZL>::Ptr pt_filter(new shape_reconstruction::PassThrough<pcl::PointXYZL>());
    pt_filter->setInputCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL> >(labeled_cloud_not_nans));
    pt_filter->setFilterFieldName("label");
    omip::SRPointCloud::Ptr points_of_current_in_origin(new omip::SRPointCloud);
    pcl::transformPointCloud<SRPoint>(*this->_current_ffs._pc_without_nans, *points_of_current_in_origin, this->_current_HTransform_inv.cast<float>());

    pt_filter->setLabels(labels_to_extend);

    boost::shared_ptr<std::vector<int> > passing_indices_ptr = boost::shared_ptr<std::vector<int> >(new std::vector<int>());
    pt_filter->filter(*passing_indices_ptr);

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._ExtendPointsToRegions",
                           "[RB" << this->_rb_id << "]: Adding " << (*passing_indices_ptr).size() << " points to the extended model (currently " <<this->_rb_shape->width << " points)");
    for (unsigned int i = 0; i < (*passing_indices_ptr).size(); i++)
    {
        this->_rb_shape->points.push_back(points_of_current_in_origin->points[(*passing_indices_ptr)[i]]);
        this->_rb_shape->width++;
    }
}

void ShapeReconstruction::_ExtendToNeighborSV(std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                                              std::vector<uint32_t>& labels_extension)
{
    //EXPERIMENTAL: Naive extension of the SV to neighboring SV with similar properties

    int number_of_sv = _supervoxel_clusters_ptr->size();
    std::vector<uint32_t> labels_extension_safety_copy = labels_extension;

    //We retrieve the range of elements with each of the added SV
    std::pair< std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator > neighbors;

    int num_extended_labels = labels_extension.size();
    int num_extended_labels_with_neigbors = 0;

    // This repeats until we do not add any other label
    while(num_extended_labels != num_extended_labels_with_neigbors)
    {
        num_extended_labels = labels_extension.size();
        std::vector<uint32_t> labels_extension_copy = labels_extension;
        for(std::vector<uint32_t>::iterator labels_it = labels_extension.begin(); labels_it!=labels_extension.end(); labels_it++)
        {
            // Find the neighbors of the sv
            neighbors =supervoxel_adjacency.equal_range(*labels_it);

            // Query the supervoxel to be extended
            if(_supervoxel_clusters_ptr->find(*labels_it) != _supervoxel_clusters_ptr->end())
            {
                pcl::Supervoxel<pcl::PointXYZRGB>::Ptr sv = _supervoxel_clusters_ptr->at(*labels_it);

                // Normal of the supervoxel to be extended
                Eigen::Vector3d sv_normal(sv->normal_.data_n[0], sv->normal_.data_n[1], sv->normal_.data_n[2]);

                // Position and color (RGB) of the supervoxel to be extended
                pcl::PointXYZRGB sv_centroid_xyzrgb( sv->centroid_.r, sv->centroid_.g, sv->centroid_.b);
                sv_centroid_xyzrgb.x = sv->centroid_.x;
                sv_centroid_xyzrgb.y = sv->centroid_.y;
                sv_centroid_xyzrgb.z = sv->centroid_.z;

                // Position and color (HSV) of the supervoxel to be extended
                pcl::PointXYZHSV sv_centroid_xyzhsv;
                pcl::PointXYZRGBtoXYZHSV(sv_centroid_xyzrgb, sv_centroid_xyzhsv);

                // Iterate over the pairs of sv-neighbor
                for(std::multimap<uint32_t, uint32_t>::iterator neighbor_it = neighbors.first; neighbor_it!= neighbors.second; ++neighbor_it)
                {
                    // Check if the neighbor was already added
                    if(std::find(labels_extension_copy.begin(), labels_extension_copy.end(), neighbor_it->second)  == labels_extension_copy.end() )
                    {
                        // Query the neighbor supervoxel to extend to
                        pcl::Supervoxel<pcl::PointXYZRGB>::Ptr nsv = _supervoxel_clusters_ptr->at(neighbor_it->second);

                        // Normal of the neighbor supervoxel to extend to
                        Eigen::Vector3d nsv_normal(nsv->normal_.data_n[0], nsv->normal_.data_n[1], nsv->normal_.data_n[2]);

                        // Position and color (RGB) of the neighbor supervoxel to extend to
                        pcl::PointXYZRGB nsv_centroid_xyzrgb( nsv->centroid_.r, nsv->centroid_.g, nsv->centroid_.b);
                        nsv_centroid_xyzrgb.x = nsv->centroid_.x;
                        nsv_centroid_xyzrgb.y = nsv->centroid_.y;
                        nsv_centroid_xyzrgb.z = nsv->centroid_.z;

                        // Position and color (HSV) of the neighbor supervoxel to extend to
                        pcl::PointXYZHSV nsv_centroid_xyzhsv;
                        pcl::PointXYZRGBtoXYZHSV(nsv_centroid_xyzrgb, nsv_centroid_xyzhsv);

                        // We compare the mean normal and the color of the centroid point
                        // CAREFUL: h values in pcl are between 0 and 360
                        double h_diff = std::fabs(sv_centroid_xyzhsv.h - nsv_centroid_xyzhsv.h);
                        h_diff = std::fmod(h_diff, 360.0);
                        h_diff = h_diff > 180.0 ? 360.0 - h_diff : h_diff;
                        // TODO: Handle black and white regions (use s and v)

                        if(std::fabs(sv_normal.dot(nsv_normal) - 1.0) < _similarity_in_normal*(M_PI/180.0) && h_diff < _similarity_in_h)
                        {
                            labels_extension_copy.push_back(neighbor_it->second);
                        }
                    }
                }
            }
        }
        num_extended_labels_with_neigbors = labels_extension_copy.size();
        labels_extension = labels_extension_copy;
    }

    // Emergency brake! -> We are adding too many svs
    int added_supervoxels = labels_extension.size() - labels_extension_safety_copy.size();
    // Options: max number of sv to add (absolute) or max fraction of the total amount of sv to add (relative to the total amount of sv)
    if((double)added_supervoxels/(double)number_of_sv > 0.2 )
    {
        ROS_INFO_STREAM_NAMED("ShapeReconstruction._ExtendPointsToRegions","We are adding too many supervoxels! Undo!");
        labels_extension = labels_extension_safety_copy;
    }
}

void ShapeReconstruction::_EstimateTransformations()
{
    ROSTwist2EigenTwist(this->_previous_ffs._transformation.twist, this->_previous_twist);
    Twist2TransformMatrix(this->_previous_twist, this->_previous_HTransform);
    this->_previous_HTransform_inv = this->_previous_HTransform.inverse();

    ROSTwist2EigenTwist(this->_current_ffs._transformation.twist, this->_current_twist);
    Twist2TransformMatrix(this->_current_twist, this->_current_HTransform);
    this->_current_HTransform_inv = this->_current_HTransform.inverse();

    ROS_DEBUG_STREAM_NAMED("ShapeReconstruction._EstimateTransformations",  "[RB" << this->_rb_id << "]: _current_HTransform " << std::endl <<
                           _current_HTransform);

    this->_current_to_previous_HTransform = this->_current_HTransform*this->_previous_HTransform_inv;

    this->_previous_to_current_HTransform = this->_current_to_previous_HTransform.inverse();

    ROS_DEBUG_STREAM_NAMED("ShapeReconstruction._EstimateTransformations",  "[RB" << this->_rb_id << "]: current to previous " << std::endl <<
                           _current_to_previous_HTransform);
}

void ShapeReconstruction::_GenerateMesh(const omip::SRPointCloud::Ptr& pc_source,
                                        std::string shape_file_prefix)
{
    if(pc_source->points.size())
    {
        // Normal estimation*
        this->_rb_tree_for_normal_estimation_ptr->setInputCloud(pc_source);
        this->_rb_normal_estimator_ptr->setInputCloud(pc_source);
        this->_rb_normal_estimator_ptr->setSearchMethod (this->_rb_tree_for_normal_estimation_ptr);
        this->_rb_normal_estimator_ptr->compute(*this->_rb_estimated_normals_pc_ptr);

        // Concatenate the XYZ and normal fields*
        pcl::concatenateFields (*pc_source, *this->_rb_estimated_normals_pc_ptr, *this->_rb_position_color_and_normals_pc_ptr);
        this->_rb_tree_for_triangulation_ptr->setInputCloud (this->_rb_position_color_and_normals_pc_ptr);

        // Get result
        this->_rb_greedy_projection_triangulator_ptr->setInputCloud(this->_rb_position_color_and_normals_pc_ptr);
        this->_rb_greedy_projection_triangulator_ptr->setSearchMethod(this->_rb_tree_for_triangulation_ptr);
        this->_rb_greedy_projection_triangulator_ptr->reconstruct(*this->_rb_triangulated_mesh_ptr);

        // Additional vertex information
        //std::vector<int> parts = gp3.getPartIDs();
        //std::vector<int> states = gp3.getPointStates();

        std::string mesh_path = ros::package::getPath("shape_reconstruction") + std::string("/meshes/");
        std::stringstream rb_name_ss;
        rb_name_ss << shape_file_prefix << this->_rb_id << ".stl";

        std::string rb_mesh_full_file_name = mesh_path + rb_name_ss.str();

        pcl::io::mesh2vtk(*this->_rb_triangulated_mesh_ptr, this->_rb_polygon_data_ptr);

        this->_rb_polygon_writer_ptr->SetInputData (this->_rb_polygon_data_ptr);
        this->_rb_polygon_writer_ptr->SetFileName (rb_mesh_full_file_name.c_str ());
        this->_rb_polygon_writer_ptr->Write ();

        ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh",  "[RB" << this->_rb_id << "]: Resulting triangular mesh written to " << rb_mesh_full_file_name);
    }else{
        ROS_WARN_STREAM_NAMED("ShapeReconstruction.generateMesh",  "[RB" << this->_rb_id << "]: Impossible to generate a triangular mesh for this model, it doesn't contain any points!");
    }
}

void ShapeReconstruction::generateMesh()
{
    ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh",  "[RB" << this->_rb_id << "]: Called the generation of 3D triangular mesh from the model of RB" << this->_rb_id <<
                          " based on extending other models with supervoxels.");
    std::string shape_ext_d_and_c_file_prefix("shape_ext_d_and_c_rb");
    this->_GenerateMesh(this->_rb_shape, shape_ext_d_and_c_file_prefix);


    ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh",  "[RB" << this->_rb_id << "]: Finished the generation of 3D triangular mesh from the point cloud of RB " << this->_rb_id);
}

void ShapeReconstruction::getShapeModel(omip_msgs::ShapeModelsPtr shapes)
{
    omip_msgs::ShapeModel shape;
    shape.rb_id = this->_rb_id;
    sensor_msgs::PointCloud2 rb_shape_ros;
    rb_shape_ros.header.frame_id = "/camera_rgb_optical_frame";

    this->_rb_shape->header = this->_current_ffs._pc->header;
    pcl::toROSMsg(*this->_rb_shape, rb_shape_ros);

    shape.rb_shape_model = rb_shape_ros;
    shapes->rb_shape_models.push_back(shape);
}

void ShapeReconstruction::PublishMovedModelAndSegment(const ros::Time current_time,
                                                      const geometry_msgs::TwistWithCovariance &rb_transformation,
                                                      rosbag::Bag& bag,
                                                      bool bagOpen)
{
    Eigen::Twistd current_twist;
    ROSTwist2EigenTwist(rb_transformation.twist, current_twist);

    Eigen::Matrix4d current_HTransform;
    Twist2TransformMatrix(current_twist, current_HTransform);

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._PublishMovedModel",  "[RB" << this->_rb_id << "]: Publishing " << this->_rb_shape->points.size() << " points reconstructed from RB" << this->_rb_id
                          << " based on extending both depth and color model with supervoxels.");
    this->_rb_shape->header = this->_current_ffs._pc->header;
    this->_rb_shape->header.frame_id = "/camera_rgb_optical_frame";
    this->_rb_shape_in_current_frame.reset(new omip::SRPointCloud);
    pcl::transformPointCloud<SRPoint>(*this->_rb_shape, *this->_rb_shape_in_current_frame, current_HTransform.cast<float>());
    sensor_msgs::PointCloud2 rb_shape_ext_d_and_c_in_current_frame_ros;
    pcl::toROSMsg(*this->_rb_shape_in_current_frame, rb_shape_ext_d_and_c_in_current_frame_ros);
    rb_shape_ext_d_and_c_in_current_frame_ros.header.frame_id = "/camera_rgb_optical_frame";
    this->_rb_shape_pub.publish(rb_shape_ext_d_and_c_in_current_frame_ros);

    if (bagOpen){
        bag.write(_rb_shape_pub.getTopic(), current_time, rb_shape_ext_d_and_c_in_current_frame_ros);
    }

    // segment
    {
        this->_rb_segment->header = this->_current_ffs._pc->header;
        this->_rb_segment->header.frame_id = "/camera_rgb_optical_frame";
        sensor_msgs::PointCloud2 rb_segment_in_current_frame_ros;
        pcl::toROSMsg(*this->_rb_segment, rb_segment_in_current_frame_ros);
        rb_segment_in_current_frame_ros.header.frame_id = "/camera_rgb_optical_frame";
        this->_rb_segment_pub.publish(rb_segment_in_current_frame_ros);
        if (bagOpen){
            bag.write(_rb_segment_pub.getTopic(), current_time, rb_segment_in_current_frame_ros);
        }
    }
}

void ShapeReconstruction::RemoveInconsistentPoints(const SRPointCloud::Ptr &pc_msg,
                                                   const geometry_msgs::TwistWithCovariance &rb_transformation)
{
    Eigen::Twistd current_twist;
    ROSTwist2EigenTwist(rb_transformation.twist, current_twist);
    Eigen::Matrix4d current_HTransform;
    Twist2TransformMatrix(current_twist, current_HTransform);

    SRPointCloud::Ptr current_pc(new SRPointCloud());
    pcl::copyPointCloud(*pc_msg, *current_pc);

    // Create a depth map from the current organized depth map
    cv::Mat current_dm;
    this->_current_ffs._dm->image.copyTo(current_dm);   // Just to initialize current_dm correctly
    OrganizedPC2DepthMap(current_pc, current_dm);

    this->_RemoveInconsistentPointsFromRBModel("", current_HTransform, this->_rb_shape, current_dm,current_pc, this->_rb_segment);

}
