#include "shape_reconstruction/ShapeReconstructionNode.h"

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include <boost/filesystem.hpp>
#include <ctime>

#include <std_msgs/Bool.h>

using namespace omip;

ShapeReconstructionNode::ShapeReconstructionNode():
    _active(true),
    _frame_ctr(0),
    _max_rb(100),
    _detect_static_environment(false),
    _record_result_bag(false),
    _result_bag_open(false),
    _manually_advance_after_processing(false),
    _sv_estimated(false),
    _refine_shapes_in_each_frame(false),
    _processing_interval(0.0),
    _estimate_sv(false)
{
    this->_rbshapes_combined_publisher = this->_node_handle.advertise<sensor_msgs::PointCloud2>("rbshapes_comb",10);
    this->_rgbd_pc_subscriber.subscribe(this->_node_handle, "/camera/depth_registered/points",1);

    this->_clustered_feats_subscriber.subscribe(this->_node_handle, "/rb_tracker/clustered_tracked_feats", 1);

    this->_node_quit_subscriber = this->_node_handle.subscribe("/omip/shutdown", 1,
                                                               &ShapeReconstructionNode::TrackerQuitCallback, this);

    this->_poses_and_vels_subscriber.subscribe(this->_node_handle, "/rb_tracker/state",1);

    this->_synchronizer_with_clustered_feats =
            new message_filters::Synchronizer<SRSyncPolicyWithClusteredFeatures>(SRSyncPolicyWithClusteredFeatures(10),
                                                                                 this->_rgbd_pc_subscriber,
                                                                                 this->_poses_and_vels_subscriber,
                                                                                 this->_clustered_feats_subscriber);
    this->_synchronizer_with_clustered_feats->registerCallback(boost::bind(&ShapeReconstructionNode::measurementCallbackWithClusteredFeatures, this, _1, _2, _3));

    this->_previous_measurement_time = ros::Time(0.0);

    this->_mesh_generator_srv = this->_node_handle.advertiseService("/shape_reconstruction/mesh_generator_srv", &ShapeReconstructionNode::generateMeshes, this);

    this->_node_handle.getParam("/shape_reconstruction/max_rb", _max_rb);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","Maximal number of rigid bodies tracked: " << _max_rb);

    this->_node_handle.getParam("/shape_reconstruction/detect_static_environment", this->_detect_static_environment);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","detect_static_environment = " << this->_detect_static_environment);

    this->_node_handle.getParam("/shape_reconstruction/publish_shapes_in_each_frame", this->_publish_shapes_in_each_frame);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","publish_shapes_in_each_frame = " << this->_publish_shapes_in_each_frame);

    this->_node_handle.getParam("/shape_reconstruction/refine_shapes_in_each_frame", this->_refine_shapes_in_each_frame);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","refine_shapes_in_each_frame = " << this->_refine_shapes_in_each_frame);

    this->_node_handle.getParam("/shape_reconstruction/manually_advance_after_processing", this->_manually_advance_after_processing);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","manually_advance_after_processing = " << this->_manually_advance_after_processing);

    this->_node_handle.getParam("/shape_reconstruction/processing_interval", this->_processing_interval);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","processing_interval = " << this->_processing_interval);

    this->_node_handle.getParam("/shape_reconstruction/estimate_sv", this->_estimate_sv);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","_estimate_sv = " << this->_estimate_sv);


    this->_current_pc.reset(new SRPointCloud());
    this->_current_pc_down.reset(new SRPointCloud());
    this->_current_pc_without_nans.reset(new SRPointCloud());

    this->_clustered_feats.reset(new pcl::PointCloud<pcl::PointXYZL>);

    this->_static_environment_ext_d_and_c_pub = this->_node_handle.advertise<sensor_msgs::PointCloud2>("/shape_recons/shape_static_environment", 1);

    this->_shape_models_ext_d_and_c_pub = this->_node_handle.advertise<omip_msgs::ShapeModels>("/shape_recons/state", 1);

    this->_advance_feature_tracker_pub = this->_node_handle.advertise<std_msgs::Bool>("segmentation_info_msg", 1);

    /// For SE shape, defined as PC - RB (it is wrong if there are other RBs)
    this->_se_polygon_data_ptr = vtkSmartPointer<vtkPolyData>::New ();
    this->_se_polygon_writer_ptr = vtkSmartPointer<vtkSTLWriter>::New ();
    this->_se_polygon_writer_ptr->SetFileTypeToBinary();
    this->_se_normal_estimator_ptr.reset(new pcl::NormalEstimationOMP<omip::SRPoint, pcl::Normal>());
    this->_se_normal_estimator_ptr->setKSearch (50);
    this->_se_estimated_normals_pc_ptr.reset(new pcl::PointCloud<pcl::Normal>());
    this->_se_tree_for_normal_estimation_ptr.reset(new pcl::search::KdTree<omip::SRPoint>());
    this->_se_position_color_and_normals_pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    this->_se_tree_for_triangulation_ptr.reset(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
    this->_se_greedy_projection_triangulator_ptr.reset(new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>());
    // Set typical values for the parameters
    this->_se_greedy_projection_triangulator_ptr->setMu (2.5);
    this->_se_greedy_projection_triangulator_ptr->setMaximumNearestNeighbors (100);
    this->_se_greedy_projection_triangulator_ptr->setMaximumSurfaceAngle(M_PI); // 180 degrees
    this->_se_greedy_projection_triangulator_ptr->setMinimumAngle(M_PI/36); // 5 degrees
    this->_se_greedy_projection_triangulator_ptr->setMaximumAngle(M_PI/2); // 180 degrees
    this->_se_greedy_projection_triangulator_ptr->setNormalConsistency(true);
    this->_se_greedy_projection_triangulator_ptr->setConsistentVertexOrdering(true);
    // Set the maximum distance between connected points (maximum edge length)
    this->_se_greedy_projection_triangulator_ptr->setSearchRadius (0.3);
    this->_se_triangulated_mesh_ptr.reset(new pcl::PolygonMesh());

    this->_node_handle.getParam("/shape_reconstruction/record_result_bag", _record_result_bag);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","record_result_bag = " << _record_result_bag);

    if (_record_result_bag) {
        if (!this->_node_handle.getParam("/shape_reconstruction/result_bag_path", _result_bag_path)) {
            _result_bag_path = "/tmp";
        }
        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","storing result bag in = " << _result_bag_path);
    }

    this->_voxel_resolution = 0.008f;
    this->_seed_resolution = 0.2f;
    this->_color_importance = 0.4f;
    this->_spatial_importance = 0.4f;
    this->_normal_importance = 0.8f;


    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_voxel_resolution"), this->_voxel_resolution);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_voxel_resolution = " << this->_voxel_resolution);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_seed_resolution"), this->_seed_resolution);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_seed_resolution = " << this->_seed_resolution);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_color_importance"), this->_color_importance);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_color_importance = " << this->_color_importance);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_spatial_importance"), this->_spatial_importance);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_spatial_importance = " << this->_spatial_importance);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_normal_importance"), this->_normal_importance);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_normal_importance = " << this->_normal_importance);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_visualization"), this->_visualization_sv);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_visualization = " << this->_visualization_sv);

    this->_node_handle.getParam(std::string("/shape_reconstruction/sv_visualization"), this->_visualization_sv);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode","sv_visualization = " << this->_visualization_sv);

    std::string ci_topic;
    this->getROSParameter<std::string>(std::string("/feature_tracker/camera_info_topic"),ci_topic);
    this->_ci_sub = this->_node_handle.subscribe(ci_topic, 1,
                                                 &ShapeReconstructionNode::CameraInfoCallback, this);
}

ShapeReconstructionNode::~ShapeReconstructionNode()
{
    if(_result_bag_open) {
        this->_result_bag.close();
        this->_video_bag.close();
    }
}

void ShapeReconstructionNode::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci_msg)
{
    // We have to do this because sometimes the first ci message that is sent is zero (why?)
    if(ci_msg->height != 0)
    {
        this->_ci = sensor_msgs::CameraInfo(*ci_msg);
        this->_ci_sub.shutdown();
        for(std::map<omip::RB_id_t,omip::ShapeReconstructionPtr >::iterator it = this->_rb_shapes.begin(); it!=this->_rb_shapes.end(); it++)
        {
            it->second->setCameraInfo(this->_ci);
        }
    }
}

omip::ShapeReconstructionPtr ShapeReconstructionNode::_createNewShapeReconstruction()
{
    // We create a new ShapeReconstruction object for this RB and we add it to the map
    ROS_WARN_STREAM_NAMED("ShapeReconstructionNode::measurementCallback","Create a new ShapeReconstructor for the newly detected RB.");

    bool to_initial=false;
    this->_node_handle.getParam("/shape_reconstruction/to_initial", to_initial);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","to_initial = " << to_initial);

    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","detect_static_environment = " << this->_detect_static_environment);

    double color_changes_erode_size=false;
    this->_node_handle.getParam("/shape_reconstruction/color_changes_erode_size", color_changes_erode_size);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","color_changes_erode_size = " << color_changes_erode_size);

    bool depth_filling=false;
    this->_node_handle.getParam("/shape_reconstruction/depth_filling", depth_filling);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","depth_filling = " << depth_filling);

    bool remove_inconsistent_points=false;
    this->_node_handle.getParam("/shape_reconstruction/remove_inconsistent_points", remove_inconsistent_points);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","remove_inconsistent_points = " << remove_inconsistent_points);

    bool accumulate_change_candidates=false;
    this->_node_handle.getParam("/shape_reconstruction/accumulate_change_candidates", accumulate_change_candidates);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","accumulate_change_candidates = " << accumulate_change_candidates);

    double min_depth_change;
    this->_node_handle.getParam("/shape_reconstruction/min_depth_change", min_depth_change);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","min_depth_change = " << min_depth_change);

    int min_color_change;
    this->_node_handle.getParam("/shape_reconstruction/min_color_change", min_color_change);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","min_color_change = " << min_color_change);

    double knn_min_radius;
    this->_node_handle.getParam("/shape_reconstruction/knn_min_radius", knn_min_radius);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","knn_min_radius = " << knn_min_radius);

    int min_num_model_pixels_in_sv;
    this->_node_handle.getParam("/shape_reconstruction/sv_min_num_model_pixels", min_num_model_pixels_in_sv);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","sv_min_num_model_pixels = " << min_num_model_pixels_in_sv);

    bool record_videos;
    this->_node_handle.getParam("/shape_reconstruction/record_videos", record_videos);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","record_videos = " << record_videos);

    bool extend_model_to_neighbor_sv;
    this->_node_handle.getParam("/shape_reconstruction/extend_model_to_neighbor_sv", extend_model_to_neighbor_sv);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","extend_model_to_neighbor_sv = " << extend_model_to_neighbor_sv);

    double similarity_in_h;
    this->_node_handle.getParam("/shape_reconstruction/similarity_in_h", similarity_in_h);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","similarity_in_h = " << similarity_in_h);

    double similarity_in_normal;
    this->_node_handle.getParam("/shape_reconstruction/similarity_in_h", similarity_in_normal);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","similarity_in_normal = " << similarity_in_normal);

    bool use_clustered_feats;
    this->_node_handle.getParam("/shape_reconstruction/use_clustered_feats", use_clustered_feats);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","use_clustered_feats = " << use_clustered_feats);

    bool data_from_bag;
    this->_node_handle.getParam("/feature_tracker/data_from_bag", data_from_bag);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","data_from_bag = " << data_from_bag);

    double avg_leaf_size;
    this->_node_handle.getParam("/shape_reconstruction/avg_leaf_size", avg_leaf_size);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","avg_leaf_size = " << avg_leaf_size);

    double ror_radius_search;
    this->_node_handle.getParam("/shape_reconstruction/ror_radius_search", ror_radius_search);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","ror_radius_search = " << ror_radius_search);

    int ror_min_neighbors;
    this->_node_handle.getParam("/shape_reconstruction/ror_min_neighbors", ror_min_neighbors);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","ror_min_neighbors = " << ror_min_neighbors);

    // TODO external parameters for depth_filling and detect_changes_in_color
    omip::ShapeReconstructionPtr shape_ptr = omip::ShapeReconstructionPtr(new omip::ShapeReconstruction());

    shape_ptr->setEstimateSV(_estimate_sv);
    shape_ptr->setSuperVoxelizerPtr(this->_supervoxelizer);
    shape_ptr->setSuperVoxelClustersPtr(this->_supervoxel_clusters);
    shape_ptr->setDetectStaticEnvironment(this->_detect_static_environment);
    shape_ptr->setAccumulateChangeCandidates(accumulate_change_candidates);
    shape_ptr->setMinDepthChange(min_depth_change);
    shape_ptr->setMinColorChange(min_color_change);
    shape_ptr->setRemoveInconsistentPoints(remove_inconsistent_points);
    shape_ptr->setKNNMinRadius(knn_min_radius);
    shape_ptr->setSVMinNumberModelPixels(min_num_model_pixels_in_sv);
    shape_ptr->setExtendToNeighborSV(extend_model_to_neighbor_sv);
    shape_ptr->setSimilarityInH(similarity_in_h);
    shape_ptr->setSimilarityInNormal(similarity_in_normal);
    shape_ptr->setCameraInfo(this->_ci);
    //shape_ptr->setSupportingFeatures(features_of_this_rb);
    shape_ptr->useClusteredFeatures(use_clustered_feats);
    shape_ptr->setDepthFilling(depth_filling);
    shape_ptr->setColorChangesErodeSize(color_changes_erode_size);
    shape_ptr->setToInitial(to_initial);
    shape_ptr->setRecordVideos(record_videos);
    shape_ptr->setLiveStream(!data_from_bag);
    shape_ptr->setApproxVoxelGridLeafSize(avg_leaf_size);
    shape_ptr->setRadiusOutRemovalSearch(ror_radius_search);
    shape_ptr->setRadiusOutRemovalMinNeighbors(ror_min_neighbors);    

    return shape_ptr;
}

void ShapeReconstructionNode::measurementCallbackWithClusteredFeatures(const sensor_msgs::PointCloud2ConstPtr &pc_msg,
                                                                       const boost::shared_ptr<omip::rbt_state_t const> &poses_and_vels,
                                                                       const sensor_msgs::PointCloud2ConstPtr &features_pc)
{
    ros::Time ta = ros::Time::now();

    this->_current_pc_down->points.clear();
    this->_current_pc_down->height = IMG_HEIGHT;
    this->_current_pc_down->width = IMG_WIDTH;
    this->_current_pc_down->is_dense = true;


    this->_sv_estimated = false;
    _InitResultRosbag();

    // Extract the time of the upcoming message and estimate the elapsed time since the last message was processed
    _current_measurement_time = pc_msg->header.stamp;
    ros::Duration time_difference = _current_measurement_time - this->_previous_measurement_time;

    if(this->_previous_measurement_time.toSec() != 0.0 )
    {
        ROS_ERROR_STREAM("Time between segmentations: " << time_difference.toSec() << " s");
    }

    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(*pc_msg, *this->_current_pc);

#ifdef DOWNSAMPLING
    // Downsample the input pcl point cloud
    for(int u=0; u<this->_current_pc->width; u+=2)
    {
        for(int v=0; v<this->_current_pc->height; v+=2)
        {
            this->_current_pc_down->points.push_back(this->_current_pc->at(u, v));
        }
    }
#else
    this->_current_pc_down = this->_current_pc;
#endif

    // Convert clustered features message into a pcl point cloud
    pcl::fromROSMsg(*features_pc, *this->_clustered_feats);

    // Clean the input pcl point cloud of nans
    pcl::removeNaNFromPointCloud<SRPoint>(*this->_current_pc_down,*this->_current_pc_without_nans, this->_not_nan_indices);

    // Iterate over the received rb poses and check if a ShapeReconstructor for each tracked RB already exists
    // If not, create a new ShapeReconstructor and pass the current point cloud as initial point cloud
    // NOTE: This is done no matter how much time passed since the last processed message
    omip::RB_id_t rb_id_temp;
    for(int rb_poses_idx = 1; rb_poses_idx < poses_and_vels->rb_poses_and_vels.size(); rb_poses_idx++)
    {
        rb_id_temp = poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).rb_id;

        if(this->_rb_shapes.find(rb_id_temp) == this->_rb_shapes.end())
        {// If this RB (identified by its id) was not previously tracked, we create a new ShapeReconstruction object for it
            // Unless we reached the max number of rb to reconstruct
            if (_rb_shapes.size() >= _max_rb)
            {
                ROS_WARN_ONCE_NAMED("ShapeReconstructionNode.measurementCallback","Maximum limit of RBs for ShapeReconstructor reached");
                continue;
            }

            omip::ShapeReconstructionPtr shape_ptr = this->_createNewShapeReconstruction();

            shape_ptr->setRBId(rb_id_temp);
            shape_ptr->setInitialFullRGBDPCAndRBT(this->_current_pc_down,
                                                  poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc);
            shape_ptr->initialize();

            this->_rb_shapes[rb_id_temp] = shape_ptr;
        }
        else
        {// If this RB (identified by its id) was previously tracked, we already have a ShapeReconstruction object for it
            if(_refine_shapes_in_each_frame)
            {
                this->_rb_shapes[rb_id_temp]->RemoveInconsistentPoints(this->_current_pc_down,
                                                                       poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc);
            }

            if(_publish_shapes_in_each_frame)
            {
                this->_rb_shapes[rb_id_temp]->PublishMovedModelAndSegment(_current_measurement_time,
                                                                          poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc,
                                                                          this->_video_bag,
                                                                          this->_result_bag_open);
            }
        }

        // Extract all the features that support the rigid body
        pcl::PassThrough<pcl::PointXYZL>::Ptr pt_filter(new pcl::PassThrough<pcl::PointXYZL>());
        pt_filter->setInputCloud(this->_clustered_feats);
        pt_filter->setFilterFieldName("label");
        pcl::PointCloud<pcl::PointXYZL>::Ptr features_of_this_rb(new pcl::PointCloud<pcl::PointXYZL>);

        uint32_t lower_bound = rb_id_temp;
        float lower_bound_float = 0;
        memcpy (&lower_bound_float, &lower_bound, sizeof (float));
        uint32_t upper_bound = rb_id_temp;
        float upper_bound_float = 0;
        memcpy (&upper_bound_float, &upper_bound, sizeof (float));
        pt_filter->setFilterLimits(lower_bound_float, upper_bound_float);
        //std::vector<int> passing_indices;
        pcl::IndicesPtr passing_indices(new std::vector<int>);
        pt_filter->filter(*passing_indices);
        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.measurementCallbackWithClusteredFeatures",
                              "[RB" << rb_id_temp << "]: " << passing_indices->size() << " of " << this->_clustered_feats->points.size() << " features support this RB.");

        // Extract the features from the clustered features point cloud
        this->_extractor_cf.setInputCloud(this->_clustered_feats);
        this->_extractor_cf.setIndices(passing_indices);
        this->_extractor_cf.filter(*features_of_this_rb);

        // Set the supporting features of this rigid body
        this->_rb_shapes[rb_id_temp]->setSupportingFeatures(features_of_this_rb);
    }

    // Check if the elapsed time since the last processed message is over the processing time interval
    // If not, return and ignore the measurement
    // If yes, process this measurement
    if(time_difference.toSec() < this->_processing_interval && this->_previous_measurement_time.toSec() != 0.0)
    {
        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode::measurementCallback","Not processing this measurement.");
        _AdvanceFeatureTracker();
        return;
    }else{
        ROS_ERROR_STREAM_NAMED("ShapeReconstructionNode::measurementCallback","Processing this measurement.");
    }

    ros::Time te = ros::Time::now();

    // write current point cloud, depth and rgb image
    if (_result_bag_open) {
        this->_result_bag.write(_rgbd_pc_subscriber.getTopic(), _current_measurement_time, *pc_msg);

        // Create a depth map from the current organized depth map
        cv::Mat depth_image_raw(this->_current_pc->height, this->_current_pc->width, CV_32FC1);
        OrganizedPC2DepthMap(this->_current_pc, depth_image_raw);
        //        cv::Mat depth_image(depth_image_raw.rows, depth_image_raw.cols, CV_8UC1);
        //        cv::convertScaleAbs(depth_image_raw, depth_image, 100, 0);
        //        sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_image).toImageMsg();
        sensor_msgs::ImagePtr depth_msg;
        DepthImage2CvImage(depth_image_raw, depth_msg);

        this->_result_bag.write("/camera/depth_registered/image_rect", _current_measurement_time, *depth_msg);

        cv::Mat color_image(this->_current_pc->height, this->_current_pc->width, CV_8UC3);
        OrganizedPC2ColorMap(this->_current_pc, color_image);
        sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();

        this->_result_bag.write("/camera/rgb/image_rect_color", _current_measurement_time, *color_msg);

    }

    ros::Time te2;
    ros::Time te3;
    // If we process the measurement we pass it to each ShapeReconstructor
    // We create a temp mapping so that any existing RBs get deleted
    std::map<omip::RB_id_t,omip::ShapeReconstructionPtr > rb_shapes_updated;
    for(int rb_poses_idx = 1; rb_poses_idx < poses_and_vels->rb_poses_and_vels.size(); rb_poses_idx++)
    {
        rb_id_temp = poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).rb_id;

        // If this RB (identified by its id) was previously tracked, we already have a ShapeReconstruction object for it
        if(this->_rb_shapes.find(rb_id_temp) != this->_rb_shapes.end())
        {
            rb_shapes_updated[rb_id_temp] = this->_rb_shapes[rb_id_temp];

            if(!this->_sv_estimated && _estimate_sv)
            {
                te2= ros::Time::now();
                this->_EstimateSupervoxels();
                te3= ros::Time::now();
            }

            rb_shapes_updated[rb_id_temp]->setFullRGBDPCandRBT(this->_current_pc_down,
                                                               poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc);
            rb_shapes_updated[rb_id_temp]->PublishMovedModelAndSegment(_current_measurement_time,
                                                                       poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc,
                                                                       this->_video_bag,
                                                                       this->_result_bag_open);
        }else{
            ROS_WARN_STREAM_NAMED("ShapeReconstructionNode.measurementCallback","There is no ShapeReconstructor associated to this RB id - it was detected before max_rb was reached");
        }
    }
    // Swap the maps (in this way, the entries of the map that were not updated because the RB was lost, get deleted)
    this->_rb_shapes = rb_shapes_updated;

    ros::Time tf = ros::Time::now();

    this->_CollectAndPublishShapeModels();

    // Write the results into bag
    if (_result_bag_open) {
        int rb_poses_idx=1;
        BOOST_FOREACH(shape_reconstructors_map_t::value_type rb_shape_it, this->_rb_shapes)
        {
            rb_shape_it.second->PublishMovedModelAndSegment(_current_measurement_time, poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).pose_wc, _result_bag, _result_bag_open);
            rb_poses_idx++;
        }
    }

    // we have processed the measurement - if the flag
    if (this->_manually_advance_after_processing) {
        std::cout << " Hit enter to advance feature tracker" << std::endl;
        getchar();
    }

    _AdvanceFeatureTracker();

    ros::Time th = ros::Time::now();

//    std::cout << "SV estimation: " << (te3-te2).toSec() << std::endl;
//    std::cout << "Process RGBD frame and publish results (including SV estimation): " << (tf-te).toSec() << std::endl;
//    std::cout << "Total: " << (th-ta).toSec() << std::endl;

    // Update previous time
    this->_previous_measurement_time = _current_measurement_time;
}

void ShapeReconstructionNode::ReadRosBag()
{
}

void ShapeReconstructionNode::_CollectAndPublishShapeModels()
{
    omip_msgs::ShapeModelsPtr shape_models(new omip_msgs::ShapeModels());
    shape_models->header.stamp = this->_current_measurement_time;

    static ros::Publisher* current_pc_repub = NULL;
    if (current_pc_repub == NULL) {
        current_pc_repub = new ros::Publisher;
    }
    *current_pc_repub = this->_node_handle.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);

    // First we publish each shape moved with the latest motion
    BOOST_FOREACH(shape_reconstructors_map_t::value_type rb_shape_it, this->_rb_shapes)
    {
        rb_shape_it.second->getShapeModel(shape_models);

        if (_manually_advance_after_processing) {
            std::cout << "Republishing depth registered points (in order to sync ground truth) [to switch off set manually_advance_after_processing=false" << std::endl;

            SRPointCloud::Ptr current_pc = rb_shape_it.second->getCurrentPointCloud();
            sensor_msgs::PointCloud2 current_pc_msg;
            pcl::toROSMsg(*current_pc, current_pc_msg);
            current_pc_repub->publish(current_pc_msg);
        }
    }

    _shape_models_ext_d_and_c_pub.publish(*shape_models);

    // Then we query all the shapes in the current frame (transformed using the latest motion) and subtract them from the current point cloud to obtaint the static environment
    if(this->_detect_static_environment)
    {

        pcl::PointIndices::Ptr indices_of_moving_bodies_ptr = pcl::PointIndices::Ptr(new pcl::PointIndices());
        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode._PublishShapes",
                              "Subtracting reconstructed shapes (obtained from the extension of both models to supervoxels) from current point cloud to estimate the static environment.");

        this->_set_knn_ptr.reset(new pcl::search::KdTree<omip::SRPoint>());
        this->_set_knn_ptr->setInputCloud(this->_current_pc_without_nans);

        indices_of_moving_bodies_ptr.reset(new pcl::PointIndices());
        BOOST_FOREACH(shape_reconstructors_map_t::value_type rb_shape_it, this->_rb_shapes)
        {
            std::vector<int> empty;
            std::vector<std::vector<int> > indices_of_moving_body;
            std::vector<std::vector<float> > distances_of_moving_body;
            SRPointCloud::Ptr rb_shape_ext_moved_ptr = rb_shape_it.second->getMovedRigidBodyShapeExtDandC();
            this->_set_knn_ptr->radiusSearch(*rb_shape_ext_moved_ptr, empty, 0.02, indices_of_moving_body, distances_of_moving_body, 0);

            for(int idx = 0; idx < indices_of_moving_body.size(); idx++)
            {
                if(indices_of_moving_body.at(idx).size())
                {
                    for(int i = 0; i<indices_of_moving_body.at(idx).size(); i++)
                    {
                        indices_of_moving_bodies_ptr->indices.push_back(indices_of_moving_body.at(idx).at(i));
                    }
                }
            }
        }
        std::set<int> tmp_d_and_c(indices_of_moving_bodies_ptr->indices.begin(), indices_of_moving_bodies_ptr->indices.end());
        indices_of_moving_bodies_ptr->indices.clear();
        std::copy(tmp_d_and_c.begin(), tmp_d_and_c.end(), std::back_inserter(indices_of_moving_bodies_ptr->indices));

        pcl::ExtractIndices<SRPoint> extract_d_and_c;
        this->_static_env_ext_d_and_c_in_current_frame = omip::SRPointCloud::Ptr(new omip::SRPointCloud());
        // Extract the inliers
        extract_d_and_c.setInputCloud (this->_current_pc_without_nans);
        extract_d_and_c.setIndices (indices_of_moving_bodies_ptr);
        extract_d_and_c.setNegative (true);
        extract_d_and_c.filter (*this->_static_env_ext_d_and_c_in_current_frame);

        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode._PublishShapes",
                              "The static environment once subtracted the reconstructed shapes (obtained from the extension of both models to supervoxels) contains "
                              << this->_static_env_ext_d_and_c_in_current_frame->points.size()
                              << " points.");

        sensor_msgs::PointCloud2 static_env_ext_d_and_c_in_current_frame_ros;
        pcl::toROSMsg(*this->_static_env_ext_d_and_c_in_current_frame, static_env_ext_d_and_c_in_current_frame_ros);

        this->_static_environment_ext_d_and_c_pub.publish(static_env_ext_d_and_c_in_current_frame_ros);
    }
}

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBA PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void ShapeReconstructionNode::_EstimateSupervoxels()
{
    this->_sv_estimated = true;

    this->_supervoxelizer.reset(new pcl::SupervoxelClustering<SRPoint>(this->_voxel_resolution,this->_seed_resolution, true));
    this->_supervoxelizer->setColorImportance (this->_color_importance);
    this->_supervoxelizer->setSpatialImportance (this->_spatial_importance);
    this->_supervoxelizer->setNormalImportance (this->_normal_importance);

    this->_supervoxelizer->setInputCloud (this->_current_pc_down);

    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode._SegmentPointCloud()", "Estimating supervoxels");
    this->_supervoxelizer->extract (this->_supervoxel_clusters);
    ROS_INFO_STREAM_NAMED("ShapeReconstructionNode._SegmentPointCloud()", "Found " << this->_supervoxel_clusters.size () << " supervoxels.");

    if(this->_visualization_sv)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);

//        PointCloudT::Ptr voxel_centroid_cloud = this->_supervoxelizer->getVoxelCentroidCloud ();
//        viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

        PointCloudT2::Ptr colored_voxel_cloud = this->_supervoxelizer->getColoredVoxelCloud ();
        viewer->addPointCloud (colored_voxel_cloud, "colored voxels");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "colored voxels");

        PointNCloudT::Ptr sv_normal_cloud = this->_supervoxelizer->makeSupervoxelNormalCloud (this->_supervoxel_clusters);
        //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
        viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,1,0.05f, "supervoxel_normals");


        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
    }
}

bool ShapeReconstructionNode::generateMeshes(shape_reconstruction::generate_meshes::Request& request, shape_reconstruction::generate_meshes::Response& response)
{
    // First we generate a triangular mesh for each shape and save them in stl files (done by each ShapeReconstruction object independently)
    BOOST_FOREACH(shape_reconstructors_map_t::value_type rb_shape_it, this->_rb_shapes)
    {
        rb_shape_it.second->generateMesh();
    }

    // Then we generate a mesh of the static environment
    if(this->_detect_static_environment)
    {
        ROS_INFO_STREAM_NAMED("ShapeReconstructionNode.generateMeshes",
                              "Called the generation of 3D triangular mesh from the model of the static environment (based on extension of both models to supervoxels).");
        std::string shape_ext_d_and_c_file_prefix("static_env_ext_d_and_c");
        this->_GenerateMesh(this->_static_env_ext_d_and_c_in_current_frame, shape_ext_d_and_c_file_prefix);
    }
    ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh", "Finished the generation of 3D triangular mesh from the point cloud of the static environment.");

    ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh", "Finished the generation of 3D triangular meshes");

    return true;
}

void ShapeReconstructionNode::_GenerateMesh(const omip::SRPointCloud::Ptr& pc_source, std::string shape_file_prefix)
{
    if(pc_source->points.size())
    {
        // Normal estimation*
        this->_se_tree_for_normal_estimation_ptr->setInputCloud(pc_source);
        this->_se_normal_estimator_ptr->setInputCloud(pc_source);
        this->_se_normal_estimator_ptr->setSearchMethod (this->_se_tree_for_normal_estimation_ptr);
        this->_se_normal_estimator_ptr->compute(*this->_se_estimated_normals_pc_ptr);

        // Concatenate the XYZ and normal fields*
        pcl::concatenateFields (*pc_source, *this->_se_estimated_normals_pc_ptr, *this->_se_position_color_and_normals_pc_ptr);
        this->_se_tree_for_triangulation_ptr->setInputCloud (this->_se_position_color_and_normals_pc_ptr);

        // Get result
        this->_se_greedy_projection_triangulator_ptr->setInputCloud(this->_se_position_color_and_normals_pc_ptr);
        this->_se_greedy_projection_triangulator_ptr->setSearchMethod(this->_se_tree_for_triangulation_ptr);
        this->_se_greedy_projection_triangulator_ptr->reconstruct(*this->_se_triangulated_mesh_ptr);

        // Additional vertex information
        //std::vector<int> parts = gp3.getPartIDs();
        //std::vector<int> states = gp3.getPointStates();

        std::string mesh_path = ros::package::getPath("shape_reconstruction") + std::string("/meshes/");
        std::stringstream se_name_ss;
        se_name_ss << shape_file_prefix << ".stl";

        std::string se_mesh_full_file_name = mesh_path + se_name_ss.str();

        pcl::io::mesh2vtk(*this->_se_triangulated_mesh_ptr, this->_se_polygon_data_ptr);

        this->_se_polygon_writer_ptr->SetInput (this->_se_polygon_data_ptr);
        this->_se_polygon_writer_ptr->SetFileName (se_mesh_full_file_name.c_str ());
        this->_se_polygon_writer_ptr->Write ();

        ROS_INFO_STREAM_NAMED("ShapeReconstruction.generateMesh", "Resulting triangular mesh written to " << se_mesh_full_file_name);
    }else{
        ROS_WARN_STREAM_NAMED("ShapeReconstruction.generateMesh", "Impossible to generate a triangular mesh for this model, it doesn't contain any points!");
    }
}

void ShapeReconstructionNode::_InitResultRosbag() {
    if (!_record_result_bag)
        return;

    if (_result_bag_open)
        return;

    using namespace boost::filesystem;

    // prepare path
    path result_bag_out(_result_bag_path);
    if (!boost::filesystem::exists(result_bag_out)) {
        if (!boost::filesystem::create_directories(result_bag_out)) {
            ROS_ERROR_NAMED("ShapeReconstruction.InitResultRosbag", "Output directory for result bag does not exist and cannot be created!");
            return;
        }
    }

    // get input rosbag name
    std::string input_bag_name;
    if (!_node_handle.getParam("/feature_tracker/bag_file", input_bag_name)){
        ROS_ERROR_NAMED("ShapeReconstruction.InitResultRosbag", "Cannot infer name of input bag file from feature tracker");
        return;
    }
    path input_bag_path(input_bag_name);

    // construct output rosbag name
    std::stringstream result_bag_name;
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    result_bag_name << input_bag_path.stem().string() << "_";
    result_bag_name << (now->tm_mon + 1) << "-" << now->tm_mday << "_" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec;

    result_bag_out /= path(result_bag_name.str() + ".bag");

    ROS_INFO_STREAM_NAMED("ShapeReconstruction.InitResultRosbag", "Opening result rosbag " << result_bag_out.string());
    this->_result_bag.open(result_bag_out.string(), rosbag::bagmode::Write);

    this->_result_bag_open=true;

    // construct full video rosbag
    // construct output rosbag name
    std::stringstream video_bag_name;

    video_bag_name << input_bag_path.stem().string() << "_";
    video_bag_name << (now->tm_mon + 1) << "-" << now->tm_mday << "_" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec;
    video_bag_name << "_VIDEO.bag";

    path video_bag_out(_result_bag_path);
    video_bag_out /= path(video_bag_name.str());

    this->_video_bag.open(video_bag_out.string(), rosbag::bagmode::Write);

    std::string cfg_src_path = ros::package::getPath("shape_reconstruction") + std::string("/cfg/shape_reconstruction_cfg.yaml");
    // copy shape reconstruction cfg
    path cfg_dest_path = _result_bag_path;
    cfg_dest_path /= result_bag_name.str() + ".yaml";

    std::ifstream ifs(cfg_src_path.c_str());
    std::ofstream ofs(cfg_dest_path.string().c_str());
    ofs << ifs.rdbuf();

}

void ShapeReconstructionNode::_AdvanceFeatureTracker() {
//    ros::Duration d(0.09);
//    d.sleep(); // sleep otherwise ROS might lose this message

    ROS_DEBUG_STREAM_NAMED("ShapeReconstruction._AdvanceFeatureTracker", "Advancing Feature Tracker");
    this->_advance_feature_tracker_pub.publish(std_msgs::Bool());
}

void ShapeReconstructionNode::TrackerQuitCallback(const std_msgs::EmptyConstPtr &empty)
{
    ROS_INFO_STREAM_NAMED("ShapeReconstruction.TrackerQuitCallback", "Shape Reconstruction quit stopping!");
    _active = false;
}

// Main program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ShapeReconstructionNode");
    ShapeReconstructionNode sr_node;

    ros::Rate r(10); // 10 hz
    while (ros::ok() && sr_node.isActive())
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_ERROR("Shutting down ShapeReconstructionNode" );

    return (0);
}
