/*
 * ShapeReconstruction.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014) and the extension to segment, reconstruct and track shapes introduced in our paper
 * "An Integrated Approach to Visual Perception of Articulated Objects" (Martín-Martín, Höfer and Brock, 2016)
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

@inproceedings{martinmartin_hoefer_iros_2016,
Title = {An Integrated Approach to Visual Perception of Articulated Objects},
Author = {Roberto {Mart\'{i}n-Mart\'{i}n} and Sebastian H\"ofer and Oliver Brock},
Booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation},
Pages = {5091 - 5097},
Year = {2016},
Doi = {10.1109/ICRA.2016.7487714},
Location = {Stockholm, Sweden},
Month = {05},
Note = {http://www.redaktion.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martin_hoefer_15_iros_sr_opt.pdf},
Url = {http://www.redaktion.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martin_hoefer_15_iros_sr_opt.pdf},
Url2 = {http://ieeexplore.ieee.org/xpl/articleDetails.jsp?tp=&arnumber=7487714},
Projectname = {Interactive Perception}
}
 * If you have questions or suggestions, contact us:
 * roberto.martinmartin@tu-berlin.de
 *
 * Enjoy!
 */

#ifndef SHAPE_RECONSTRUCTION_H_
#define SHAPE_RECONSTRUCTION_H_

#include <omip_common/OMIPTypeDefs.h>

//ROS and OpenCV
#include <opencv2/core/core.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <omip_common/OMIPUtils.h>

#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/supervoxel_clustering.h>

#include <boost/circular_buffer.hpp>

#include "shape_reconstruction/SRUtils.h"

#include <opencv2/highgui/highgui.hpp>

#include <rosbag/bag.h>

#include <omip_msgs/ShapeModel.h>
#include <omip_msgs/ShapeModels.h>

#include <unordered_set>

namespace omip
{

struct FrameForSegmentation
{
    ros::Time                           _time;
    SRPointCloud::Ptr                   _pc;
    SRPointCloud::Ptr                   _pc_moving_pts;
    SRPointCloud::Ptr                   _pc_moving_pts_transf;
    SRPointCloud::Ptr                   _pc_without_nans;
    std::vector<int>                    _not_nan_indices;
    sensor_msgs::Image                  _rgb;
    cv_bridge::CvImagePtr               _dm;
    cv::Mat                             _dm_filled;
    geometry_msgs::TwistWithCovariance  _transformation;

    FrameForSegmentation()
    {
        this->_time.fromSec(0.0);
        this->_pc = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_moving_pts = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_moving_pts_transf = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_without_nans = SRPointCloud::Ptr(new SRPointCloud);
        this->_rgb = sensor_msgs::Image();
        this->_dm = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
        this->_dm->encoding = std::string("32FC1");
        this->_dm->header.frame_id = "camera_rgb_optical_frame";
        this->_dm->image = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
        this->_dm_filled = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
        this->_transformation = geometry_msgs::TwistWithCovariance();
    }

    void reset() {
        this->_time.fromSec(0.0);
        this->_pc = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_moving_pts = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_moving_pts_transf = SRPointCloud::Ptr(new SRPointCloud);
        this->_pc_without_nans = SRPointCloud::Ptr(new SRPointCloud);
        this->_rgb = sensor_msgs::Image();
        this->_dm = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
        this->_dm->encoding = std::string("32FC1");
        this->_dm->header.frame_id = "camera_rgb_optical_frame";
        this->_dm->image = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
        this->_dm_filled = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_32FC1);
        this->_transformation = geometry_msgs::TwistWithCovariance();
    }

    // this returns a deep copy
    FrameForSegmentation clone() const {
        FrameForSegmentation new_ffs;
        new_ffs._time = this->_time;
        new_ffs._pc.reset(new SRPointCloud(*this->_pc));
        new_ffs._pc_moving_pts.reset(new SRPointCloud(*this->_pc_moving_pts));
        new_ffs._pc_moving_pts_transf.reset(new SRPointCloud(*this->_pc_moving_pts_transf));
        new_ffs._pc_without_nans.reset(new SRPointCloud(*this->_pc_without_nans));
        new_ffs._not_nan_indices = _not_nan_indices;
        new_ffs._rgb = _rgb;
        * (new_ffs._dm) = *(_dm);
        new_ffs._dm_filled = _dm_filled.clone();
        new_ffs._transformation = _transformation;
        return new_ffs;
    }
};


class ShapeReconstruction;
typedef boost::shared_ptr<ShapeReconstruction> ShapeReconstructionPtr;

class ShapeReconstruction
{    
public:
    ShapeReconstruction();

    virtual void setInitialFullRGBDPCAndRBT(const SRPointCloud::Ptr &initial_pc_msg,
                                            const geometry_msgs::TwistWithCovariance &rb_transformation_initial);

    virtual void initialize();

    ShapeReconstruction(const ShapeReconstruction &sr);

    virtual ~ShapeReconstruction();

    virtual void setCameraInfo(const sensor_msgs::CameraInfo& camera_info);

    virtual void setFullRGBDPCandRBT(const SRPointCloud::Ptr &pc_msg,
                                     const geometry_msgs::TwistWithCovariance &rb_transformation);

    virtual void generateMesh();

    virtual void getShapeModel(omip_msgs::ShapeModelsPtr shapes);

    virtual void PublishMovedModelAndSegment(const ros::Time current_time,
                                             const geometry_msgs::TwistWithCovariance &rb_transformation,
                                             rosbag::Bag& bag,
                                             bool bagOpen);

    virtual void RemoveInconsistentPoints(const SRPointCloud::Ptr &pc_msg,
                                          const geometry_msgs::TwistWithCovariance &rb_transformation);

    omip::RB_id_t getId() const
    {
        return this->_rb_id;
    }

    SRPointCloud::Ptr getCurrentPointCloud() const
    {
        return this->_current_ffs._pc;
    }

    SRPointCloud::Ptr getRigidBodyShapeExtDandC() const
    {
        return this->_rb_shape;
    }

    SRPointCloud::Ptr getMovedRigidBodyShapeExtDandC() const
    {
        return this->_rb_shape_in_current_frame;
    }

    virtual void setDetectStaticEnvironment(bool b)
    {
        _detect_static_environment = b;
    }

    virtual void setAccumulateChangeCandidates(bool b)
    {
        _accumulate_change_candidates = b;
    }

    virtual void setMinDepthChange(double b)
    {
        _min_depth_change = b;
    }

    virtual void setRemoveInconsistentPoints(double b)
    {
        _remove_inconsistent_points = b;
    }

    virtual void setMinColorChange(int min_color_change)
    {
        _min_color_change = min_color_change;
    }

    virtual void setKNNMinRadius(double knn_min_radius)
    {
        _knn_min_radius = knn_min_radius;
    }

    virtual void setSVMinNumberModelPixels(int min_number_model_pixels_in_sv)
    {
        _min_number_model_pixels_in_sv = min_number_model_pixels_in_sv;
    }

    virtual void setSuperVoxelizerPtr(boost::shared_ptr<pcl::SupervoxelClustering<SRPoint> >&  supervoxelizer)
    {
        this->_supervoxelizer_ptr_ptr = &supervoxelizer;
    }

    virtual void setSuperVoxelClustersPtr(std::map <uint32_t, pcl::Supervoxel<SRPoint>::Ptr >&  supervoxel_clusters)
    {
        this->_supervoxel_clusters_ptr = &supervoxel_clusters;
    }

    virtual void setExtendToNeighborSV(bool extend_to_neighbor_sv)
    {
        this->_extend_to_neighbor_sv = extend_to_neighbor_sv;
    }

    virtual void setSimilarityInH(double similarity_in_h)
    {
        this->_similarity_in_h = similarity_in_h;
    }

    virtual void setSimilarityInNormal(double similarity_in_normal)
    {
        this->_similarity_in_normal = similarity_in_normal;
    }

    virtual void setSupportingFeatures(pcl::PointCloud<pcl::PointXYZL>::Ptr supporting_features)
    {
        this->_supporting_features = supporting_features;
    }

    virtual void useClusteredFeatures(bool use_clustered_features)
    {
        this->_use_clustered_features = use_clustered_features;
    }

    virtual void setDepthFilling(bool depth_filling)
    {
        this->_depth_filling = depth_filling;
    }

    virtual void setColorChangesErodeSize(double color_changes_erode_size)
    {
        this->_color_changes_erode_size = color_changes_erode_size;
    }

    virtual void setToInitial(bool to_initial)
    {
        this->_to_initial = to_initial;
    }

    virtual void setRecordVideos(bool record_videos)
    {
        this->_record_videos = record_videos;
    }

    virtual void setLiveStream(bool live_stream)
    {
        this->_live_stream = live_stream;
    }

    virtual void setRBId(omip::RB_id_t rb_id)
    {
        this->_rb_id = rb_id;
    }

    virtual void setApproxVoxelGridLeafSize(double leaf_size)
    {
        this->_leaf_size = leaf_size;
    }

    virtual void setRadiusOutRemovalSearch(double ror_radius_search)
    {
        this->_ror_radius_search = ror_radius_search;
    }

    virtual void setRadiusOutRemovalMinNeighbors(int ror_min_neighbors)
    {
        this->_ror_min_neighbors = ror_min_neighbors;
    }

    ShapeReconstructionPtr clone() const
    {
        return (ShapeReconstructionPtr(doClone()));
    }

    void setEstimateSV(bool estimate_sv)
    {
        _estimate_supervoxels = estimate_sv;
    }

protected:

    ///Clone function
    virtual ShapeReconstruction* doClone() const
    {
        return (new ShapeReconstruction(*this));
    }

    virtual void _DetectImageLocationsWhereDepthChanges();

    virtual void _DetectImageLocationsWhereColorChanges();

    virtual void _TestMotionCoherencyOfPointsInImageLocationsWhereDepthChanged();

    virtual void _TestMotionCoherencyOfPointsInImageLocationsWhereColorChanged();

    virtual void _FindCandidatesInPreviousPC(FrameForSegmentation& _previous_ffs,
                                             const FrameForSegmentation& _current_ffs,
                                             pcl::PointIndices::Ptr& _moving_pts_of_previous_idx_depth,
                                             Eigen::Matrix4d& _current_to_previous_HTransform,
                                             std::vector<std::vector<int > >& _current_k_indices_depth,
                                             std::vector<std::vector<float > >& _current_sqrt_dist_depth);

    virtual void _FindCandidatesInCurrentPC(const FrameForSegmentation& _previous_ffs,
                                            FrameForSegmentation& _current_ffs,
                                            pcl::PointIndices::Ptr& _moving_pts_of_current_idx_depth,
                                            Eigen::Matrix4d& _previous_to_current_HTransform,
                                            std::vector<std::vector<int > >& _previous_k_indices_depth,
                                            std::vector<std::vector<float > >& _previous_sqrt_dist_depth);

    virtual void _MergeValidPointsIntoModels();

    virtual void _MergeValidPointsIntoModels(const std::string& logger_name,
                                                          omip::SRPointCloud::Ptr& points_of_current_in_origin,
                                                          omip::SRPointCloud::Ptr& points_of_previous_in_origin,
                                                          omip::SRPointCloud::Ptr& rb_model,
                                                          const std::vector<std::vector<int > >& current_k_indices,
                                                          const std::vector<std::vector<int > >& previous_k_indices);


    virtual void _RemoveInconsistentPointsFromModelsAndExtendToRegions();

    virtual void _RemoveInconsistentPointsFromRBModel(const std::string method,
                                                      const Eigen::Matrix4d& HTransform,
                                                      SRPointCloud::Ptr& rb_shape_depth,
                                                      cv::Mat& current_dm,
                                                      SRPointCloud::Ptr current_pc,
                                                      SRPointCloud::Ptr& rb_segment_depth);

    virtual void _FindInconsistentPoints(const omip::SRPointCloud::Ptr& pc_source,
                                     const cv::Mat & dm_true,
                                     pcl::PointIndicesPtr& indices_to_remove,
                                     pcl::PointIndicesPtr& indices_matching_in_true,
                                     pcl::PointIndicesPtr& indices_matching_in_dm,
                                     const double min_depth_error=0.03);

    virtual void _ExtendPointsToRegions();

    virtual void _ExtendToNeighborSV(std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                                                  std::vector<uint32_t>& labels_extension);

    virtual void _FilterModel();

    virtual void _EstimateTransformations();

    virtual void _GenerateMesh(const omip::SRPointCloud::Ptr& pc_source,
                               std::string shape_file_prefix);

    void growSVRecursively(uint32_t label_sv_seed,
                           std::vector<uint32_t>& labels_extension,
                           std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,
                           std::vector<uint32_t>& sv_containing_supporting_features,
                           int& distance_to_feature_sv,
                           std::map<uint32_t, std::unordered_set<int> >& labels_of_segments_to_extend);


    omip::RB_id_t                                                       _rb_id;

    SRPointCloud::Ptr                                                   _rb_shape;
    SRPointCloud::Ptr                                                   _rb_shape_in_current_frame;
    SRPointCloud::Ptr                                                   _rb_segment;

    bool                                                                _to_initial;
    bool                                                                _depth_filling;
    bool                                                                _detect_static_environment;
    bool                                                                _accumulate_change_candidates;
    bool                                                                _remove_inconsistent_points;
    bool                                                                _record_videos;
    bool                                                                _extend_to_neighbor_sv;
    bool                                                                _use_clustered_features;
    bool                                                                _live_stream;
    bool                                                                _estimate_supervoxels;

    double                                                              _min_depth_change;
    int                                                                 _min_color_change;
    double                                                              _knn_min_radius;
    int                                                                 _min_number_model_pixels_in_sv;
    double                                                              _color_changes_erode_size;
    double                                                              _timestamp_color;
    double                                                              _similarity_in_h;
    double                                                              _similarity_in_normal;
    int                                                                 _t;

    FrameForSegmentation                                                _initial_ffs;
    FrameForSegmentation                                                _previous_ffs;
    FrameForSegmentation                                                _current_ffs;    

    pcl::RangeImagePlanar::Ptr                                          _rip_temp;

    cv::Mat                             _acc_candidates_of_current;
    cv::Mat                             _difference_in_depth;
    cv::Mat                             _candidates_of_current;
    cv::Mat                             _candidates_of_previous;
    cv::Mat                             _candidates_of_current_8u;
    cv::Mat                             _candidates_of_previous_8u;
    cv::Mat                             _previous_depth_mask, _current_depth_mask;

    cv::Mat                             _difference_in_depth_moved;

    pcl::PointIndices::Ptr              _moving_pts_of_current_idx_depth;
    pcl::PointIndices::Ptr              _moving_pts_of_previous_idx_depth;

    pcl::PointIndices::Ptr              _moving_pts_of_current_idx_color;
    pcl::PointIndices::Ptr              _moving_pts_of_previous_idx_color;

    pcl::PointIndices::Ptr              _occluded_pts_of_current_idx;
    pcl::PointIndices::Ptr              _out_of_view_pts_of_current_idx;

    Eigen::Twistd                       _previous_twist;
    Eigen::Matrix4d                     _previous_HTransform;
    Eigen::Matrix4d                     _previous_HTransform_inv;

    Eigen::Twistd                       _current_twist;
    Eigen::Matrix4d                     _current_HTransform;
    Eigen::Matrix4d                     _current_HTransform_inv;

    Eigen::Matrix4d                     _current_to_previous_HTransform;
    Eigen::Matrix4d                     _previous_to_current_HTransform;

    std::vector<std::vector<int > >     _current_k_indices;
    std::vector<std::vector<int > >     _previous_k_indices;
    std::vector<std::vector<int > >     _previous_k_indices_previous;
    std::vector<std::vector<float > >   _sqrt_dist;

    // Create the filtering object
    pcl::ExtractIndices<SRPoint>        _extractor;

    pcl::search::Search<SRPoint>::Ptr   _knn;

    ros::Publisher                      _rb_shape_pub;
    ros::Publisher                      _occlusions_pub;
    ros::Publisher                      _occlusions_transformed_pub;
    ros::Publisher                      _rb_segment_pub;

    ros::NodeHandle                     _node_handle;

    SRPointCloud::Ptr                   _candidates;
    SRPointCloud::Ptr                   _candidates_in_current;

    // Filter to remove outliers. It should be able to remove single supervoxels
    pcl::RadiusOutlierRemoval<SRPoint>                                  _radius_outlier_removal;
    int                                                                 _ror_min_neighbors;
    double                                                              _ror_radius_search;

    // Filter to maintain the shape model of aprox. constant size / num of points
    pcl::ApproximateVoxelGrid<SRPoint>                                  _approximate_voxel_grid_filter;
    double                                                              _leaf_size;

    // Generation of triangulare mesh model////////////////////////////////////////////////////////////////////
    pcl::NormalEstimationOMP<omip::SRPoint, pcl::Normal>::Ptr           _rb_normal_estimator_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr                                   _rb_estimated_normals_pc_ptr;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr                        _rb_position_color_and_normals_pc_ptr;
    pcl::search::KdTree<omip::SRPoint>::Ptr                             _rb_tree_for_normal_estimation_ptr;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr                    _rb_tree_for_triangulation_ptr;
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>::Ptr     _rb_greedy_projection_triangulator_ptr;
    pcl::PolygonMesh::Ptr                                               _rb_triangulated_mesh_ptr;
    vtkSmartPointer<vtkPolyData>                                        _rb_polygon_data_ptr;
    vtkSmartPointer<vtkSTLWriter>                                       _rb_polygon_writer_ptr;
    // Generation of triangulare mesh model////////////////////////////////////////////////////////////////////

    rosbag::Bag                                                         _videos;

    // SuperVoxels////////////////////////////////////////////////////////////////////
    boost::shared_ptr<pcl::SupervoxelClustering<SRPoint> > *            _supervoxelizer_ptr_ptr;
    std::map <uint32_t, pcl::Supervoxel<SRPoint>::Ptr >*                _supervoxel_clusters_ptr;
    // SuperVoxels////////////////////////////////////////////////////////////////////

    sensor_msgs::CameraInfo _ci;

    pcl::PointCloud<pcl::PointXYZL>::Ptr                        _supporting_features;
};
}

#endif /* FEATURE_TRACKER_H_ */
