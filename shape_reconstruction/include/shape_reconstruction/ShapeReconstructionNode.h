/*
 * ShapeReconstructionNode.h
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

#ifndef SHAPE_RECONSTRUCTION_NODE_H_
#define SHAPE_RECONSTRUCTION_NODE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>

#include <rosbag/bag.h>
#include <tf/tf.h>

#include "shape_reconstruction/ShapeReconstructionNode.h"
#include "shape_reconstruction/ShapeReconstruction.h"

#include <boost/thread.hpp>

#include <omip_common/OMIPTypeDefs.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

#include <shape_reconstruction/generate_meshes.h>

#include <pcl_conversions/pcl_conversions.h>

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
#include <pcl/filters/extract_indices.h>
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

#include <pcl/segmentation/supervoxel_clustering.h>

#include <boost/circular_buffer.hpp>

#include "shape_reconstruction/SRUtils.h"

#include <omip_msgs/ShapeModels.h>

namespace omip
{

class ShapeReconstructionNode
{
    // Policies to synchorize point clouds, and the RBP from the RBT
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, omip::rbt_state_t> SRSyncPolicy;

    // Policies to synchorize point clouds, and the RBP and clustered features from the RBT
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, omip::rbt_state_t, rbt_measurement_ros_t> SRSyncPolicyWithClusteredFeatures;

    typedef std::map<omip::RB_id_t, omip::ShapeReconstructionPtr > shape_reconstructors_map_t;

public:

    /**
     * Constructor
     */
    ShapeReconstructionNode();

    /**
     * Destructor
     */
    virtual ~ShapeReconstructionNode();

    virtual void measurementCallbackWithClusteredFeatures(const sensor_msgs::PointCloud2ConstPtr &pc_msg,
                                                                           const boost::shared_ptr<omip::rbt_state_t const> &poses_and_vels,
                                                                           const sensor_msgs::PointCloud2ConstPtr &features_pc);

    virtual void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci_msg);

    virtual void ReadRosBag();

    virtual bool generateMeshes(shape_reconstruction::generate_meshes::Request& request, shape_reconstruction::generate_meshes::Response& response);

    virtual void setSVVoxelResolution(double voxel_resolution) {
        _voxel_resolution = voxel_resolution;
    }

    virtual void setSVSeedResolution(double seed_resolution) {
        _seed_resolution = seed_resolution;
    }

    virtual void setSVColorImportance(double color_importance) {
        _color_importance = color_importance;
    }

    virtual void setSVSpatialImportance(double spatial_importance) {
        _spatial_importance = spatial_importance;
    }

    virtual void setSVNormalImportance(double normal_importance) {
        _normal_importance = normal_importance;
    }

    virtual void setSVVisualization(bool visualization_sv) {
        _visualization_sv = visualization_sv;
    }

    virtual bool isActive() const {
        return _active;
    }

    virtual void TrackerQuitCallback(const std_msgs::EmptyConstPtr &empty);

protected:

    virtual void _CollectAndPublishShapeModels();

    virtual void _GenerateMesh(const omip::SRPointCloud::Ptr& pc_source, std::string shape_file_prefix);

    virtual void _InitResultRosbag();

    virtual void _AdvanceFeatureTracker();

    virtual void _EstimateSupervoxels();

    omip::ShapeReconstructionPtr _createNewShapeReconstruction();

    bool                                                                        _active;

    int                                                                         _frame_ctr;
    std::map<omip::RB_id_t,omip::ShapeReconstructionPtr >                       _rb_shapes;
    // maximum number of rigid bodies to be tracked (default: 100)
    int                                                                         _max_rb;

    double                                                                      _processing_interval;

    bool                                                                        _detect_static_environment;
    bool                                                                        _publish_shapes_in_each_frame;
    bool                                                                        _refine_shapes_in_each_frame;
    ros::Time                                                                   _previous_measurement_time;
    ros::Time                                                                   _current_measurement_time;
    ros::NodeHandle                                                             _node_handle;
    ros::Publisher                                                              _rbshapes_combined_publisher;

    ros::Publisher                                                              _static_environment_ext_d_and_c_pub;
    ros::Publisher                                                              _advance_feature_tracker_pub;
    ros::Subscriber                                                             _node_quit_subscriber;
    message_filters::Subscriber<sensor_msgs::PointCloud2>                       _rgbd_pc_subscriber;
    message_filters::Subscriber<omip::rbt_state_t>                              _poses_and_vels_subscriber;
    message_filters::Subscriber<sensor_msgs::PointCloud2>                           _clustered_feats_subscriber;
    message_filters::Synchronizer<SRSyncPolicy>*                                _synchronizer;
    message_filters::Synchronizer<SRSyncPolicyWithClusteredFeatures>*           _synchronizer_with_clustered_feats;
    ros::ServiceServer                                                          _mesh_generator_srv;

    SRPointCloud::Ptr                                                           _current_pc;
    SRPointCloud::Ptr _current_pc_down;
    SRPointCloud::Ptr                                                           _current_pc_without_nans;

    std::vector<int> _not_nan_indices;

    SRPointCloud::Ptr                                                           _static_env_ext_d_and_c_in_current_frame;
    pcl::search::Search<omip::SRPoint>::Ptr                                     _set_knn_ptr;
    // Triangular mesh generation
    pcl::NormalEstimationOMP<omip::SRPoint, pcl::Normal>::Ptr                   _se_normal_estimator_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr                                           _se_estimated_normals_pc_ptr;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr                                _se_position_color_and_normals_pc_ptr;
    pcl::search::KdTree<omip::SRPoint>::Ptr                                     _se_tree_for_normal_estimation_ptr;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr                            _se_tree_for_triangulation_ptr;
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>::Ptr             _se_greedy_projection_triangulator_ptr;
    pcl::PolygonMesh::Ptr                                                       _se_triangulated_mesh_ptr;
    vtkSmartPointer<vtkPolyData>                                                _se_polygon_data_ptr;
    vtkSmartPointer<vtkSTLWriter>                                               _se_polygon_writer_ptr;

    bool                                                                        _record_result_bag;
    std::string                                                                 _result_bag_path;
    rosbag::Bag                                                                 _result_bag;
    rosbag::Bag                                                                 _video_bag;
    bool                                                                        _result_bag_open;

    bool                                                                        _manually_advance_after_processing;

    // Supervoxels
    double                                                               _voxel_resolution;
    double                                                               _seed_resolution;
    double                                                               _color_importance;
    double                                                               _spatial_importance;
    double                                                               _normal_importance;
    boost::shared_ptr<pcl::SupervoxelClustering<SRPoint> >              _supervoxelizer;
    std::map <uint32_t, pcl::Supervoxel<SRPoint>::Ptr >                 _supervoxel_clusters;

    bool                                                                _visualization_sv;
    bool                                                                _sv_estimated;
    bool                                                                _estimate_sv;

    ros::Publisher                                                              _shape_models_ext_d_and_c_pub;

    ros::Subscriber                                                             _ci_sub;
    sensor_msgs::CameraInfo                                                     _ci;

    pcl::PointCloud<pcl::PointXYZL>::Ptr                                        _clustered_feats;

    pcl::ExtractIndices<pcl::PointXYZL>                                                _extractor_cf;

    template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
        if (!(this->_node_handle.getParam(param_name, param_container)))
        {
            ROS_ERROR_NAMED("ShapeReconstructionNode.getROSParameter", "The parameter %s can not be found.", param_name.c_str());
            throw(std::string("[ShapeReconstructionNode.getROSParameter] The parameter can not be found. Parameter name: ") + param_name);
            return false;
        }
        else
            return true;
    }
};
}

#endif /* FEATURE_TRACKER_NODE_H_ */

