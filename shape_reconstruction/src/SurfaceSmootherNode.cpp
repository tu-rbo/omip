#include "shape_reconstruction/SurfaceSmootherNode.h"

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

#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <vcg/complex/complex.h> //class UpdateCurvature
#include <vcg/complex/all_types.h>     //class UpdateNormals
#include <vcg/complex/algorithms/clean.h> //class UpdateCurvature

#include <vcg/complex/algorithms/update/bounding.h> //class UpdateCurvature
#include <vcg/complex/algorithms/update/normal.h> //class UpdateCurvature
#include <vcg/complex/algorithms/update/topology.h> //class UpdateCurvature

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/normal_refinement.h>

#include <pcl/surface/simplification_remove_unused_vertices.h>

using namespace omip;

SurfaceSmootherNode::SurfaceSmootherNode()
{
    std::string topic_name;
    this->_node_handle.getParam("/surface_smoother/topic_name", topic_name);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","topic_name: " << topic_name);
    this->_pc_subscriber = this->_node_handle.subscribe(topic_name, 1,
                                                        &SurfaceSmootherNode::measurementCallback, this);
    this->_current_pc.reset(new SRPointCloud());
    this->_current_pc_copy.reset(new SRPointCloud());
}

SurfaceSmootherNode::~SurfaceSmootherNode()
{

}

void SurfaceSmootherNode::measurementCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(*pc_msg, *this->_current_pc);

    pcl::fromROSMsg(*pc_msg, *this->_current_pc_copy);


    this->generateMesh();
}

void SurfaceSmootherNode::ReadRosBag()
{


}

void SurfaceSmootherNode::generateMesh()
{    
    bool preprocessing_remove_nans;
    this->_node_handle.getParam("/surface_smoother/preprocessing_remove_nans", preprocessing_remove_nans);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","preprocessing_remove_nans: " << preprocessing_remove_nans);
    if(preprocessing_remove_nans)
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin NaNs removal");

        // Clean the input pcl point cloud of nans
        std::vector<int> not_nan_indices;
        pcl::removeNaNFromPointCloud<pcl::PointXYZRGB>(*this->_current_pc,*this->_current_pc, not_nan_indices);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End NaNs removal");
    }

    bool preprocessing_rar;
    this->_node_handle.getParam("/surface_smoother/preprocessing_rar", preprocessing_rar);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","preprocessing_rar: " << preprocessing_rar);
    if(preprocessing_rar)
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin radius outlier removal");

        int rar_num_neighbors;
        this->_node_handle.getParam("/surface_smoother/rar_num_neighbors", rar_num_neighbors);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","rar_num_neighbors: " << rar_num_neighbors);

        float rar_search_radius;
        this->_node_handle.getParam("/surface_smoother/rar_search_radius", rar_search_radius);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","rar_search_radius: " << rar_search_radius);

        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rar;

        rar.setMinNeighborsInRadius(rar_num_neighbors);\
        rar.setRadiusSearch(rar_search_radius);
        rar.setInputCloud(this->_current_pc);

        rar.filter(*this->_current_pc);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End radius outlier removal");
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combined_rgbd_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());

    int pre_meshing_and_ne;
    this->_node_handle.getParam("/surface_smoother/pre_meshing", pre_meshing_and_ne);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","pre_meshing: " << pre_meshing_and_ne);
    switch(pre_meshing_and_ne)
    {    
    case 0:
    default:
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin normals estimation");

        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setNumberOfThreads(8);
        ne.setInputCloud(this->_current_pc);


        int ne_num_neighbors;
        this->_node_handle.getParam("/surface_smoother/ne_num_neighbors", ne_num_neighbors);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","ne_num_neighbors: " << ne_num_neighbors);

        double ne_min_radius;
        this->_node_handle.getParam("/surface_smoother/ne_min_radius", ne_min_radius);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","ne_min_radius: " << ne_min_radius);

        if(ne_num_neighbors != -1 && ne_min_radius != -1)
        {
            ROS_WARN_STREAM_NAMED("SurfaceSmootherNode.generateMesh","Both ne_min_radius and ne_num_neighbors are not -1. Using ne_num_neighbors." << ne_min_radius);
        }

        if(ne_num_neighbors != -1)
        {
            ne.setKSearch(ne_num_neighbors);
        }else{
            ne.setRadiusSearch(ne_num_neighbors);
        }

        ne.useSensorOriginAsViewPoint();
        ne.compute(*cloud_normals);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End normals estimation");

        concatenateFields(*this->_current_pc, *cloud_normals, *combined_rgbd_normals);
        break;
    }
    case 1:
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin MLS and normals estimation");

        float mls_search_radius;
        this->_node_handle.getParam("/surface_smoother/mls_search_radius", mls_search_radius);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mls_search_radius: " << mls_search_radius);

        // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

        mls.setComputeNormals (true);

        // Set parameters
        mls.setInputCloud (this->_current_pc);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (mls_tree);
        mls.setSearchRadius (mls_search_radius);

        // Reconstruct
        mls.process (*combined_rgbd_normals);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End MLS and normals estimation");
        break;
    }
    }

    bool align_normals;
    this->_node_handle.getParam("/surface_smoother/align_normals", align_normals);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","align_normals: " << align_normals);
    if(align_normals)
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin normals aligment");

        bool use_centroid_for_normal_alignment;
        this->_node_handle.getParam("/surface_smoother/use_centroid_for_normal_alignment", use_centroid_for_normal_alignment);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","use_centroid_for_normal_alignment: " << use_centroid_for_normal_alignment);

        Eigen::Vector4f point_to_align;

        if(use_centroid_for_normal_alignment)
        {
            pcl::compute3DCentroid(*this->_current_pc, point_to_align);
        }else{
            point_to_align[0] = 0;
            point_to_align[1] = 0;
            point_to_align[2] = 0;
        }

        for(size_t i = 0; i < combined_rgbd_normals->size(); ++i)
        {
            pcl::flipNormalTowardsViewpoint<pcl::PointXYZRGBNormal>(combined_rgbd_normals->points[i], point_to_align[0], point_to_align[1], point_to_align[2],
                    combined_rgbd_normals->points[i].normal_x,
                    combined_rgbd_normals->points[i].normal_y,
                    combined_rgbd_normals->points[i].normal_z);
        }

        // For visualization we also align it in the pc cloud_normals
        for(size_t i = 0; i < combined_rgbd_normals->size(); ++i)
        {

            pcl::flipNormalTowardsViewpoint<pcl::PointXYZRGBNormal>(combined_rgbd_normals->points[i], point_to_align[0], point_to_align[1], point_to_align[2],
                    cloud_normals->points[i].normal_x,
                    cloud_normals->points[i].normal_y,
                    cloud_normals->points[i].normal_z);
        }

        if(use_centroid_for_normal_alignment)
        {
            for(size_t i = 0; i < cloud_normals->size(); ++i){
                cloud_normals->points[i].normal_x *= -1;
                cloud_normals->points[i].normal_y *= -1;
                cloud_normals->points[i].normal_z *= -1;
            }

            for(size_t i = 0; i < combined_rgbd_normals->size(); ++i){
                combined_rgbd_normals->points[i].normal_x *= -1;
                combined_rgbd_normals->points[i].normal_y *= -1;
                combined_rgbd_normals->points[i].normal_z *= -1;
            }
        }

        //        pcl::NormalRefinement<pcl::PointXYZRGBNormal> nr;
        //        nr.setMaxIterations(100);
        //        nr.setInputCloud(combined_rgbd_normals);
        //        nr.filter(*combined_rgbd_normals);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End normals aligment");
    }

    bool show_pc_and_normals;
    this->_node_handle.getParam("/surface_smoother/show_pc_and_normals", show_pc_and_normals);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","show_pc_and_normals: " << show_pc_and_normals);
    if(show_pc_and_normals)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer1->setBackgroundColor (0, 0, 0);
        viewer1->addPointCloud(this->_current_pc, "cloudy");
        viewer1->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(this->_current_pc, cloud_normals,100, 0.05,"normals");

        Eigen::Vector4f centroid2;
        pcl::compute3DCentroid(*this->_current_pc, centroid2);
        viewer1->addCoordinateSystem(0.2,centroid2[0],centroid2[1], centroid2[2]);

        viewer1->addCoordinateSystem (0.10);
        viewer1->initCameraParameters ();

        while (!viewer1->wasStopped ()){
            viewer1->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    pcl::PolygonMesh mesh;

    int meshing_method;
    this->_node_handle.getParam("/surface_smoother/meshing_method", meshing_method);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","meshing_method: " << meshing_method);
    switch(meshing_method)
    {
    case 1: //Poisson
    default:
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Poisson surface reconstruction");

        int poisson_depth;
        this->_node_handle.getParam("/surface_smoother/poisson_depth", poisson_depth);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","poisson_depth: " << poisson_depth);

        bool poisson_confidence;
        this->_node_handle.getParam("/surface_smoother/poisson_confidence", poisson_confidence);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","poisson_confidence: " << poisson_confidence);

        int poisson_samples_per_node;
        this->_node_handle.getParam("/surface_smoother/poisson_samples_per_node", poisson_samples_per_node);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","poisson_samples_per_node: " << poisson_samples_per_node);

        pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
        poisson.setDepth(poisson_depth);
        poisson.setOutputPolygons(false);
        poisson.setConfidence(poisson_confidence);
        poisson.setSamplesPerNode(poisson_samples_per_node);
        //poisson.setDegree();

        poisson.setInputCloud(combined_rgbd_normals);
        poisson.reconstruct(mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Poisson surface reconstruction");
        break;
    }
    case 2: //MarchingCubes rbf
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Marching Cubes RBF surface reconstruction (this will very likely crush with a SegFault!)");

        int mc_grid_resolution;
        this->_node_handle.getParam("/surface_smoother/mc_grid_resolution", mc_grid_resolution);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_grid_resolution: " << mc_grid_resolution);

        double mc_percentage_ext_grid;
        this->_node_handle.getParam("/surface_smoother/mc_percentage_ext_grid", mc_percentage_ext_grid);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_percentage_ext_grid: " << mc_percentage_ext_grid);

        double mc_rbf_offset_sfc_disp;
        this->_node_handle.getParam("/surface_smoother/mc_rbf_offset_sfc_disp", mc_rbf_offset_sfc_disp);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_rbf_offset_sfc_disp: " << mc_rbf_offset_sfc_disp);

        double mc_iso_level;
        this->_node_handle.getParam("/surface_smoother/mc_iso_level", mc_iso_level);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_iso_level: " << mc_iso_level);

        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree2->setInputCloud (combined_rgbd_normals);
        pcl::MarchingCubesRBF<pcl::PointXYZRGBNormal> mc;
        mc.setIsoLevel (mc_iso_level);
        mc.setGridResolution (mc_grid_resolution, mc_grid_resolution, mc_grid_resolution);
        mc.setPercentageExtendGrid (mc_percentage_ext_grid);
        mc.setOffSurfaceDisplacement (mc_rbf_offset_sfc_disp);
        mc.setInputCloud(combined_rgbd_normals);
        mc.setSearchMethod (tree2);
        mc.reconstruct (mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Marching Cubes RBF surface reconstruction");
        break;
    }
    case 3: //MarchingCubes hoppe
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Marching Cubes Hoppe surface reconstruction");

        int mc_grid_resolution;
        this->_node_handle.getParam("/surface_smoother/mc_grid_resolution", mc_grid_resolution);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_grid_resolution: " << mc_grid_resolution);

        double mc_percentage_ext_grid;
        this->_node_handle.getParam("/surface_smoother/mc_percentage_ext_grid", mc_percentage_ext_grid);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_percentage_ext_grid: " << mc_percentage_ext_grid);

        double mc_iso_level;
        this->_node_handle.getParam("/surface_smoother/mc_iso_level", mc_iso_level);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","mc_iso_level: " << mc_iso_level);

        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree2->setInputCloud (combined_rgbd_normals);
        pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> mc;
        mc.setIsoLevel (mc_iso_level);
        mc.setGridResolution (mc_grid_resolution, mc_grid_resolution, mc_grid_resolution);
        mc.setPercentageExtendGrid (mc_percentage_ext_grid);
        mc.setInputCloud(combined_rgbd_normals);
        mc.setSearchMethod (tree2);
        mc.reconstruct (mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Marching Cubes Hoppe surface reconstruction");
        break;
    }
    case 4: // Convex hull
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Convex hull surface reconstruction");
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr convex_hull(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::ConvexHull<pcl::PointXYZRGBNormal> cHull;
        cHull.setComputeAreaVolume(false);
        cHull.setInputCloud(combined_rgbd_normals);
        cHull.reconstruct (*convex_hull, mesh.polygons);
        pcl::toPCLPointCloud2(*convex_hull, mesh.cloud);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Convex hull surface reconstruction");
        break;
    }
    }

    bool orient_normals_in_mesh;
    this->_node_handle.getParam("/surface_smoother/orient_normals_in_mesh", orient_normals_in_mesh);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","orient_normals_in_mesh: " << orient_normals_in_mesh);
    if(orient_normals_in_mesh)
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin orienting normals in mesh");
        orientNormalsInMesh(mesh);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End orienting normals in mesh");
    }

    pcl::PolygonMesh processed_mesh;

    int post_meshing_method;
    this->_node_handle.getParam("/surface_smoother/post_meshing_method", post_meshing_method);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","post_meshing_method: " << post_meshing_method);
    switch(post_meshing_method)
    {
    case 0://NOTHING
    default:
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "No post-processing of the mesh");
        processed_mesh = mesh;
        break;
    }
    case 1: //EARCLIPPING
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Earclipping");

        pcl::EarClipping clipper;
        pcl::PolygonMesh::ConstPtr mesh_aux (new pcl::PolygonMesh(mesh));

        clipper.setInputMesh (mesh_aux);
        clipper.process (processed_mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Earclipping");
        break;
    }
    case 2: //MESHSMOOTHINGLAPLACIANVTK
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Smoothing Laplacian VTK");

        bool ms_boundary_smoothing;
        this->_node_handle.getParam("/surface_smoother/ms_boundary_smoothing", ms_boundary_smoothing);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_boundary_smoothing: " << ms_boundary_smoothing);

        int ms_num_iter;
        this->_node_handle.getParam("/surface_smoother/ms_num_iter", ms_num_iter);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_num_iter: " << ms_num_iter);

        bool ms_feat_edge_smoothing;
        this->_node_handle.getParam("/surface_smoother/ms_feat_edge_smoothing", ms_feat_edge_smoothing);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_feat_edge_smoothing: " << ms_feat_edge_smoothing);

        float ms_feat_angle;
        this->_node_handle.getParam("/surface_smoother/ms_feat_angle", ms_feat_angle);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_feat_angle: " << ms_feat_angle);

        float ms_lap_convergence;
        this->_node_handle.getParam("/surface_smoother/ms_lap_convergence", ms_lap_convergence);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_lap_convergence: " << ms_lap_convergence);

        float ms_lap_relaxation_factor;
        this->_node_handle.getParam("/surface_smoother/ms_lap_relaxation_factor", ms_lap_relaxation_factor);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_lap_relaxation_factor: " << ms_lap_relaxation_factor);

        float ms_edge_angle;
        this->_node_handle.getParam("/surface_smoother/ms_edge_angle", ms_edge_angle);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_edge_angle: " << ms_edge_angle);

        pcl::MeshSmoothingLaplacianVTK ms;
        pcl::PolygonMesh::ConstPtr mesh_aux (new pcl::PolygonMesh(mesh));

        ms.setBoundarySmoothing(ms_boundary_smoothing);
        ms.setNumIter(ms_num_iter);
        ms.setFeatureEdgeSmoothing(ms_feat_edge_smoothing);
        ms.setFeatureAngle(ms_feat_angle);
        ms.setConvergence(ms_lap_convergence);
        ms.setRelaxationFactor(ms_lap_relaxation_factor);
        ms.setEdgeAngle(ms_edge_angle);

        ms.setInputMesh (mesh_aux);
        ms.process (processed_mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Smoothing Laplacian VTK");
        break;
    }
    case 3: //MESHSMOOTHINGWINDOWEDSINCVTK
    {
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin Smoothing Windowed Sinc VTK");

        int ms_num_iter;
        this->_node_handle.getParam("/surface_smoother/ms_num_iter", ms_num_iter);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_num_iter: " << ms_num_iter);

        bool ms_boundary_smoothing;
        this->_node_handle.getParam("/surface_smoother/ms_boundary_smoothing", ms_boundary_smoothing);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_boundary_smoothing: " << ms_boundary_smoothing);

        bool ms_feat_edge_smoothing;
        this->_node_handle.getParam("/surface_smoother/ms_feat_edge_smoothing", ms_feat_edge_smoothing);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_feat_edge_smoothing: " << ms_feat_edge_smoothing);

        float ms_feat_angle;
        this->_node_handle.getParam("/surface_smoother/ms_feat_angle", ms_feat_angle);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_feat_angle: " << ms_feat_angle);

        float ms_edge_angle;
        this->_node_handle.getParam("/surface_smoother/ms_edge_angle", ms_edge_angle);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_edge_angle: " << ms_edge_angle);

        bool ms_win_norm_coord;
        this->_node_handle.getParam("/surface_smoother/ms_win_norm_coord", ms_win_norm_coord);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_win_norm_coord: " << ms_win_norm_coord);

        float ms_win_pass_band;
        this->_node_handle.getParam("/surface_smoother/ms_win_pass_band", ms_win_pass_band);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","ms_win_pass_band: " << ms_win_pass_band);

        pcl::MeshSmoothingWindowedSincVTK ms;
        pcl::PolygonMesh::ConstPtr mesh_aux (new pcl::PolygonMesh(mesh));

        ms.setBoundarySmoothing(ms_boundary_smoothing);
        ms.setNumIter(ms_num_iter);
        ms.setFeatureEdgeSmoothing(ms_feat_edge_smoothing);
        ms.setFeatureAngle(ms_feat_angle);
        ms.setEdgeAngle(ms_edge_angle);
        ms.setNormalizeCoordinates(ms_win_norm_coord);
        ms.setPassBand(ms_win_pass_band);

        ms.setInputMesh (mesh_aux);
        ms.process (processed_mesh);

        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End Smoothing Windowed Sinc VTK");
        break;
    }
    }

    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "Begin coloring the mesh");
    this->colorMesh(processed_mesh);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh", "End coloring the mesh");

    bool save_mesh;
    this->_node_handle.getParam("/surface_smoother/save_mesh", save_mesh);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","save_mesh: " << save_mesh);
    if(save_mesh)
    {
        std::string save_mesh_filename;
        this->_node_handle.getParam("/surface_smoother/save_mesh_filename", save_mesh_filename);
        ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","save_mesh_filename: " << save_mesh_filename);

        std::string save_mesh_filename_ply_with_dots = save_mesh_filename + std::string(".withdots.ply");
        std::string save_mesh_filename_ply = save_mesh_filename + std::string(".ply");
        pcl::io::savePLYFile(save_mesh_filename_ply_with_dots, processed_mesh);

        // hack because the file is saved using dots as decimal separator and meshlab freaks out
        std::string with_dots_line;
        std::ifstream with_dots;
        with_dots.open(save_mesh_filename_ply_with_dots);
        std::ofstream with_commas;
        with_commas.open(save_mesh_filename_ply);
        while (!with_dots.eof())
        {
            getline(with_dots, with_dots_line);
            std::replace(with_dots_line.begin(), with_dots_line.end(), '.', ',');
            with_commas << with_dots_line << std::endl;
        }

        with_dots.close();
        with_commas.close();

        std::string save_mesh_filename_stl = save_mesh_filename + std::string(".stl");

        vtkSmartPointer<vtkPolyData> vtk_polygon_data_ptr = vtkSmartPointer<vtkPolyData>::New ();
        vtkSmartPointer<vtkSTLWriter> vtk_polygon_writer_ptr = vtkSmartPointer<vtkSTLWriter>::New ();
        vtk_polygon_writer_ptr->SetFileTypeToBinary();

        pcl::io::mesh2vtk(processed_mesh, vtk_polygon_data_ptr);

        vtk_polygon_writer_ptr->SetInputData (vtk_polygon_data_ptr);
        vtk_polygon_writer_ptr->SetFileName (save_mesh_filename_stl.c_str ());
        vtk_polygon_writer_ptr->Write ();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(processed_mesh,"meshes");

    bool show_pc_with_mesh;
    this->_node_handle.getParam("/surface_smoother/show_pc_with_mesh", show_pc_with_mesh);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","show_pc_with_mesh: " << show_pc_with_mesh);
    if(show_pc_with_mesh)
    {
        viewer->addPointCloud(this->_current_pc, "cloudy");
    }

    viewer->addCoordinateSystem (0.10);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    ROS_ERROR_STREAM_NAMED("SurfaceSmootherNode.generateMesh","Finished! Send me another pc, you bastard!");
}

void SurfaceSmootherNode::colorMesh(pcl::PolygonMesh& mesh)
{    
    bool clean_mesh_while_coloring;
    this->_node_handle.getParam("/surface_smoother/clean_mesh_while_coloring", clean_mesh_while_coloring);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode.generateMesh","clean_mesh_while_coloring: " << clean_mesh_while_coloring);

    bool interpolate_colors;
    this->_node_handle.getParam("/surface_smoother/interpolate_colors", interpolate_colors);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","interpolate_colors: " << interpolate_colors);

    int num_neighbors_color_interp;
    this->_node_handle.getParam("/surface_smoother/num_neighbors_color_interp", num_neighbors_color_interp);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","num_neighbors_color_interp: " << num_neighbors_color_interp);

    float radius_search_color_interp;
    this->_node_handle.getParam("/surface_smoother/radius_search_color_interp", radius_search_color_interp);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","radius_search_color_interp: " << radius_search_color_interp);

    bool print_outs_color_interp;
    this->_node_handle.getParam("/surface_smoother/print_outs_color_interp", print_outs_color_interp);
    ROS_INFO_STREAM_NAMED("SurfaceSmootherNode","print_outs_color_interp: " << print_outs_color_interp);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (this->_current_pc);

    // Convert the PointCloud2 of the polygon mesh into a pcl point cloud xyz
    pcl::PointCloud<pcl::PointXYZ> cloud_mesh;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_mesh);

    // Convert the pcl pc xyz into a pcl pc xyzrgb
    pcl::PointCloud<pcl::PointXYZRGB> cloud_color_mesh;
    copyPointCloud(cloud_mesh, cloud_color_mesh);

    if(interpolate_colors)
    {
        // num_neighbors_color_interp nearest neighbor search
        std::vector<int> pointIdxNKNSearch(num_neighbors_color_interp);
        std::vector<float> pointNKNSquaredDistance(num_neighbors_color_interp);

        // find 4 nearest-neighbours to transfer colours using an IDW2 approach.
        for(size_t i=0; i< cloud_color_mesh.points.size();++i)
        {
            float red = 0.0,green = 0.0, blue = 0.0, dist = 0.0;
            int red_int = 0, green_int =0, blue_int=0;
            if(kdtree.radiusSearch(cloud_color_mesh.points[i], radius_search_color_interp, pointIdxNKNSearch, pointNKNSquaredDistance, num_neighbors_color_interp) > 0 )
            //if ( kdtree.nearestKSearch (cloud_color_mesh.points[i], num_neighbors_color_interp, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                //Inverse distance weighted colour assignment
                for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
                {
                    red += (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].r * 1.0/pointNKNSquaredDistance[j];
                    green += (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].g * 1.0/pointNKNSquaredDistance[j];
                    blue += (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].b * 1.0/pointNKNSquaredDistance[j];
                    dist += 1.0/pointNKNSquaredDistance[j];

                    if(print_outs_color_interp)
                    {
                        std::cout<<"partial red: "<< (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].r << std::endl;
                        std::cout<<"partial green: "<< (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].g << std::endl;
                        std::cout<<"partial blue: "<< (int)this->_current_pc->points[ pointIdxNKNSearch[j] ].b << std::endl;
                        std::cout<<"dist: "<< pointNKNSquaredDistance[j] << std::endl;
                        getchar();
                    }
                }
                red_int = floor(red/dist+0.5);
                green_int = floor(green/dist+0.5);
                blue_int = floor(blue/dist+0.5);
            }
            else if(clean_mesh_while_coloring)   //No neghbors in the radius!!!!
            {

                // If we need to delete a point+vertices that link to that point what we do is to delete the polygons
                // but we do not delete the points (otherwise all other polygons would get corrupted because point to the wrong points by index!)
                // We make the points of the mesh to be at 0,0,0
                cloud_color_mesh.points[i].x = 0;
                cloud_color_mesh.points[i].y = 0;
                cloud_color_mesh.points[i].z = 0;
                std::vector<int> polygons_to_remove;
                for(int k=0; k<mesh.polygons.size(); k++)
                {
                    // the vertex is in the poligon
                    if(std::find(mesh.polygons[k].vertices.begin(), mesh.polygons[k].vertices.end(), i) != mesh.polygons[k].vertices.end())
                    {

//                        mesh.polygons[k].vertices[0] = 0;
//                        mesh.polygons[k].vertices[1] = 0;
//                        mesh.polygons[k].vertices[2] = 0;
                        polygons_to_remove.push_back(k);
                    }
                }

                for(int r=polygons_to_remove.size() -1; r>=0; r--)
                {
                    mesh.polygons.erase(mesh.polygons.begin() + polygons_to_remove[r]);
                }
            }

            if(print_outs_color_interp)
            {
                std::cout<<"red: "<< red << std::endl;
                std::cout<<"green: "<< green << std::endl;
                std::cout<<"blue: "<< blue << std::endl;
                std::cout<<"red int: "<< red_int << std::endl;
                std::cout<<"green int: "<< green_int << std::endl;
                std::cout<<"blue int: "<< blue_int << std::endl;
                std::cout<<"--------"<<std::endl;
                getchar();
            }

            // If no close points are found we will have an exception
            cloud_color_mesh.points[i].r = red_int;
            cloud_color_mesh.points[i].g = green_int;
            cloud_color_mesh.points[i].b = blue_int;
        }
    }else{

        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // find 1 nearest-neighbour to transfer colour directly
        for(size_t i=0; i< cloud_color_mesh.points.size();++i)
        {
            int red_int = 0, green_int =0, blue_int=0;
            if(kdtree.radiusSearch(cloud_color_mesh.points[i], radius_search_color_interp, pointIdxNKNSearch, pointNKNSquaredDistance, 1) > 0 )
            //if(kdtree.nearestKSearch(cloud_color_mesh.points[i], radius_search_color_interp, pointIdxNKNSearch, pointNKNSquaredDistance, num_neighbors_color_interp) > 0 )
            {
                    red_int = (int)this->_current_pc->points[ pointIdxNKNSearch[0] ].r ;
                    green_int = (int)this->_current_pc->points[ pointIdxNKNSearch[0] ].g ;
                    blue_int = (int)this->_current_pc->points[ pointIdxNKNSearch[0] ].b ;
            }else if(clean_mesh_while_coloring)   //No neghbors in the radius!!!!
            {
                // If we need to delete a point+vertices that link to that point what we do is to delete the polygons
                // but we do not delete the points (otherwise all other polygons would get corrupted because point to the wrong points by index!)
                // We make the points of the mesh to be at 0,0,0
                cloud_color_mesh.points[i].x = 0;
                cloud_color_mesh.points[i].y = 0;
                cloud_color_mesh.points[i].z = 0;
                std::vector<int> polygons_to_remove;
                for(int k=0; k<mesh.polygons.size(); k++)
                {
                    // the vertex is in the poligon
                    if(std::find(mesh.polygons[k].vertices.begin(), mesh.polygons[k].vertices.end(), i) != mesh.polygons[k].vertices.end())
                    {
//                        mesh.polygons[k].vertices[0] = 0;
//                        mesh.polygons[k].vertices[1] = 0;
//                        mesh.polygons[k].vertices[2] = 0;
                        polygons_to_remove.push_back(k);
                    }
                }
                for(int r=polygons_to_remove.size() -1; r>=0; r--)
                {
                    mesh.polygons.erase(mesh.polygons.begin() + polygons_to_remove[r]);
                }
            }

            if(print_outs_color_interp)
            {
                std::cout<<"red int: "<< red_int << std::endl;
                std::cout<<"green int: "<< green_int << std::endl;
                std::cout<<"blue int: "<< blue_int << std::endl;
                std::cout<<"--------"<<std::endl;
                getchar();
            }

            // If no close points are found we will have an exception
            cloud_color_mesh.points[i].r = red_int;
            cloud_color_mesh.points[i].g = green_int;
            cloud_color_mesh.points[i].b = blue_int;
        }
    }

    // Convert the pcl pc xyzrgb into PointCloud2 of the mesh
    pcl::toPCLPointCloud2(cloud_color_mesh, mesh.cloud);

    if(clean_mesh_while_coloring)
    {
        pcl::PolygonMesh cleaned_mesh;
        pcl::surface::SimplificationRemoveUnusedVertices sruv;
        sruv.simplify(mesh, cleaned_mesh);
        mesh = cleaned_mesh;
    }


}

using namespace vcg;

class MyVertex; class MyEdge; class MyFace;
struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex>   ::AsVertexType,
                                           vcg::Use<MyEdge>     ::AsEdgeType,
                                           vcg::Use<MyFace>     ::AsFaceType>{};
class MyVertex  : public vcg::Vertex< MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  >{};
class MyFace    : public vcg::Face<   MyUsedTypes, vcg::face::FFAdj,  vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::BitFlags > {};
class MyEdge    : public vcg::Edge<   MyUsedTypes> {};
class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> , std::vector<MyEdge>  > {};


void SurfaceSmootherNode::orientNormalsInMesh(pcl::PolygonMesh& mesh)
{
    // VCG library implementation
    MyMesh m;

    // Convert pcl::PolygonMesh to VCG MyMesh
    m.Clear();

    // Create temporary cloud in to have handy struct object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(mesh.cloud,*cloud1);

    // Now convert the vertices to VCG MyMesh
    int vertCount = cloud1->width*cloud1->height;
    vcg::tri::Allocator<MyMesh>::AddVertices(m, vertCount);
    for(unsigned int i=0;i<vertCount;++i)
        m.vert[i].P()=vcg::Point3f(cloud1->points[i].x,cloud1->points[i].y,cloud1->points[i].z);

    // Now convert the polygon indices to VCG MyMesh => make VCG faces..
    int triCount = mesh.polygons.size();
    if(triCount==1)
    {
        if(mesh.polygons[0].vertices[0]==0 && mesh.polygons[0].vertices[1]==0 && mesh.polygons[0].vertices[2]==0)
            triCount=0;
    }
    vcg::tri::Allocator<MyMesh>::AddFaces(m, triCount);
    for(unsigned int i=0;i<triCount;++i)
    {
        m.face[i].V(0)=&m.vert[mesh.polygons[i].vertices[0]];
        m.face[i].V(1)=&m.vert[mesh.polygons[i].vertices[1]];
        m.face[i].V(2)=&m.vert[mesh.polygons[i].vertices[2]];
    }

    vcg::tri::UpdateBounding<MyMesh>::Box(m);
    vcg::tri::UpdateNormal<MyMesh>::PerFace(m);
    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(m);
    printf("Input mesh  vn:%i fn:%i\n",m.VN(),m.FN());

    // Start to flip all normals to outside
    vcg::face::FFAdj<MyMesh>();
    vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
    bool oriented, orientable;
    if ( vcg::tri::Clean<MyMesh>::CountNonManifoldEdgeFF(m)>0 ) {
        std::cout << "Mesh has some not 2-manifold faces, Orientability requires manifoldness" << std::endl; // text
        return; // can't continue, mesh can't be processed
    }

    vcg::tri::Clean<MyMesh>::OrientCoherentlyMesh(m, oriented, orientable);
    //vcg::tri::Clean<MyMesh>::FlipNormalOutside(m);
    //vcg::tri::Clean<MyMesh>::FlipMesh(m);
    //vcg::tri::UpdateTopology<MyMesh>::FaceFace(m);
    //vcg::tri::UpdateTopology<MyMesh>::TestFaceFace(m);
    vcg::tri::UpdateNormal<MyMesh>::PerVertexNormalizedPerFace(m);
    vcg::tri::UpdateNormal<MyMesh>::PerVertexFromCurrentFaceNormal(m);

    // now convert VCG back to pcl::PolygonMesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = false;
    cloud->width = vertCount;
    cloud->height = 1;
    cloud->points.resize (vertCount);
    // Now fill the pointcloud of the mesh
    for(int i=0; i<vertCount; i++)
    {
        cloud->points[i].x = m.vert[i].P()[0];
        cloud->points[i].y = m.vert[i].P()[1];
        cloud->points[i].z = m.vert[i].P()[2];
    }
    pcl::toPCLPointCloud2(*cloud,mesh.cloud);
    std::vector<pcl::Vertices> polygons;
    // Now fill the indices of the triangles/faces of the mesh
    for(int i=0; i<triCount; i++)
    {
        pcl::Vertices vertices;
        vertices.vertices.push_back(m.face[i].V(0)-&*m.vert.begin());
        vertices.vertices.push_back(m.face[i].V(1)-&*m.vert.begin());
        vertices.vertices.push_back(m.face[i].V(2)-&*m.vert.begin());
        polygons.push_back(vertices);
    }
    mesh.polygons = polygons;
}

// Main program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "SurfaceSmootherNode");
    SurfaceSmootherNode sr_node;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    std::cout << " Shutting down SurfaceSmootherNode " << std::endl;

    return (0);
}
