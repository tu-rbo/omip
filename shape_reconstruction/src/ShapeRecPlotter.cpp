#include "shape_reconstruction/ShapeRecPlotter.h"

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

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>


using namespace omip;

ShapeRecPlotter::ShapeRecPlotter()
{
    std::string topic_name = std::string("");
    //this->_node_handle.getParam("/surface_smoother/topic_name", topic_name);
    ROS_INFO_STREAM_NAMED("ShapeRecPlotter","topic_name: " << topic_name);
    this->_pc_subscriber = this->_node_handle.subscribe(topic_name, 1,
                                                        &ShapeRecPlotter::InputPCmeasurementCallback, this);

    std::string topic_name2 = std::string("moved_pc");
    this->_pc_publisher = this->_node_handle.advertise<sensor_msgs::PointCloud2>(topic_name2,10, true);

    this->_current_pc.reset(new SRPointCloud());



    _tf_listener = new tf::TransformListener();

}

ShapeRecPlotter::~ShapeRecPlotter()
{
}

void ShapeRecPlotter::plotShape()
{
    if(_current_pc->size())
    {
    sensor_msgs::PointCloud2 pc2, pcout;

    pcl::toROSMsg(*this->_current_pc, pc2);

    _tf_listener->waitForTransform("/camera_rgb_optical_frame", "/ip/rb2", ros::Time(0), ros::Duration(0.1));

    bool nooo = false;
    tf::StampedTransform tfTransform;
    try{

    _tf_listener->lookupTransform ("/camera_rgb_optical_frame", "/ip/rb2",ros::Time(0), tfTransform);

    }catch(...){
        nooo = true;
        std::cout << "." << std::endl ;
    }

    if(!nooo)
    {
        std::cout << std::endl ;
    Eigen::Matrix4f transform;
    transform(0,0) = tfTransform.getBasis()[0][0];
    transform(0,1) = tfTransform.getBasis()[0][1];
    transform(0,2) = tfTransform.getBasis()[0][2];
    transform(1,0) = tfTransform.getBasis()[1][0];
    transform(1,1) = tfTransform.getBasis()[1][1];
    transform(1,2) = tfTransform.getBasis()[1][2];
    transform(2,0) = tfTransform.getBasis()[2][0];
    transform(2,1) = tfTransform.getBasis()[2][1];
    transform(2,2) = tfTransform.getBasis()[2][2];
    transform(0,3) = tfTransform.getOrigin()[0];
    transform(1,3) = tfTransform.getOrigin()[1];
    transform(2,3) = tfTransform.getOrigin()[2];
    transform(3,0) = 0;
    transform(3,1) = 0;
    transform(3,2) = 0;
    transform(3,3) = 1;

    std::cout << transform << std::endl;
    pcl_ros::transformPointCloud(transform, pc2, pcout);

    _pc_publisher.publish(pcout);
    }else{
        std::cout << std::endl ;
    Eigen::Matrix4f transform;
    transform(0,0) = 1;
    transform(0,1) = 0;
    transform(0,2) = 0;
    transform(1,0) = 0;
    transform(1,1) = 1;
    transform(1,2) = 0;
    transform(2,0) = 0;
    transform(2,1) = 0;
    transform(2,2) = 1;
    transform(0,3) = 0;
    transform(1,3) = 0;
    transform(2,3) = 0;
    transform(3,0) = 0;
    transform(3,1) = 0;
    transform(3,2) = 0;
    transform(3,3) = 1;
    pcl_ros::transformPointCloud(transform, pc2, pcout);

    _pc_publisher.publish(pcout);
    }
    }

}

void ShapeRecPlotter::InputPCmeasurementCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
    std::cout << "received" << std::endl;
    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(*pc_msg, *this->_current_pc);

}

// Main program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ShapeRecPlotter");
    ShapeRecPlotter sr_node;

    ros::Rate r(30); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        sr_node.plotShape();
        r.sleep();
    }

    std::cout << " Shutting down ShapeRecPlotter " << std::endl;

    return (0);
}
