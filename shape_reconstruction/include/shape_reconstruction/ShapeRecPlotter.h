/*
 * ShapeRecPlotter.h
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

#ifndef SHAPE_RECONSTRUCTION_PLOTTER_NODE_H_
#define SHAPE_RECONSTRUCTION_PLOTTER_NODE_H_

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

#include "shape_reconstruction/ShapeRecPlotter.h"
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

#include <tf/transform_listener.h>

namespace omip
{

class ShapeRecPlotter
{

public:

    /**
     * Constructor
     */
    ShapeRecPlotter();

    /**
     * Destructor
     */
    virtual ~ShapeRecPlotter();

    virtual void InputPCmeasurementCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

    void plotShape();

protected:


    ros::NodeHandle                                                             _node_handle;

    ros::Subscriber                                                             _pc_subscriber;

    ros::Publisher                                                             _pc_publisher;

    SRPointCloud::Ptr                                                           _current_pc;

    tf::TransformListener *                                 _tf_listener;


    template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
        if (!(this->_node_handle.getParam(param_name, param_container)))
        {
            ROS_ERROR_NAMED("ShapeRecPlotter.getROSParameter", "The parameter %s can not be found.", param_name.c_str());
            throw(std::string("[ShapeRecPlotter.getROSParameter] The parameter can not be found. Parameter name: ") + param_name);
            return false;
        }
        else
            return true;
    }
};
}

#endif /* FEATURE_TRACKER_NODE_H_ */

