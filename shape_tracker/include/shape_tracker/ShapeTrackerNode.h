/*
 * ShapeTrackerNode.h
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

#ifndef SHAPE_TRACKER_NODE_H_
#define SHAPE_TRACKER_NODE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosbag/bag.h>
#include <tf/tf.h>

#include <boost/thread.hpp>

#include <omip_common/OMIPTypeDefs.h>
#include <omip_msgs/ShapeModels.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

#include <omip_common/OMIPTypeDefs.h>

//ROS and OpenCV
#include <opencv2/core/core.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <omip_common/OMIPUtils.h>

#include <shape_tracker/ShapeTracker.h>

#include <std_msgs/Empty.h>


namespace omip
{

class ShapeTrackerNode
{
    // Policies to synchorize point clouds, and the RBP from the RBT
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, omip::rbt_state_t> STSyncPolicy;

    typedef std::map<omip::RB_id_t, omip::ShapeTrackerPtr > shape_trackers_map_t;

public:

    /**
     * Constructor
     */
    ShapeTrackerNode();

    /**
     * Destructor
     */
    virtual ~ShapeTrackerNode();

    virtual void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci_msg);

    virtual void ShapeModelsCallback(const omip_msgs::ShapeModelsConstPtr &models_msg);

    virtual void RigibBodyMotionsAndPCCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const boost::shared_ptr<omip::rbt_state_t const> &poses_and_vels);

    virtual void TrackerNodeQuitCallback(const std_msgs::EmptyConstPtr &msg);

    virtual bool getActive() const
    {
        return this->_active;
    }

protected:

    shape_trackers_map_t			                                _rb_trackers;
    ros::NodeHandle                                                             _node_handle;
    ros::Publisher                                                              _st_state_pub;

    message_filters::Subscriber<sensor_msgs::PointCloud2>                       _rgbd_pc_subscriber;
    message_filters::Subscriber<omip::rbt_state_t>                              _poses_and_vels_subscriber;
    message_filters::Synchronizer<STSyncPolicy>*                                _synchronizer;

    ros::Subscriber                      _shape_models_subscriber;

    ros::Subscriber                                                             _ci_sub;
    sensor_msgs::CameraInfo _ci;

    shape_model_selector_t _model_type_to_listen;

    ros::Subscriber _node_quit_subscriber;

    bool _active;

    template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
        if (!(this->_node_handle.getParam(param_name, param_container)))
        {
            ROS_ERROR_NAMED("ShapeTrackerNode.getROSParameter", "The parameter %s can not be found.", param_name.c_str());
            throw(std::string("[ShapeTrackerNode.getROSParameter] The parameter can not be found. Parameter name: ") + param_name);
            return false;
        }
        else
            return true;
    }
};
}

#endif /* FEATURE_TRACKER_NODE_H_ */

