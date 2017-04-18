/*
 * MultiJointTrackerNode.h
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

#ifndef JOINT_TRACKER_NODE_H
#define JOINT_TRACKER_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include "joint_tracker/publish_urdf.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/publisher.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "joint_tracker/JointCombinedFilter.h"
#include "joint_tracker/MultiJointTracker.h"

#include "omip_msgs/RigidBodyPosesAndVelsMsg.h"
#include "omip_msgs/RigidBodyPoseAndVelMsg.h"

#include "omip_common/RecursiveEstimatorNodeInterface.h"
#include "omip_common/OMIPTypeDefs.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <std_msgs/Float64MultiArray.h>

namespace omip{

class MultiJointTrackerNode  : public RecursiveEstimatorNodeInterface<ks_measurement_ros_t, ks_state_ros_t, MultiJointTracker>
{

public:

    /**
   * Constructor
   */
    MultiJointTrackerNode();

    /**
   * Destructor
   */
    virtual ~MultiJointTrackerNode();

    /**
   * Read parameters to create the JointTracker
   */
    void ReadParameters();

    /**
   * @brief Callback for the measurements for this RE level (Poses and vels of the tracker RB from the lower level, RBT)
   *
   * @param poses_and_vels Input tracked RB poses and vels
   */
    virtual void measurementCallback(const boost::shared_ptr<ks_measurement_ros_t const> &poses_and_vels);
    virtual void measurementFTCallback(const std_msgs::Float64MultiArrayConstPtr &ft_values);
    virtual void measurementEE2CPCallback(const std_msgs::Float64MultiArrayConstPtr &rel_pose);
    virtual void slippageDetectedCallback(const std_msgs::Int32ConstPtr &msg);


    /**
   * @brief Callback for the predictions about the state of this RE level coming from the higher level of the hierarchy
   *
   * @param predicted_next_state Predictions about the next state. Right now there is no higher lever (unused)
   */
    virtual void statePredictionCallback(const boost::shared_ptr<ks_state_ros_t const> & predicted_next_state){}

    /**
   * Version to work offline. Not implemented
   */
    void ReadBag(){}

    virtual bool publishURDF(joint_tracker::publish_urdf::Request& request, joint_tracker::publish_urdf::Response& response);

protected:

    /**
   * @brief Publish the current state of this RE level
   *
   */
    virtual void _publishState() const;

    /**
   * @brief Publish the prediction about the next measurement by this RE level
   *
   */
    virtual void _publishPredictedMeasurement() const;

    /**
   * @brief Print the results of the joint tracker on the terminal
   *
   */
    virtual void _PrintResults() const;

    virtual void _generateURDF(std_msgs::String &urdf_string_msg, sensor_msgs::JointState &joint_state_msg) const;

    ros::ServiceServer _urdf_pub_service;

    double _sensor_fps;
    int _processing_factor;
    double _loop_period_ns;


    ros::Publisher _state_publisher_urdf;

    ros::Publisher _state_publisher_joint_states;

    ros::Publisher _state_publisher_rviz_markers;

    int _min_joint_age_for_ee;

    std::string _sr_path;

    bool _robot_interaction;

    ros::Subscriber _measurement_subscriber_ft;
    ros::Subscriber _measurement_subscriber_cp;
    ros::Subscriber _slippage_detector_cp;

    std::string _ft_topic;
    std::vector<double> _last_ft_values;
    std::vector<double> _last_ee2cp_relpose;
    bool _ft_values_being_used;
    ros::Publisher _grasping_type_publisher;

};
}

#endif /* JOINT_TRACKER_NODE_H */
