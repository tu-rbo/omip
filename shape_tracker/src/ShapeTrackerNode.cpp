#include "shape_tracker/ShapeTrackerNode.h"

#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ros/package.h>

#include <cmath>

#include <ros/console.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <boost/filesystem.hpp>
#include <ctime>

#include <omip_msgs/ShapeTrackerStates.h>

using namespace omip;

ShapeTrackerNode::ShapeTrackerNode() :
    _active(true)
{

    this->_node_quit_subscriber = this->_node_handle.subscribe("/omip/shutdown", 1, &ShapeTrackerNode::TrackerNodeQuitCallback, this);

    this->_rgbd_pc_subscriber.subscribe(this->_node_handle, "/camera/depth_registered/points",1);
    this->_poses_and_vels_subscriber.subscribe(this->_node_handle, "/rb_tracker/state_after_feat_correction",1);

    this->_synchronizer = new message_filters::Synchronizer<STSyncPolicy>(STSyncPolicy(10), this->_rgbd_pc_subscriber, this->_poses_and_vels_subscriber);
    this->_synchronizer->registerCallback(boost::bind(&ShapeTrackerNode::RigibBodyMotionsAndPCCallback, this, _1, _2));

    int model_to_listen;
    this->_node_handle.getParam("/shape_tracker/model_type_to_listen", model_to_listen);
    this->_model_type_to_listen = (shape_model_selector_t)model_to_listen;
    ROS_INFO_STREAM_NAMED("ShapeTrackerNode.ShapeTrackerNode","model_type_to_listen = " << this->_model_type_to_listen);

    this->_shape_models_subscriber = this->_node_handle.subscribe("/shape_recons/state", 10, &ShapeTrackerNode::ShapeModelsCallback, this);

    this->_st_state_pub = this->_node_handle.advertise<omip_msgs::ShapeTrackerStates>("/shape_tracker/state", 1);

    std::string ci_topic;
    this->getROSParameter<std::string>(std::string("/feature_tracker/camera_info_topic"),ci_topic);
    this->_ci_sub = this->_node_handle.subscribe(ci_topic, 1,
                                                 &ShapeTrackerNode::CameraInfoCallback, this);
}

ShapeTrackerNode::~ShapeTrackerNode()
{

}

void ShapeTrackerNode::TrackerNodeQuitCallback(const std_msgs::EmptyConstPtr &msg)
{
    ROS_INFO_STREAM("Shape Tracker node quit stopping!");
    _active = false;
}

void ShapeTrackerNode::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci_msg)
{
    // We have to do this because sometimes the first ci message that is sent is zero (why?)
    if(ci_msg->height != 0)
    {
        this->_ci = sensor_msgs::CameraInfo(*ci_msg);
        this->_ci_sub.shutdown();
        for(shape_trackers_map_t::iterator it = this->_rb_trackers.begin(); it!=this->_rb_trackers.end(); it++)
        {
            it->second->setCameraInfo(this->_ci);
        }
    }
}

void ShapeTrackerNode::ShapeModelsCallback(const omip_msgs::ShapeModelsConstPtr &models_msg)
{
    ROS_INFO_STREAM_NAMED("ShapeTrackerNode.ShapeModelsCallback", "Received new shape models. " << models_msg->rb_shape_models.size() << " models.");
    omip::RB_id_t rb_id_temp;
    for(int rb_models_idx = 0; rb_models_idx < models_msg->rb_shape_models.size(); rb_models_idx++)
    {
        rb_id_temp = models_msg->rb_shape_models.at(rb_models_idx).rb_id;
        int num_points = models_msg->rb_shape_models.at(rb_models_idx).rb_shape_model.height*models_msg->rb_shape_models.at(rb_models_idx).rb_shape_model.width;
        ROS_INFO_STREAM_NAMED("ShapeTrackerNode.ShapeModelsCallback",
                              "Received new shape model of RB " << rb_id_temp << " containing " << num_points  << " points");

        // If this RB (identified by its id) was previously tracked, we already have a ShapeTracker object for it
        if(this->_rb_trackers.find(rb_id_temp) == this->_rb_trackers.end())
        {
            omip::ShapeTrackerPtr tracker_ptr = omip::ShapeTrackerPtr(new omip::ShapeTracker(rb_id_temp));
            tracker_ptr->setCameraInfo(this->_ci);
            this->_rb_trackers[rb_id_temp] = tracker_ptr;

        }
        this->_rb_trackers[rb_id_temp]->setShapeModel(models_msg->rb_shape_models.at(rb_models_idx).rb_shape_model);
    }
}

void ShapeTrackerNode::RigibBodyMotionsAndPCCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const boost::shared_ptr<omip::rbt_state_t const> &poses_and_vels)
{
    //ROS_WARN_STREAM( "Received. Poses time: " << poses_and_vels->header.stamp << " PC time: " << pc_msg->header.stamp );
    ros::WallTime t_init = ros::WallTime::now();
    omip::RB_id_t rb_id_temp;
    omip_msgs::ShapeTrackerStates shape_tracker_states;
    for(int rb_poses_idx = 1; rb_poses_idx < poses_and_vels->rb_poses_and_vels.size(); rb_poses_idx++)
    {
        rb_id_temp = poses_and_vels->rb_poses_and_vels.at(rb_poses_idx).rb_id;

        omip_msgs::ShapeTrackerState shape_tracker_state;
        // If this RB (identified by its id) was previously tracked, we already have a ShapeTracker object for it
        if(this->_rb_trackers.find(rb_id_temp) != this->_rb_trackers.end())
        {
            this->_rb_trackers[rb_id_temp]->step(pc_msg,
                                                 poses_and_vels->rb_poses_and_vels.at(rb_poses_idx),
                                                 shape_tracker_state);
            shape_tracker_states.shape_tracker_states.push_back(shape_tracker_state);
        }
    }
    //ROS_WARN_STREAM( "poses time minus pc time: " << (poses_and_vels->header.stamp - pc_msg->header.stamp).toSec() );
    shape_tracker_states.header.stamp = poses_and_vels->header.stamp;
    this->_st_state_pub.publish(shape_tracker_states);
    ros::WallTime t_end = ros::WallTime::now();
    ROS_ERROR_STREAM("Total time ST: " << t_end - t_init);
}

// Main program
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ShapeTrackerNode");
    ShapeTrackerNode st_node;

    ros::Rate r(100); // 10 hz
    while (ros::ok() && st_node.getActive())
    {
        ros::spinOnce();
        r.sleep();
    }

    ROS_ERROR_STREAM("Shutting down ShapeTrackerNode");

    return (0);
}
