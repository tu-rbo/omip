/*
 * MultiRBTrackerNode.h
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

#ifndef MULTI_RB_TRACKER_NODE_H
#define MULTI_RB_TRACKER_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <rb_tracker/RBTrackerDynReconfConfig.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/publisher.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "rb_tracker/RBFilter.h"
#include "rb_tracker/MultiRBTracker.h"

#include "omip_msgs/RigidBodyPosesMsg.h"
#include "omip_msgs/RigidBodyPoseMsg.h"

#include "omip_msgs/RigidBodyPosesAndVelsMsg.h"
#include "omip_msgs/RigidBodyPoseAndVelMsg.h"

#include "omip_msgs/RigidBodyTwistsWithCovMsg.h"
#include "omip_msgs/RigidBodyTwistWithCovMsg.h"

#include "omip_common/RecursiveEstimatorNodeInterface.h"
#include "omip_common/OMIPTypeDefs.h"

#include <std_msgs/Float64.h>

#include "omip_msgs/ShapeTrackerStates.h"

namespace omip{

class MultiRBTrackerNode : public RecursiveEstimatorNodeInterface<rbt_measurement_ros_t, rbt_state_ros_t, MultiRBTracker>
{

public:

  /**
   * Constructor
   */
  MultiRBTrackerNode();

  /**
   * Destructor
   */
  virtual ~MultiRBTrackerNode();

  /**
   * @brief Callback for the measurements for this RE level (3D features from the lower level, FT)
   *
   * @param features_pc Input point cloud of the 3D tracked features
   */
  virtual void measurementCallback(const boost::shared_ptr<rbt_measurement_ros_t const> &features_pc);

  void MeasurementFromShapeTrackerCallback(const boost::shared_ptr<omip_msgs::ShapeTrackerStates const> &shape_tracker_states);

  void MatthiasRefinementsCallback(const boost::shared_ptr<rbt_state_t const> &matthias_refinements);

  void RGBDPCCallback(const sensor_msgs::PointCloud2ConstPtr &full_pc_msg);

  /**
   * @brief Callback for the predictions about the state of this RE level coming from the higher level of the hierarchy
   *
   * @param predicted_next_state Predictions about the next state (next feature 3D locations) from the RBT
   */
  virtual void statePredictionCallback(const boost::shared_ptr<rbt_state_t const> &predicted_next_state);

  /**
   * @brief Callback for the Dynamic Reconfigure parameters
   *
   * @param config Values from the Dynamic Reconfigure server
   * @param level
   */
  void DynamicReconfigureCallback(rb_tracker::RBTrackerDynReconfConfig &config, uint32_t level);

  /**
   * Read parameters to create the RBMEKFTracker
   */
  void ReadParameters();

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
   * @brief Print the results of the RB tracker on the terminal
   *
   */
  virtual void _PrintResults() const;

  /**
   * @brief Publish the tracked features clustered in RBs (the cluster id is written in the "label" field of
   * the published PointCloud<PointXYZL>
   *
   */
  virtual void _PublishClusteredFeatures();

  /**
   * Write the results of the tracking in the terminal and publish the poses and covariances to ROS
   */
  virtual void _PublishPosesWithCovariance();

  /**
   * Publish the results of the tracking into the TF system of ROS
   * We publish the tracked RBs as frames relative to the camera
   */
  virtual void _PublishTF();

  std::map<RB_id_t, ros::Publisher*> _est_body_publishers;
  ros::Publisher _rbposes_publisher;
  ros::Publisher _clustered_pc_publisher;

  ros::Publisher _freefeats_pc_publisher;
  ros::Publisher _predictedfeats_pc_publisher;
  ros::Publisher _atbirthfeats_pc_publisher;

  ros::Subscriber _meas_from_st_subscriber;
  ros::Subscriber _matthias_refinements_subscriber;

  // Maximum number of iterations of RANSAC to find a good hypothesis for the free features
  int _ransac_iterations;

  // Maximum error allowed between predicted and measured feature position to consider it to STILL
  // support a RBM
  double _estimation_error_threshold;

  // Minimum motion to consider that a feature moves
  double _static_motion_threshold;

  // Maximum error allowed for the inliers of a new RBM hypothesis in RANSAC
  double _new_rbm_error_threshold;

  // Maximum error allowed between the predicted and the measured position of a feature to assign it to a RBM
  double _max_error_to_reassign_feats;

  double _sensor_fps;
  int _processing_factor;
  double _loop_period_ns;

  // Minimum number of features that have to support a RBM to not be deleted
  int _supporting_features_threshold;

  // Total number of tracked features
  int _num_tracked_feats;

  // Minimum number of free features to trigger the generation of a new RBM
  int _min_num_feats_for_new_rb;

  // Minimum number of frames that a features has to be present to be used to create new RBM
  int _min_num_frames_for_new_rb;

  // Flags
  bool _publishing_rbposes_with_cov;
  bool _publishing_tf;
  bool _publishing_clustered_pc;
  bool _printing_rb_poses;  

  static_environment_tracker_t _static_environment_tracker_type;

  mutable bool _shape_tracker_received;
  omip_msgs::ShapeTrackerStates _shape_tracker_meas;

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<rb_tracker::RBTrackerDynReconfConfig> _dr_srv;
  dynamic_reconfigure::Server<rb_tracker::RBTrackerDynReconfConfig>::CallbackType _dr_callback;

  ros::Publisher _state_publisher2;
  Eigen::Twistd _previous_twist;
  rbt_state_t _last_predictions_kh;

};

}

#endif /* MULTI_RB_TRACKER_NODE_H */
