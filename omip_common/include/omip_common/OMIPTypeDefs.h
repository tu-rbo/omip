/*
 * TypeDefs.h
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

#ifndef OMIPTYPEDEFS_H_
#define OMIPTYPEDEFS_H_

// ROS, PCL, Opencv
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

// OMIP
#include <omip_msgs/RigidBodyPosesAndVelsMsg.h>
#include <omip_msgs/RigidBodyPoseAndVelMsg.h>
#include <omip_msgs/JointMsg.h>
#include <omip_msgs/KinematicStructureMsg.h>

#define OMIP_ADD_POINT4D \
  union { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  } EIGEN_ALIGN16; \
  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

namespace omip
{

//Forward declaration
class JointFilter;

//Forward declaration
class JointCombinedFilter;

typedef pcl::PointXYZ PointPCL;
typedef pcl::PointXYZL FeaturePCL;

struct EIGEN_ALIGN16 _FeaturePCLwc
{
  OMIP_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  uint32_t label;
  float covariance[9];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FeaturePCLwc : public _FeaturePCLwc
{
  inline FeaturePCLwc ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    label = 0;
    covariance[0] = 1.0f;
    covariance[1] = 0.0f;
    covariance[2] = 0.0f;
    covariance[3] = 0.0f;
    covariance[4] = 1.0f;
    covariance[5] = 0.0f;
    covariance[6] = 0.0f;
    covariance[7] = 0.0f;
    covariance[8] = 1.0f;
  }
};

typedef pcl::PointCloud<PointPCL> PointCloudPCLNoColor;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudPCL;
typedef pcl::PointCloud<FeaturePCL> FeatureCloudPCL;
typedef pcl::PointCloud<FeaturePCLwc> FeatureCloudPCLwc;

typedef long int RB_id_t;
typedef long int Joint_id_t;

typedef pcl::PointCloud<PointPCL> RigidBodyShape;

typedef std::map<std::pair<int, int>, boost::shared_ptr<JointCombinedFilter> > KinematicModel;

// Feature Tracker NODE MEASUREMENT type. We don't use this type because we need to synchronize several
// inputs as measurement for the feature tracker node (at least depth and rgb images) and this cannot be handled
// correctly with one single type definition
typedef void ft_measurement_ros_t;

// Feature Tracker FILTER MEASUREMENT type. The first pointer points to the RGB image. The second pointer points to the depth image
typedef std::pair<cv_bridge::CvImagePtr, cv_bridge::CvImagePtr> ft_measurement_t;

// Feature Tracker NODE STATE type
typedef sensor_msgs::PointCloud2 ft_state_ros_t;

// Feature Tracker FILTER STATE type
typedef FeatureCloudPCLwc::Ptr ft_state_t;

// Rigid Body Tracker NODE MEASUREMENT type (= ft_state_ros_t).
typedef sensor_msgs::PointCloud2 rbt_measurement_ros_t;

// Rigid Body Tracker FILTER MEASUREMENT type.
typedef FeatureCloudPCLwc::Ptr rbt_measurement_t;

// Rigid Body Tracker NODE STATE type
typedef omip_msgs::RigidBodyPosesAndVelsMsg rbt_state_ros_t;

// Rigid Body Tracker FILTER STATE type
typedef omip_msgs::RigidBodyPosesAndVelsMsg rbt_state_t;

// Kinematic Model Tracker NODE MEASUREMENT type.
typedef omip_msgs::RigidBodyPosesAndVelsMsg ks_measurement_ros_t;

// Kinematic Model Tracker FILTER MEASUREMENT type.
typedef omip_msgs::RigidBodyPosesAndVelsMsg ks_measurement_t;

// Kinematic Model tracker NODE STATE type
typedef omip_msgs::KinematicStructureMsg::Ptr ks_state_ros_t;

// Kinematic Model Tracker FILTER STATE type
typedef KinematicModel ks_state_t;

// Measurement type for the combined joint filter and all joint filters
typedef std::pair<omip_msgs::RigidBodyPoseAndVelMsg, omip_msgs::RigidBodyPoseAndVelMsg> joint_measurement_t;

enum shape_model_selector_t
{
    BASED_ON_DEPTH = 1,
    BASED_ON_COLOR = 2,
    BASED_ON_EXT_DEPTH = 3,
    BASED_ON_EXT_COLOR = 4,
    BASED_ON_EXT_DEPTH_AND_COLOR = 5
};

enum static_environment_tracker_t
{
    STATIC_ENVIRONMENT_EKF_TRACKER = 1,
    STATIC_ENVIRONMENT_ICP_TRACKER = 2
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(omip::FeaturePCLwc,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint32_t, label, label)
                                  (float[9], covariance, covariance))


#endif /* OMIPTYPEDEFS_H_ */
