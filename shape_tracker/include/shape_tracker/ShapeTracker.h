/*
 * ShapeTracker.h
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

#ifndef SHAPE_TRACKER_H_
#define SHAPE_TRACKER_H_

#include <omip_common/OMIPTypeDefs.h>

//ROS and OpenCV
#include <opencv2/core/core.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include <omip_common/OMIPUtils.h>

#include <omip_msgs/ShapeTrackerState.h>

#include <opencv2/highgui/highgui.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>

#ifdef USE_LIBPOINTMATCHER_ICP
#include "shape_tracker/point_cloud_conversions.h"
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
#endif

namespace omip
{

void calculate_ICP_COV(pcl::PointCloud<pcl::PointXYZ>& data_pi, pcl::PointCloud<pcl::PointXYZ>& model_qi, Eigen::Matrix4f& transform, Eigen::MatrixXd& ICP_COV);

class ShapeTracker;
typedef boost::shared_ptr<ShapeTracker> ShapeTrackerPtr;

class ShapeTracker
{
public:
    ShapeTracker(omip::RB_id_t rb_id);

    ShapeTrackerPtr clone() const
    {
        return (ShapeTrackerPtr(doClone()));
    }

    ShapeTracker(const ShapeTracker &sr);

    virtual ~ShapeTracker();

    virtual void step(const sensor_msgs::PointCloud2ConstPtr& pc_msg,
                      const omip_msgs::RigidBodyPoseAndVelMsg& tracked_rbm,
                      omip_msgs::ShapeTrackerState& st_state);

    virtual void setShapeModel(const sensor_msgs::PointCloud2& model);

    virtual void setCameraInfo(const sensor_msgs::CameraInfo& camera_info);

protected:

    virtual void _RemoveInconsistentPointsFromRBModel(pcl::PointCloud<pcl::PointXYZ >::Ptr model_current_pose,
                                                      pcl::PointCloud<pcl::PointXYZ >::Ptr current_pc,
                                                      pcl::PointCloud<pcl::PointXYZ >::Ptr& rb_segment_depth,
                                                      pcl::PointCloud<pcl::PointXYZ >::Ptr &current_pc_extended_segment);


    virtual void _FindInconsistentPoints(const pcl::PointCloud<pcl::PointXYZ >::Ptr& pc_source,
                                            const cv::Mat & dm_true,
                                            pcl::PointIndicesPtr& indices_to_remove,
                                            pcl::PointIndicesPtr& indices_matching_in_true,
                                            pcl::PointIndicesPtr& indices_matching_in_dm,
                                            const double min_depth_error=0.03);

    ///Clone function
    virtual ShapeTracker* doClone() const
    {
        return (new ShapeTracker(*this));
    }

    omip::RB_id_t                                                       _rb_id;
    pcl::PointCloud<pcl::PointXYZ >::Ptr                             _rb_model;

    sensor_msgs::CameraInfo _ci;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;

    ros::NodeHandle _nh;
    ros::Publisher _segment_pub;

    bool _new_model;

    cv::Mat _current_dm;
    cv::Mat _segment_of_current;
    cv::Mat _segment_of_current_dilated;
    cv::Mat _dilation_element;

    pcl::ExtractIndices<pcl::PointXYZ>        _extractor;

#ifdef USE_LIBPOINTMATCHER_ICP
    boost::shared_ptr<DP> _cloud_lpm;
    PM::ICP* _icp_lpm;
    boost::shared_ptr<DP> _rb_model_lpm;
#endif
};
}

#endif /* SHAPE_TRACKER_H_ */

