/*
 * StaticEnvironmentFilter.h
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

#ifndef STATIC_ENVIRONMENT_FILTER_H_
#define STATIC_ENVIRONMENT_FILTER_H_

#include "rb_tracker/RBFilter.h"
#include "omip_common/OMIPTypeDefs.h"
#include <pcl/registration/transformation_estimation_svd.h>
#include <tf/transform_listener.h>

namespace omip
{

typedef enum
{
    NO_CONSTRAINED = 0,
    NO_ROLL_PITCH = 1,
    NO_ROLL_PITCH_TZ = 2,
    NO_TRANSLATION = 3,
    NO_TRANSLATION_ROLL_YAW = 4,
    NO_ROTATION = 5,
    NO_MOTION = 6,
    ROBOT_XY_BASELINK_PLANE = 7
}
MotionConstraint;

/**
 * Class StaticEnvironmentICPFilter
 * Represents an special type of RBFilter that tracks the motion of the static environment
 * wrt to the camera
 * Features of the environment that are not moving in the environment support this filter
 * and should be used to estimate the motion of the environment wrt to the camera
 */
class StaticEnvironmentFilter : public RBFilter
{
public:

    // Shared pointer type
    typedef boost::shared_ptr<StaticEnvironmentFilter> Ptr;

    /**
   * Constructor
   * @param static_motion_threshold - Threshold to detect Features that move (do not support the static rigid body any longer)
   */
    StaticEnvironmentFilter(double loop_period_ns, FeaturesDataBase::Ptr feats_database, double environment_motion_threshold);

    /**
   * Destructor
   */
    virtual ~StaticEnvironmentFilter(){}

    /**
   * Predict
   * Generate a prediction of the next state based on the previous state
   * For the StaticRB the state (pose and velocity) is always zero
   */
    virtual void predictState(double time_interval_ns);

    /**
   * Correct
   * Correct the predicted next state based on the supporting Features
   * For the StaticRB the state (pose and velocity) is always zero
   */
    virtual void correctState();

    /**
     * Add the ID of a feature to the list of supporting features of this RB
     * This function is called when a new feature is added to the list of tracked features
     * We create a prediction about its next position (the same it has now) to be used in the support
     * @param supporting_feat_id - ID of the Feature that now supports this RB
     */
    virtual void addSupportingFeature(Feature::Id supporting_feat_id);

    /**
     * @brief Set a constraint for the motion of the RB
     * Right now this is only used by the StaticEnvironmentFilter
     *
     * @param motion_constraint Value of the motion constraint. The meaning of each value is in motion_estimation.h
     */
    virtual void setMotionConstraint(int motion_constraint);

    /**
     * @brief Estimate the motion between the last 2 frames of the set of supporting features, assuming they are
     * all on the static environment
     *
     * @param supporting_features_ids IDs of the features that support the static environment
     * @param previous_current_Tf Resulting transformation as a tf transform. It is the transformation of the static environment from the
     * previous to the current frame
     */
    virtual void estimateDeltaMotion(std::vector<Feature::Id>& supporting_features_ids, tf::Transform& previous_current_Tf);

    /**
     * @brief Estimate the transformation between 2 sets of feature locations by iteratively estimating ICP until either the maximum
     * number of iterations is reached or the iterations do not change the transformation (convergence)
     *
     * @param previous_locations Point cloud with the previous locations of the features
     * @param current_locations Point cloud with the current locations of the features
     * @param previous_current_Tf Resulting transformation as a tf transform. It is the transformation of the static environment from the
     * previous to the current frame
     */
    virtual void iterativeICP( pcl::PointCloud<pcl::PointXYZ>& previous_locations,
                                                 pcl::PointCloud<pcl::PointXYZ>& current_locations, tf::Transform& previous_current_Tf);

    /**
     * @brief Function that constrains the last estimated pose of the static environment to the camera according to the motion constraint
     *
     */
    virtual void constrainMotion();

    /**
     * @brief Set the computation type for the static environment
     * Options: ICP based (t-1 to t) or EKF based (similar to any other RB)
     *
     * @param computation_type Value of the type of computation (defined in omip_common/OMIPTypeDefs.h)
     */
    virtual void setComputationType(static_environment_tracker_t computation_type);

    virtual void Init();

private:
    // Minimum motion to consider that a feature moves
    double _environment_motion_threshold;
    int _motion_constraint;

    int _max_iterations;
    double _tf_epsilon_linear;
    double _tf_epsilon_angular;

    ros::NodeHandle _nh;
    tf::TransformListener _tf_listener;

    static_environment_tracker_t _computation_type;

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> _svdecomposer;
};
}

#endif /* STATIC_ENVIRONMENT_FILTER_H_ */
