/*
 * RBFilter.h
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

#ifndef RB_FILTER_H_
#define RB_FILTER_H_

#include <rb_tracker/RBTrackerDynReconfConfig.h>

#include <pcl/point_types.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <lgsm/Lgsm>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf_conversions/tf_eigen.h>

#include "omip_common/OMIPTypeDefs.h"
#include "omip_common/Feature.h"
#include "omip_common/FeaturesDataBase.h"

// Dimensions of the system state: 6dof of rbpose and 6dof of rbvelocity
#define STATE_DIM_RBF 6

// Dimensions of each measurement (each feature location)
#define MEAS_DIM_RBF 3

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include "omip_common/OMIPUtils.h"


/**
 * NOTE:
 * The purpose of this program is to construct a kalman filter (EKF) for the problem
 * of tracking a rigid body from a varying set of 3d tracked points.
 *
 * We maintain in parallel two linear SYSTEM MODELS (leading to two EKFs) to update the state based on the previous step
 * (and an input signal that in our EKF is always 0)
 *    - One model supposes that the rigid body moves with the velocity estimated
 *    in the previous step (pose'=pose+velocity)
 *    pose_k = pose_{k-1} + v_{k-1} delta_t
 *    - One model supposes that the rigid body stops abruptly (p'=p)
 *    pose_k = pose_{k-1}
 *
 *  The MEASUREMENT MODEL predicts the next measurement based on the next state predicted by the SYSTEM MODEL.
 *  The MEASUREMENT MODEL of our two parallel EKFs is:
 *   z = M*[(pose_k)*initial_point_location]
 */

namespace omip
{

/**
 * Class RBFilter
 * Represents one moving rigid body
 * Tracks the pose of this rigid body (state)
 * Estimates the Features that move coherently with the pose and velocity of this rigid body
 * and uses them to update the state
 */
class RBFilter
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Shared Pointer to a RBFilter
    typedef boost::shared_ptr<RBFilter> Ptr;

    // Trajectory of a RB
    typedef std::vector<Eigen::Matrix4d> Trajectory;

    enum PredictionHypothesis
    {
        BASED_ON_VELOCITY = 0,
        BASED_ON_BRAKING_EVENT = 1,
        BASED_ON_KINEMATICS = 2
    };

    /**
   * Default constructor
   */
    RBFilter();

    /**
   * Constructor
   * @param first_transform - Twist of the initial pose of the body in SF
   * @param initial_velocity - Twist of the initial velocity of the body in SF
   * @param feats_database - Shared pointer to the database of Features
   * @param estimation_error_threshold - Maximum error allowed between predicted and measured Feature
   * location to consider that it still supports this RBM
   */
    RBFilter(double loop_period_ns,
             const std::vector<Eigen::Matrix4d> &trajectory,
             const Eigen::Twistd& initial_velocity,
             FeaturesDataBase::Ptr feats_database,
             double estimation_error_threshold);

    virtual void Init();

    /**
   * Destructor
   */
    virtual ~RBFilter();

    /**
   * Copy constructor
   * @param rbm - Object constant reference to be copied
   */
    RBFilter(const RBFilter &rbm);

    /**
   * Clones this RBFilter
   * @return - Shared pointer to the clone of this RBFilter
   */
    RBFilter::Ptr clone() const
    {
        return (RBFilter::Ptr(doClone()));
    }

    /**
   * Predict the next pose of the RB based on the current pose and the current velocity
   */
    virtual void predictState(double time_interval_ns = -1.);

    /**
   * @brief Second step when updating the filter. The next measurement is predicted from the predicted next state
   *
   */
    virtual void predictMeasurement();

    /**
   * Correct the predicted pose of the RB using the Features that support this RB
   */
    virtual void correctState();

    virtual void correctState(const RB_id_t& rrb_id, const Eigen::Matrix4d& other_rb_pose, const Eigen::Matrix<double, 6, 6>& other_rb_pose_cov);

    virtual void integrateShapeBasedPose(const geometry_msgs::TwistWithCovariance twist_refinement, double pose_time_elapsed_ns);

    /**
   * Estimate the Features that support this RB (Features that move with the motion of this RB)
   * There are two options: to test all Features or to test the previously supporting Features only (second option implemented)
   */
    virtual void estimateBestPredictionAndSupportingFeatures();

    /**
   * Add the ID of a feature to the list of supporting features of this RB
   * @param supporting_feat_id - ID of the Feature that now supports this RB
   */
    virtual void addSupportingFeature(Feature::Id supporting_feat_id);

    /**
   * Add a new prediction of the next pose and velocity of this rigid body, computed by the higher level (JointTracker)
   * Implements the feedback from higher inference levels (JointTracker)
   * @param twist_hyp - Prediction of the next rigid body pose and velocity (with covariance)
   */
    virtual void setPredictedState(omip_msgs::RigidBodyPoseAndVelMsg hypothesis);

    /**
   * @brief
   *
   * @param predicted_location_velocity
   * @param predicted_location_brake
   * @param feature_id
   */
    virtual void addPredictedFeatureLocation(const Feature::Location& predicted_location_velocity,
                                             const Feature::Location& predicted_location_brake, const Feature::Id feature_id);

    /**
   * Predict the location of a Feature for one of the two possible event hypotheses
   * @param feature - Feature whose location we want to predict using this RB
   * @param predicted_pose - Predicted pose as twist
   * @param contemporary_last_feat_loc_and_last_pose_in_trajectory - True if the last location and the last entry of the _trajectory vector are of the same time step
   * This is the case only at the end of the loop, after the correction step (when we predict the next feature location.
   * @return - Predicted location of the Feature
   */
    virtual void PredictFeatureLocation(Feature::Ptr feature,
                                        const Eigen::Matrix4d &predicted_pose,
                                        bool contemporary_last_feat_loc_and_last_pose_in_trajectory,
                                        Feature::Location& predicted_location, bool publish_locations=false) const;

    /**
     * @brief Get the location of the feature at the frame when the RB was detected first. If the feature is "younger" we use the previous motion of the
     * RB to estimate what would have been the location of the feature at the initial frame
     *
     * @param feature - Feature whose intial location we want to know
     * @param contemporary_last_feat_loc_and_last_pose_in_trajectory - True if the last location and the last entry of the _trajectory vector are of the same time step
     * This is the case only at the end of the loop, after the correction step (when we predict the next feature location.
     * @param location_at_birth - Location of the Feature at the first frame that the body was tracked
     */
    virtual void GetLocationOfFeatureAtBirthOfRB(Feature::Ptr feature,
                                                 bool contemporary_last_feat_loc_and_last_pose_in_trajectory,
                                                 Feature::Location& location_at_birth) const;

    /**
   * Set the Feature data base to be used by this RB
   * @param feats_database - Shared pointer to the Feature data base
   */
    virtual void setFeaturesDatabase(FeaturesDataBase::Ptr feats_database);

    /**
   * Get the ID of this RB
   * @return - ID of this RB
   */
    virtual RB_id_t getId() const;

    /**
   * Get the current believed pose in homogeneous matrix representation
   * @return - Eigen matrix containing the homogeneous matrix of the current believed pose
   */
    virtual Eigen::Matrix4d getPose() const;

    virtual void setPose(const Eigen::Matrix4d& new_pose)
    {
        _pose = new_pose;

        _trajectory.pop_back();

        Eigen::Matrix4d delta_pose = _pose*_trajectory.at(_trajectory.size()-1).inverse();

        Eigen::Twistd delta_pose_ec;
        TransformMatrix2Twist(delta_pose, delta_pose_ec);
        this->_velocity = delta_pose_ec/(_last_time_interval_ns/1e9);

        _trajectory.push_back(_pose);
    }

    /**
   * Get the covariance of the current believed pose
   * @param hypothesis - hypothesis from which we want to get the covariance of the believed pose: velocity-update or brake-event update
   * @return - Eigen matrix containing the covariance of the current believed pose
   */
    virtual Eigen::Matrix<double, 6, 6> getPoseCovariance() const;

    virtual void setPoseCovariance(const Eigen::Matrix<double, 6, 6>& new_pose_covariance)
    {
        _pose_covariance = new_pose_covariance;
    }

    /**
   * Get the current believed velocity in exponential coordinates
   * @param hypothesis - hypothesis from which we want to get the believed velocity: velocity-update or brake-event update (the later with be zero)
   * @return - Eigen twist containing the exponential coordinates of the current believed velocity
   */
    virtual Eigen::Twistd getVelocity() const;

    /**
   * Get the covariance of the current believed velocity
   * @param hypothesis - hypothesis from which we want to get the covariance of the believed velocity: velocity-update or brake-event update
   * @return - Eigen matrix containing the covariance of the current believed velocity
   */
    virtual Eigen::Matrix<double, 6, 6> getVelocityCovariance() const;

    /**
   * Get the current believed pose and its covariance (6x6) as a shared pointer to a PoseWithCovariance ROS message
   * @return - Shared pointer to a ROS message containing the believed pose and its covariance
   */
    virtual geometry_msgs::PoseWithCovariancePtr getPoseWithCovariance() const;

    /**
   * Get the current believed pose in exponential coordinates and its covariance (6x6) as a shared pointer to a TwistWithCovariance ROS message
   * @return - Shared pointer to a ROS message containing the believed pose and its covariance
   */
    virtual geometry_msgs::TwistWithCovariancePtr getPoseECWithCovariance() const;

    /**
   * Get the current believed velocity and its covariance (6x6) as a shared pointer to a TwistWithCovariance ROS message
   * @param hypothesis - hypothesis from which we want to get the believed twist: velocity-update or brake-event update
   * @return - Shared pointer to a ROS message containing the believed twist and its covariance
   */
    virtual geometry_msgs::TwistWithCovariancePtr getVelocityWithCovariance() const;

    /**
   * Get the temporal series of poses of this RB
   * @return - Vector of twists of the poses of this RB over time
   */
    virtual std::vector<Eigen::Matrix4d> getTrajectory() const;

    /**
   * Get a point cloud with the predicted next 3D locations of the supporting Features of this RB
   * supposing the RB moves with the tracked velocity
   * Implements the feedback to lower inference levels (FeatureTracker)
   * @return - Point cloud with the 3D predicted locations of the supporting Features
   */
    virtual FeatureCloudPCLwc getPredictedMeasurement(PredictionHypothesis hypothesis = BASED_ON_VELOCITY);

    /**
   * Get the IDs of the current Features support this RB
   * @return - Vector of IDs of the Features that support this RB
   */
    virtual std::vector<Feature::Id> getSupportingFeatures() const;

    /**
   * Get the number of the current Features support this RB
   * @return - Number of the Features that support this RB
   */
    virtual int getNumberSupportingFeatures() const;

    /**
   * Get the Feature data base used by this RB
   * @return - Shared pointer ot the Feature data base
   */
    virtual FeaturesDataBase::Ptr getFeaturesDatabase() const;

    /**
   * @brief Set a constraint for the motion of the RB
   * Right now this is only used by the StaticEnvironmentFilter
   *
   * @param motion_constraint Value of the motion constraint. The meaning of each value is in motion_estimation.h
   */
    virtual void setMotionConstraint(int motion_constraint){}

    virtual void setPriorCovariancePose(double v)
    {
        this->_prior_cov_pose = v;
    }

    virtual void setPriorCovarianceVelocity(double v)
    {
        this->_prior_cov_vel = v;
    }

    virtual void setCovarianceSystemAccelerationTx(double v)
    {
        this->_cov_sys_acc_tx = v;
    }

    virtual void setCovarianceSystemAccelerationTy(double v)
    {
        this->_cov_sys_acc_ty = v;
    }

    virtual void setCovarianceSystemAccelerationTz(double v)
    {
        this->_cov_sys_acc_tz = v;
    }

    virtual void setCovarianceSystemAccelerationRx(double v)
    {
        this->_cov_sys_acc_rx = v;
    }

    virtual void setCovarianceSystemAccelerationRy(double v)
    {
        this->_cov_sys_acc_ry = v;
    }

    virtual void setCovarianceSystemAccelerationRz(double v)
    {
        this->_cov_sys_acc_rz = v;
    }

    virtual void setNumberOfTrackedFeatures(int v)
    {
        this->_num_tracked_feats = v;
    }

    virtual void setMinNumberOfSupportingFeaturesToCorrectPredictedState(int v)
    {
        this->_min_num_supporting_feats_to_correct = v;
    }

    virtual void setEstimationThreshold(double v)
    {
        this->_estimation_error_threshold = v;
    }

    virtual Feature::Location getIntialLocationOfCentroid() const
    {
        return _initial_location_of_centroid;
    }

    virtual void clearPredictionFromHigherLevel()
    {
        _use_predicted_state_from_kh = false;
    }

    FeatureCloudPCLwc::Ptr getFeaturesAtBirth()
    {
        return _features_at_birth;
    }

    FeatureCloudPCLwc::Ptr getFeaturesPredicted()
    {
        return _features_predicted;
    }

    void resetFeaturesAtBirth()
    {
        _features_at_birth->points.clear();
    }

    void resetFeaturesPredicted()
    {
        _features_predicted->points.clear();
    }

    void setPointerFeatCovsMap(std::map<uint32_t, Eigen::Matrix3d>* feat_covs_ptr)
    {
        _feat_covs_ptr = feat_covs_ptr;
    }

protected:

    /**
   * NOTE:
   * The system model predicts the next state based on the previous step
   * We maintain in parallel two system models:
   *    - One model supposes that the rigid body moves with constant velocity
   *    - One model supposes that the rigid body stops abruptly (p'=p)
   */

    /**
   * Initialize the auxiliar matrices that are needed in the EKF computation
   */
    virtual void _initializeAuxiliarMatrices();

    // Maximum error allowed between predicted and measured feature position to consider it to STILL
    // support a RBM
    double _estimation_error_threshold;

    std::map<Feature::Id, double> _supporting_feats_errors;

    double _prior_cov_pose;
    double _prior_cov_vel;
    double _cov_sys_acc_tx;
    double _cov_sys_acc_ty;
    double _cov_sys_acc_tz;
    double _cov_sys_acc_ry;
    double _cov_sys_acc_rx;
    double _cov_sys_acc_rz;


    Eigen::MatrixXd _G_t_memory;
    Eigen::VectorXd _R_inv_times_innovation_memory;

    Eigen::Matrix<double, 3, 6> _D_T_p0_circle;

    static RB_id_t _rb_id_generator;

    RB_id_t _id;

    // Current pose of the rigid body
    Eigen::Matrix4d _pose;
    Eigen::Matrix<double, 6, 6> _pose_covariance;
    // First derivative of the pose
    Eigen::Twistd _velocity;
    Eigen::Matrix<double, 6, 6> _velocity_covariance;

    Eigen::Matrix4d _predicted_pose;
    Eigen::Matrix<double, 6, 6> _predicted_pose_covariance;
    // First derivative of the pose
    Eigen::Twistd _predicted_velocity;
    Eigen::Matrix<double, 6, 6> _predicted_velocity_covariance;

    Feature::Location _initial_location_of_centroid;
    // Sequence of Displacements of the RB
    std::vector<Eigen::Matrix4d> _trajectory;

    std::vector<Feature::Id> _supporting_features_ids;
    std::vector<double> _supporting_features_probs;

    FeaturesDataBase::Ptr _features_database;

    // Predicted poses in exponential coordinates
    Eigen::Matrix4d _predicted_pose_vh;   // Predicted pose based on the velocity hypothesis/model
    Eigen::Matrix4d _predicted_pose_bh;   // Predicted pose based on the brake-event hypothesis/model
    Eigen::Matrix4d _predicted_delta_pose_kh;   // Predicted change in pose based on the kinematic hypothesis/model
    Eigen::Matrix4d _predicted_pose_kh;   // Predicted pose based on the kinematic hypothesis/model

    Eigen::Matrix<double, 6, 6> _predicted_pose_cov_vh; // Predicted pose covariance based on the velocity hypothesis/model
    Eigen::Matrix<double, 6, 6> _predicted_pose_cov_bh; // Predicted pose covariance based on the brake hypothesis/model
    Eigen::Matrix<double, 6, 6> _predicted_pose_cov_kh; // Predicted pose covariance based on the kinematic hypothesis/model

    Eigen::Twistd _predicted_velocity_vh;   // Predicted pose based on the velocity hypothesis/model
    Eigen::Twistd _predicted_velocity_bh;   // Predicted pose based on the brake-event hypothesis/model
    Eigen::Twistd _predicted_velocity_kh;   // Predicted pose based on the kinematic hypothesis/model

    Eigen::Matrix<double, 6, 6> _predicted_velocity_cov_vh; // Predicted velocity covariance based on the velocity hypothesis/model
    Eigen::Matrix<double, 6, 6> _predicted_velocity_cov_bh; // Predicted velocity covariance based on the brake hypothesis/model
    Eigen::Matrix<double, 6, 6> _predicted_velocity_cov_kh; // Predicted velocity covariance based on the kinematic hypothesis/model

    bool _use_predicted_state_from_kh;
    bool _use_predicted_measurement_from_kh;

    FeatureCloudPCLwc::Ptr _features_predicted;
    FeatureCloudPCLwc::Ptr _features_at_birth;

    FeatureCloudPCLwc::Ptr _predicted_measurement_pc_vh;
    FeatureCloudPCLwc::Ptr _predicted_measurement_pc_bh;
    FeatureCloudPCLwc::Ptr _predicted_measurement_pc_kh;

    FeatureCloudPCLwc::Ptr _predicted_measurement;

    std::map<Feature::Id, Feature::Location> _predicted_measurement_map_vh;
    std::map<Feature::Id, Feature::Location> _predicted_measurement_map_bh;
    std::map<Feature::Id, Feature::Location> _predicted_measurement_map_kh;

    std::map<Feature::Id, Feature::Location> _predicted_measurement_map;

    double _loop_period_ns;
    double _last_time_interval_ns;

    virtual RBFilter* doClone() const
    {
        return (new RBFilter(*this));
    }

    int _num_tracked_feats;
    int _min_num_supporting_feats_to_correct;

    std::map<RB_id_t, Eigen::Matrix4d> _predicted_change_in_relative_pose_in_rrbf;
    std::map<RB_id_t, Eigen::Matrix<double, 6, 6> > _predicted_change_in_relative_pose_cov_in_rrbf;

    std::map<uint32_t, Eigen::Matrix3d>* _feat_covs_ptr;
};

}

#endif /* RB_FILTER_H_ */
