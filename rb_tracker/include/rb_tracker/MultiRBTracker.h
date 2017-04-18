/*
 * MultiRBTracker.h
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

#ifndef MULTI_RB_TRACKER_H_
#define MULTI_RB_TRACKER_H_

#include <rb_tracker/RBTrackerDynReconfConfig.h>

#include "rb_tracker/RBFilter.h"
#include "rb_tracker/RBFilterCentralizedIntegrator.h"

#include "omip_common/FeaturesDataBase.h"
#include "omip_msgs/RigidBodyTwistWithCovMsg.h"

#include "omip_common/OMIPTypeDefs.h"

#include "omip_common/RecursiveEstimatorFilterInterface.h"

#include "omip_msgs/ShapeTrackerStates.h"

#include <tf/transform_listener.h>

#include "rb_tracker/StaticEnvironmentFilter.h"
#include "rb_tracker/EndEffectorFilter.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <fstream>

#define INTEGRATOR_VISION_CHANNEL 0
#define INTEGRATOR_PROPRIOCEPTION_CHANNEL 1

namespace omip
{

/**
 * Class MultiRBTracker
 * This class manages a set of RBFilters that tracks the rigid bodies in the environment
 * The MultiRBTracker:
 *      1- receives the 3D tracked Features and assigns them to support RBFilters
 *      2- steps existing RBFilters using the supporting Features
 *      3- creates new RBFilters if there are enough moving Features that do not support any RBFilter
 */
class MultiRBTracker : public RecursiveEstimatorFilterInterface<rbt_state_t, rbt_measurement_t>
{
public:

    /**
   * Constructor
   * @param ransac_iterations - Number of iterations of the RanSaC algorithm that creates new RBFilters from not assigned moving Features
   * @param estimation_error_threshold - Maximum error between predicted and measured Feature Location to consider
   * that this Feature still supports the RBFilter that generated the prediction
   * @param static_motion_threshold - Minimum motion from its initial Location to consider a Feature to be moving
   * @param new_rbm_error_threshold - Maximum error in the RanSaC algorithm for the creation of new RBFilters to consider
   * that a moving Feature is an inlier
   * @param max_error_to_reassign_feats - Maximum error between predicted and measured Feature Location to assign a Feature
   * to support the RBFilter that generated the prediction
   * @param supporting_features_threshold - Minimum number of supporting Features that a RBFilter has to have to remain alive
   * @param min_num_feats_for_new_rb - Minimum number of moving Features to trigger the creation of a new RBFilter
   * @param min_num_frames_for_new_rb - Minimum number of frames that a Feature has to be tracked to be used for the creation of
   * new RBFilters
   */
    MultiRBTracker(int max_num_rb,
                   double loop_period_ns,
                   int ransac_iterations,
                   double estimation_error_threshold,
                   double static_motion_threshold,
                   double new_rbm_error_threshold,
                   double max_error_to_reassign_feats,
                   int supporting_features_threshold,
                   int min_num_feats_for_new_rb,
                   int min_num_frames_for_new_rb,
                   int initial_cam_motion_constraint,
                   static_environment_tracker_t static_environment_tracker_type);

    virtual void Init();

    /**
   * Destructor
   */
    virtual ~MultiRBTracker();

    /**
   * @brief Set the latest acquired measurement
   *
   * @param acquired_measurement Latest acquired measurement
   */
    virtual void setMeasurement(rbt_measurement_t acquired_measurement, const double& measurement_timestamp);
    virtual void setMeasurementProprioception(const Eigen::Matrix4d& current_ee_pose, double time_pose_ns);

    /**
   * @brief First step when updating the filter. The next state is predicted from current state and system model (part of the RecursiveEstimatorInterace)
   * In the FeatureTracker the system model is the most simple one: no change in the state. So the predicted state is just the current state.
   *
   */
    virtual void predictState(double time_interval_ns);

    /**
   * @brief Second step when updating the filter. The next measurement is predicted from the predicted next state
   *
   */
    virtual void predictMeasurement();


    /**
   * @brief Get the predicted next measurement.
   *
   * @return rbt_measurement_t rbt_measurement_t Predicted next measurement based on the predicted next state
   */
    virtual rbt_measurement_t getPredictedMeasurement() const;

    /**
   * @brief Third and final step when updating the filter. The predicted next state is corrected based on the difference
   * between predicted and acquired measurement
   *    1- Estimate the supporting Features of each filter:
   *            1.1- Check if the previously supporting Features still support each filter
   *            1.2- Delete filters that do not have enough support
   *            1.3- Assign moving Features that do not support any filter yet
   *    2- Correct the predicted next state of each filter using the assigned Features and the Measurement Model
   */
    virtual void correctState();

    /**
   * @brief Get the currently belief state
   *
   * @return StateType Currently belief state
   */
    virtual rbt_state_t getState() const;

    /**
   * @brief Prepare the corrected state to be published
   */
    void ReflectState();

    /**
   * @brief Dynamic configuration
   *
   * @param config Value from Dynamic Reconfigure
   */
    virtual void setDynamicReconfigureValues(rb_tracker::RBTrackerDynReconfConfig &config);

    /**
     * @brief Process the measurement coming from the ShapeTrackerNode that is a refinement/substitute of the feature based tracking
     *
     * @param meas_from_st Measurement arriving from the shape-based tracker
     */
    virtual void processMeasurementFromShapeTracker(const omip_msgs::ShapeTrackerStates &meas_from_st);


    /**
   * Get tracked poses from each RBFilter
   * @return - Vector of poses of each tracked rigid body
   */
    std::vector<Eigen::Matrix4d> getPoses() const;

    /**
   * Get tracked poses from each RBFilter (translation vector + quaternion) and their covariances
   * @return - Vector of shared pointers to poses (translation vector + quaternion) and covariances of each tracked rigid body
   */
    std::vector<geometry_msgs::PoseWithCovariancePtr> getPosesWithCovariance() const;

    /**
   * Get tracked poses from each RBFilter (exponential coordinates) and their covariances
   * @return - Vector of shared pointers to poses (exponential coordinates) and covariances of each tracked rigid body
   */
    std::vector<geometry_msgs::TwistWithCovariancePtr> getPosesECWithCovariance() const;

    /**
   * Get tracked velocities from each RBFilter (twists) and their covariances
   * @return - Vector of shared pointers to velocities (twists) and covariances of each tracked rigid body
   */
    std::vector<geometry_msgs::TwistWithCovariancePtr> getVelocitiesWithCovariance() const;

    /**
   * Add the last tracked Location of a Feature to the data base
   * @param f_id - Id of the tracked Feature
   * @param f_loc - New Location of the tracked Feature
   */
    void addFeatureLocation(Feature::Id f_id, Feature::Location f_loc);

    /**
   * Get the unique Id of the RBFilter at a certain position in the list of alive Filters
   * @param n - Position in the list of alive Filters from where we want to obtain the Id
   * @return - Id of the RBFilter at that position in the list
   */
    RB_id_t getRBId(int n) const;

    /**
   * Get the number of supporting features of the RBFilter at a certain position in the list of alive Filters
   * @param n - Position in the list of alive Filters from where we want to obtain the Id
   * @return - Number of supporting features of the Filter
   */
    int getNumberSupportingFeatures(int n) const;

    /**
   * Get a point cloud with the last Location of the supporting Features labeled by the RBFilter
   * they support (kind of segmentation)
   * @return - Point cloud with the labeled supporting Features
   */
    FeatureCloudPCL getLabelledSupportingFeatures();

    FeatureCloudPCL getFreeFeatures();

    FeatureCloudPCL getPredictedFeatures();

    FeatureCloudPCL getAtBirthFeatures();


    /**
   * Get the centroids of each RBFilter as the centroid of the last Location of the Features
   * that support them. This centroids are used only for visualization purposes: they are used to
   * place the prismatic joints markers
   * @return - Vector with the 3D centroid of each RBFilter
   */
    std::vector<Eigen::Vector3d> getCentroids() const;

    /**
   * Get the shared pointer to the database of Features that can be used for both the FeatureTracker
   * and the next level in IP, the MultiRBTracker.
   * The FeatureTracker can directly update the Database of Features and it will be aware of the Feature Ids
   * which simplifies the use of predicted next Feature Locations
   * @return - Shared pointer to DataBase of Features
   */
    FeaturesDataBase::Ptr getFeaturesDatabase();

    /**
   * Pass a set of new predictions of the next pose and velocity of each rigid body, computed by the higher level (JointTracker)
   * Implements the feedback from higher inference levels (JointTracker)
   * @param twist_wc - Predictions about the next poses of the rigid bodies and their covariances
   */
    virtual void addPredictedState(const rbt_state_t& predicted_state, const double& predicted_state_timestamp_ns);

    /**
   * Collect the predictions about the next Feature Locations from each RBFilter
   * Implements the feedback to lower inference levels (FeatureTracker)
   * @return - Shared pointer to a point cloud with the predicted next Feature Locations
   */
    FeatureCloudPCL::Ptr CollectLocationPredictions();

    /**
   * Estimate the set of supporting Features of each Filter.
   * Right now it just checks that the supporting Features of each Filter STILL support it.
   * @return - Vector of Ids of the supporting Features
   */
    std::vector<Feature::Id> estimateBestPredictionsAndSupportingFeatures();

    /**
   * Estimate the set of free Features as the Features that are moving (enough motion from its initial Location)
   * but do not support any existing RBFilter
   * @param supporting_features - Vector of Ids of the features that support one RBFilter (not free)
   * @return - Vector of Ids of the free Features (= alive_features - supporting_features)
   */
    std::vector<Feature::Id> EstimateFreeFeatures(const std::vector<Feature::Id>& supporting_features);

    /**
   * Try to assign free Features (Features that are moving but do not support any existing RBFilter) to one of the
   * existing RBFilters
   * @param free_feat_ids - Vector of Ids of the free Features (= alive_features - supporting_features)
   * @return - Vector of Ids of the remaining free Features: Features that could not be reassigned to any existing Filter
   */
    std::vector<Feature::Id> ReassignFreeFeatures(const std::vector<Feature::Id>& free_feat_ids);

    /**
   * Try to create a new RBFilter from a set of free Features that could not be reassigned to any existing RBFilter
   * The process is:
   *    1- Take the free Features that are old enough (have been tracked at least during N frames)
   *    2- Take the last location and location N frames before of the old enough Features
   *    3- Run RanSaC on between these two location sets:
   *            3.1- Pick up randomly 4 pairs of last-NToLast Locations
   *            3.2- Estimate the homogeneous transform (HT) that explains the motion of these 4 Features
   *            3.3- Compute the inliers of this HT, other free Features that also moved according to
   *            this HT. To count as inlier the error between predicted location (resulting of applying the HT
   *            to the NToLast location) and measured Location (last Location) must be under a threshold (new_rbm_error_threshold)
   *            3.4- If enough Features are inliers of an HT we compute the HT that explains the motion of all inliers between
   *            last and NextToLast frames and create a new RBFilter using this HT as initial pose and initial velocity. We compute
   *            the HT between the last 2 frames so that the HTs are always at the same time steps.
   * @param really_free_feat_ids
   */
    void TryToCreateNewFilter(const std::vector<Feature::Id>& really_free_feat_ids);

    /**
   * Create a new RBFilter
   * @param initial_trajectory - Initial set of poses of the rigid body (exponential coordinates)
   * @param initial_velocity_guess - Initial velocity of the rigid body (twist)
   * @param initial_supporting_feats - Vector with the Ids of the Features that support the new RBFilter
   */
    void CreateNewFilter(const std::vector<Eigen::Matrix4d> &initial_trajectory,
                         const Eigen::Twistd& initial_velocity,
                         const std::vector<Feature::Id> &initial_supporting_feats);


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

    virtual void setMinNumPointsInSegment(int v)
    {
        this->_min_num_points_in_segment = v;
    }

    virtual void setMinProbabilisticValue(double v)
    {
        this->_min_probabilistic_value = v;
    }

    virtual void setMaxFitnessScore(double v)
    {
        this->_max_fitness_score = v;
    }

    virtual void setMinAmountTranslationForNewRB(double v)
    {
        this->_min_amount_translation_for_new_rb = v;
    }

    virtual void setMinAmountRotationForNewRB(double v)
    {
        this->_min_amount_rotation_for_new_rb = v;
    }

    virtual void setMinNumberOfSupportingFeaturesToCorrectPredictedState(int v)
    {
        this->_min_num_supporting_feats_to_correct = v;
    }

    virtual void setRobotInteraction(bool ri)
    {
        this->_robot_interaction = ri;
    }

    virtual void setInteractedRigidBodyMaxFeatureError(double v)
    {
        if(this->_interacted_rb_filter)
        {
            ROS_WARN_STREAM("New threshold for the error between predicted and measured feature locations for the interacted rigid body: " << v);
            this->_interacted_rb_filter->setEstimationThreshold(v);
        }else{
            ROS_ERROR("There is no interacted rigid body filter!");
        }
    }

    virtual void setEndEffectorEstimationThreshold(double v)
    {
        if(this->_end_effector_proprioception_filter)
        {
            ROS_WARN_STREAM("New threshold for the error between predicted and measured feature locations for the end-effector vision-based filter: " << v);
            this->_end_effector_proprioception_filter->setEstimationThreshold(v);
        }else{
            ROS_ERROR("There is no end effector vision-based filter!");
        }
    }

    virtual void setEndEffectorMaxDistanceFeaturesToBody(double max_distance_ee)
    {
        this->_max_distance_ee = max_distance_ee;
    }

    virtual void setInteractedRigidBodyMaxDistanceFeaturesToBody(double max_distance_irb)
    {
        this->_max_distance_irb = max_distance_irb;
    }

protected:    

    int _max_num_rb;
    double _prior_cov_pose;
    double _prior_cov_vel;
    double _cov_sys_acc_tx;
    double _cov_sys_acc_ty;
    double _cov_sys_acc_tz;
    double _cov_sys_acc_ry;
    double _cov_sys_acc_rx;
    double _cov_sys_acc_rz;

    int _min_num_points_in_segment;
    double _min_probabilistic_value;
    double _max_fitness_score;

    double _min_amount_translation_for_new_rb;
    double _min_amount_rotation_for_new_rb;


    std::vector<RBFilter::Ptr> _vision_based_kalman_filters;
    std::vector<RBFilter::Ptr> _proprioception_based_kalman_filters;
    std::vector<RBFilterCentralizedIntegrator::Ptr> _centralized_integrators;

    std::vector<RBFilter::Ptr> _output_kalman_filters;  //redundant!

    FeaturesDataBase::Ptr _features_db;

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

    // Minimum number of features that have to support a RBM to not be deleted
    int _supporting_features_threshold;

    // Minimum number of free features to trigger the generation of a new RBM
    int _min_num_feats_for_new_rb;

    // Minimum number of frames that a features has to be present to be used to create new RBM
    int _min_num_frames_for_new_rb;

    int _num_tracked_feats;

    int _min_num_supporting_feats_to_correct;

    StaticEnvironmentFilter::Ptr _static_environment_filter;

    rbt_state_t _last_predicted_state_kh;

    std::vector<Feature::Id> _free_feat_ids, _really_free_feat_ids;

    std::ofstream _really_free_feats_file;

    boost::mutex _measurement_timestamp_ns_mutex;
    boost::mutex::scoped_lock _measurement_timestamp_ns_lock;

    bool _robot_interaction;
    double _max_distance_ee;
    double _max_distance_irb;
    EndEffectorFilter::Ptr _end_effector_proprioception_filter;
    RBFilter::Ptr _end_effector_vision_filter;
    RBFilter::Ptr _interacted_rb_filter;
    RBFilter::Ptr _deformed_end_effector_filter;
    ros::NodeHandle _nh;
    tf::TransformListener _tf_listener;

    std::map<uint32_t, Eigen::Matrix3d> _feat_covs;
};
}

#endif /* MULTI_RB_TRACKER_H_ */
