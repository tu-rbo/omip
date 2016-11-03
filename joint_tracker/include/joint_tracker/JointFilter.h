/*
 * JointFilter.h
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

#ifndef JOINTFILTER_H_
#define JOINTFILTER_H_

#include <pcl/point_types.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Geometry>
#include <lgsm/Lgsm>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/MarkerArray.h>

#include <wrappers/matrix/matrix_wrapper.h>

#include "omip_msgs/RigidBodyTwistWithCovMsg.h"
#include "omip_common/OMIPTypeDefs.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define JOINT_AXIS_MARKER_RADIUS 0.01
#define JOINT_AXIS_AND_VARIABLE_MARKER_RADIUS 0.02
#define JOINT_VALUE_TEXT_SIZE 0.1

//#define PUBLISH_PREDICTED_POSE_AS_PWC 1

namespace omip
{

typedef int long JointCombinedFilterId;

enum JointFilterType
{
    RIGID_JOINT,
    PRISMATIC_JOINT,
    REVOLUTE_JOINT,
    DISCONNECTED_JOINT
};

class JointFilter;
typedef boost::shared_ptr<JointFilter> JointFilterPtr;

class JointFilter
{
public:

    /**
   * @brief Constructor
   */
    JointFilter();

    /**
   * @brief Destructor
   */
    virtual ~JointFilter();

    /**
   * @brief Creates a new Joint object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
    JointFilterPtr clone() const
    {
        return (JointFilterPtr(doClone()));
    }

    /**
   * @brief Copy constructor
   */
    JointFilter(const JointFilter &joint);

    virtual void initialize();

    /**
   * @brief Predict the next state
   *
   */
    virtual void predictState(double time_interval_ns){}

    /**
   * @brief Generate a prediction about the pose-twist of the second rigid body (SRB) using the frame of the reference rigid body (RRBF) as observation and ref frame
   * based on the belief state (joint parameters and latent variable) and the initial pose of the SRB in the RRBF
   * Implemented for prismatic, revolute and rigid joints, but not for disconnected (there is no way to predict the next measurement because there is no constraint in the
   * motion if the joint is disconnected)
   *
   */
    virtual void predictMeasurement(){}

    /**
   * @brief Correct the state of the filter (the joint parameters) based on the difference between the predicted and the last acquired measurement
   * It is only implemented for revolute and prismatic because rigid and disconnected have no parameters to correct
   *
   */
    virtual void correctState(){}

    /**
   * @brief Set the latest acquired measurement
   *
   * @param acquired_measurement Latest acquired measurement
   */
    virtual void setMeasurement(joint_measurement_t acquired_measurement, const double& measurement_timestamp_ns);

    virtual void setInitialMeasurement(const joint_measurement_t &initial_measurement,
                                       const Eigen::Twistd& rrb_pose_at_srb_birth_in_sf,
                                       const Eigen::Twistd& srb_pose_at_srb_birth_in_sf);

    /**
   * @brief Get the predicted pose-twist of the second rigid body (SRB) using the frame of the reference rigid body (RRBF) as observation and ref frame
   *
   * @return Eigen::Twistd Predicted pose-twist of the second rigid body (SRB) using the frame of the reference rigid body (RRBF) as observation and ref frame
   */
    virtual Eigen::Twistd getPredictedMeasurement() const;

    /**
   * @brief Estimate the error of the last joint parameters by testing them against the full observed trajectory
   * of the second RB relative to the reference RB -> Obtain the likelihood of the measurements given the model and the model parameters
   * Implemented for prismatic, revolute and rigid joints, but not for disconnected (there is no way to estimate its likelihood, we use a value that is a threshold)
   */
    virtual void estimateMeasurementHistoryLikelihood() {}

    virtual void estimateUnnormalizedModelProbability() {}

    /**
   * @brief Generate a prediction about the pose-twist of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
   * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
   */
    virtual geometry_msgs::TwistWithCovariance getPredictedSRBPoseWithCovInSensorFrame() = 0;

    /**
   * @brief Generate a prediction about the change in pose of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
   * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
   *
   * @return geometry_msgs::TwistWithCovariance Change in pose of the second rigid body in form of a twist with covariance based on the model uncertainty
   */
    virtual geometry_msgs::TwistWithCovariance getPredictedSRBDeltaPoseWithCovInSensorFrame() =0;

    /**
   * @brief Generate a prediction about the velocity of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
   * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
   *
   * @return geometry_msgs::TwistWithCovariance Velocity of the second rigid body in form of a twist with covariance based on the model uncertainty
   */
    virtual geometry_msgs::TwistWithCovariance getPredictedSRBVelocityWithCovInSensorFrame() =0;

    /**
   * @brief Set the normalizing term from the combined filter as the sum of all unnormalized probabilities
   *
   * @param normalizing_term Sum of all unnormalized probabilities
   */
    virtual void setNormalizingTerm(double normalizing_term);

    /**
   * @brief Set the prior probability of this type of model/joint. We could use this in the feature if the robot is in environments
   * where is more probable to encounter joints of certain type
   *
   * @param model_prior_probability Prior probability of this type of model/joint
   */
    virtual void setModelPriorProbability(double model_prior_probability);

    /**
   * @brief Get the prior probability of this type of model/joint. We could use this in the feature if the robot is in environments
   * where is more probable to encounter joints of certain type
   *
   * @return double The prior probability of this type of model/joint
   */
    virtual double getModelPriorProbability() const;

    /**
   * @brief Return the estimated error of the last joint parameters by testing them against the full observed trajectory
   *
   * @return double Likelihood of the last relative trajectory of the SRB wrt the RRB given the model and the model parameters
   */
    virtual double getLikelihoodOfLastMeasurements() const;

    /**
   * @brief Set the estimated error of the last joint parameters (only useful for the disconnected joint because we do not estimate the likelihood internally)
   *
   * @param likelihood Likelihood of the last relative trajectory of the SRB wrt the RRB given the model and the model parameters
   */
    virtual void setLikelihoodOfLastMeasurements(double likelihood);

    /**
   * @brief Return the unnormalized joint filter probability as p(Data|Model) x p(Model)
   *
   * @return double Unnormalized probability of this joint filter
   */
    virtual double getUnnormalizedProbabilityOfJointFilter() const;

    /**
   * @brief Return the normalized joint filter probability as p(Model|Data) = p(Data|Model) x p(Model) / p(Data)
   * where SUM_all_models( p(Model|Data) ) = 1
   *
   * @return double Normalized probability of this joint filter
   */
    virtual double getProbabilityOfJointFilter() const;

    /**
   * @brief Get the joint type as one of the possible JointFilterTypes
   *
   * @return JointFilterType The type of this joint filter
   */
    virtual JointFilterType getJointFilterType() const = 0;

    /**
   * @brief Get the joint type as a string (printing purposes)
   *
   * @return std::string The type of this joint filter as a string
   */
    virtual std::string getJointFilterTypeStr() const = 0;

    /**
   * Return rviz markers that show the type and parameters of the estimated joint
   */
    virtual std::vector<visualization_msgs::Marker> getJointMarkersInRRBFrame() const = 0;

    virtual Eigen::Vector3d getJointPositionInRRBFrame() const;

    virtual Eigen::Vector3d getJointOrientationRPYInRRBFrame() const;

    virtual Eigen::Vector3d getJointOrientationUnitaryVector() const;

    /**
   * @brief Get the unique Id that identifies this joint
   *
   * @return longint Joint id of this joint
   */
    virtual JointCombinedFilterId getJointId() const;

    virtual void setJointId(JointCombinedFilterId joint_id);

    virtual double getOrientationPhiInRRBFrame() const;

    virtual double getOrientationThetaInRRBFrame() const;

    virtual double getCovarianceOrientationPhiPhiInRRBFrame() const;

    virtual double getCovarianceOrientationThetaThetaInRRBFrame() const;

    virtual double getCovarianceOrientationPhiThetaInRRBFrame() const;

    virtual void setLoopPeriodNS(double loop_period_ns);

    virtual double getLoopPeriodNS() const;

    virtual int getNumSamplesForLikelihoodEstimation() const;

    virtual void setNumSamplesForLikelihoodEstimation(int likelihood_sample_num);

    virtual double getNormalizingTerm();

    virtual void setCovariancePrior(double  prior_cov_vel);

    virtual void setCovarianceAdditiveSystemNoisePhi(double  sigma_sys_noise_phi);

    virtual void setCovarianceAdditiveSystemNoiseTheta(double  sigma_sys_noise_theta);

    virtual void setCovarianceAdditiveSystemNoisePx(double  sigma_sys_noise_px);

    virtual void setCovarianceAdditiveSystemNoisePy(double  sigma_sys_noise_py);

    virtual void setCovarianceAdditiveSystemNoisePz(double  sigma_sys_noise_pz);

    virtual void setCovarianceAdditiveSystemNoiseJointState(double  sigma_sys_noise_pv);

    virtual void setCovarianceAdditiveSystemNoiseJointVelocity(double  sigma_sys_noise_pvd);

    virtual void setCovarianceMeasurementNoise(double  sigma_meas_noise);

    /**
   * @brief Return the joint state. Useful for plotting and testing purposes (monitoring action)
   *
   * @return double Currently estimated joint state
   */
    virtual double getJointState() const;

    /**
   * @brief Return the joint velocity. Useful for plotting and testing purposes (monitoring action)
   *
   * @return double Currently estimated joint velocity
   */
    virtual double getJointVelocity() const;

protected:

    /**
   * @brief Initialize all variables of the filter
   *
   * @param joint_id Id of the joint
   */
    void _initVariables(JointCombinedFilterId joint_id);

    JointCombinedFilterId _joint_id;

    // Number of past measurements to use to estimate the likelihood of the current belief for the joint parameters
    int _likelihood_sample_num;

    //Using the Bayes Rule, the probability of a model given the data is the likelihood of the data given the model,
    //times the (prior) probability of the model, divided by the probability of the data:
    //p(Model|Data) = p(Data|Model) x p(Model) / p(Data)
    //p(Data) is just a regularization term. We can ignore it and say that p(Model|Data) is proportional to... At the end, we can renormalize with the assumption that
    //SUM_all_models( p(Model|Data) ) = 1
    //p(Model) is the prior probability of a certain model. We usually assume an uniform distribution, so that p(Model) = 1/(NumDifferentModels), but we can set them differently,
    //checking that SUM_all_models(p(Model)) = 1
    //Finally, the probability of a model is proportional to the likelihood of the data given that model

    double _measurements_likelihood;
    bool _externally_set_likelihood;
    double _model_prior_probability;
    double _normalizing_term;

    double _unnormalized_model_probability;

    // Position of the joint axis
    Eigen::Vector3d _joint_position;
    // Uncertainty about the position of the joint axis
    Eigen::Matrix3d _uncertainty_joint_position;

    // Orientation of the joint axis in spherical coordinates (phi,theta). Radius is always one (unitary vector is a point on the unit sphere)
    double _joint_orientation_phi;
    double _joint_orientation_theta;
    // Equivalent joint orientation as unitary vector
    Eigen::Vector3d _joint_orientation;
    // Uncertainty about the orientation of the joint axis
    Eigen::Matrix2d _uncertainty_joint_orientation_phitheta;

    // Joint state (variable)
    double _joint_state;
    // Uncertainty about the joint state
    double _uncertainty_joint_state;

    // Memory of all joint states so far
    std::vector<double> _joint_states_all;

    // Joint velocity (variable)
    double _joint_velocity;
    // Uncertainty about the joint velocity
    double _uncertainty_joint_velocity;

    // Not used!
    // Joint velocity (variable)
    double _joint_acceleration;
    // Uncertainty about the joint velocity
    double _uncertainty_joint_acceleration;

    Eigen::Vector3d _rrb_centroid_in_sf;
    Eigen::Vector3d _srb_centroid_in_sf;

    // System noise
    double _prior_cov_vel;
    double _sigma_sys_noise_phi;
    double _sigma_sys_noise_theta;
    double _sigma_sys_noise_px;
    double _sigma_sys_noise_py;
    double _sigma_sys_noise_pz;
    double _sigma_sys_noise_jv;
    double _sigma_sys_noise_jvd;
    double _sigma_meas_noise;
    /////////////

    // "A Twist does not have an absolutely fixed reference frame: you use it to represent the velocity of one frame with respect to another frame,
    // expressed with respect to still another THIRD frame"
    // Twists need to define:
    //    - Tracking frame: the frame that is tracked. In our case is the second rigid body rb2
    //    - Observation frame: the frame from which the motion is observed (do you see the car from the street or from a moving train?). In our case is the frame of
    //      the reference rigid body rrb
    //    - Reference frame: the reference frame in which to EXPRESS the twist. Usually is the same as the observation frame. In our case is the frame of the reference rigid body rrb
    //    - Reference point: the reference point with which to express the twist. Usually is the origin of the tracking frame. In our case is the origin of the second rigid body rb2
    //    - Reference point frame: the frame in which the reference point is expressed. Usually is the tracking frame. In our case is the frame of the second rigid body rb2
    // All these twists should use the frame of the reference rigid body as:
    // - Observation frame AND
    // - Reference frame
    // However, they are provided using as the sensor frame as:
    // - Observation frame AND
    // - Reference frame

    // Twists relative to the sensor frame
    Eigen::Twistd _rrb_current_pose_in_sf;
    Eigen::Twistd _rrb_previous_pose_in_sf;
    Eigen::Matrix<double, 6, 6> _rrb_pose_cov_in_sf;
    Eigen::Twistd _rrb_current_vel_in_sf;
    Eigen::Matrix<double, 6, 6> _rrb_vel_cov_in_sf;

    Eigen::Twistd _srb_current_pose_in_sf;
    Eigen::Matrix<double, 6, 6> _srb_pose_cov_in_sf;
    Eigen::Twistd _srb_current_vel_in_sf;

    // Twists relative to the reference rigid body frame (rrbf)
    Eigen::Twistd _srb_initial_pose_in_rrbf;
    Eigen::Matrix<double, 6, 6> _srb_initial_pose_cov_in_rrbf;
    Eigen::Twistd _srb_current_pose_in_rrbf;
    Eigen::Matrix<double, 6, 6> _srb_current_pose_cov_in_rrbf;

    Eigen::Twistd _srb_previous_pose_in_rrbf;
    Eigen::Twistd _srb_predicted_pose_in_rrbf;
    Eigen::Twistd _srb_previous_predicted_pose_in_rrbf;
    Eigen::Twistd _current_delta_pose_in_rrbf;
    Eigen::Matrix<double, 6, 6> _current_delta_pose_cov_in_rrbf;
    Eigen::Twistd _previous_delta_pose_in_rrbf;
    Eigen::Twistd _predicted_delta_pose_in_rrbf;
    std::vector<Eigen::Twistd> _delta_poses_in_rrbf;

    virtual JointFilter* doClone() const = 0;

    double _loop_period_ns;

#ifdef PUBLISH_PREDICTED_POSE_AS_PWC
    // This is used to visualize the predictions based on the joint hypothesis
    ros::NodeHandle _predicted_next_pose_nh;
    ros::Publisher _predicted_next_pose_publisher;
#endif

    tf::TransformBroadcaster _tf_pub;

    double _measurement_timestamp_ns;

    int _rrb_id;

    // To have a memory of the turns in the revolute joint
    bool _inverted_delta_srb_pose_in_rrbf;
    bool _from_inverted_to_non_inverted;
    bool _from_non_inverted_to_inverted;
};

}

#endif /* JOINTFILTER_H_ */
