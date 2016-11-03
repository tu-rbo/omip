/*
 * JointCombinedFilter.h
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

#ifndef JOINTCOMBINEDFILTER_H_
#define JOINTCOMBINEDFILTER_H_

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

#include "joint_tracker/JointFilter.h"
#include "omip_common/OMIPTypeDefs.h"

namespace omip
{

class JointCombinedFilter;
typedef boost::shared_ptr<JointCombinedFilter> JointCombinedFilterPtr;

typedef std::map<JointFilterType, JointFilterPtr> joint_filters_map;

/**
 * Class JointCombinedFilter
 * It contains one of instance of each joint filter type and updates them with each acquired measurement
 * The measurements are RB poses: one is the reference RB and the other is the called "Second RB"
 * Each joint filter type defines a different motion constraint
 * Each joint filter type tracks a different set of parameters (joint parameters)
 * Each joint filter type generates a measurement prediction (pose of one of the rigid bodies given the motion of the reference body)
 */
class JointCombinedFilter
{
public:

  /**
   * Default Constructor
   */
    JointCombinedFilter();

  void setJointCombinedFilterId(JointCombinedFilterId new_joint_combined_filter_id);

  /**
   * Destructor
   */
  virtual ~JointCombinedFilter();

  /**
   * Creates a new JointCombinedFilter object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
  JointCombinedFilterPtr clone() const
  {
    return (JointCombinedFilterPtr(doClone()));
  }

  /**
   * Copy constructor
   */
  JointCombinedFilter(const JointCombinedFilter &joint_combined_filter);

  /**
   * @brief First step when updating the filter. The next state is predicted from current state and system model
   *
   */
  virtual void predictState(double time_interval_ns);

  /**
   * Generate a hypothesis about the pose of the second rigid body based on the pose of the
   * reference rigid body and the internal state (joint parameters and latent variable)
   */
  virtual void predictMeasurement();

  /**
   * Refine the parameters of the joint based on the difference between predicted and measured
   * pose of the second rigid body
   */
  virtual void correctState();

  /**
   * @brief Set the latest acquired measurement
   *
   * @param acquired_measurement Latest acquired measurement
   */
  virtual void setMeasurement(joint_measurement_t acquired_measurement, const double& measurement_timestamp_ns);

  /**
   * Generate a hypothesis about the pose of the second rigid body based on the pose of the
   * reference rigid body and the internal state (joint parameters and latent variable)
   */
  std::map<JointFilterType, Eigen::Twistd> getPredictedMeasurement();

  virtual void estimateJointFilterProbabilities();

  /**
   * @brief Estimate the probability of each joint filter type.
   * NOTE: Using the Bayes Rule, the probability of a model given the data is the likelihood of the data given the model,
   * times the (prior) probability of the model, divided by the probability of the data:
   * p(Model|Data) = p(Data|Model) x p(Model) / p(Data)
   * p(Data) is just a regularization term. We can ignore it and say that p(Model|Data) is proportional to... At the end, we can renormalize with the assumption that
   * SUM_all_models( p(Model|Data) ) = 1
   * p(Model) is the prior probability of a certain model. We usually assume an uniform distribution, so that p(Model) = 1/(NumDifferentModels), but we can set them differently,
   * checking that SUM_all_models(p(Model)) = 1
   * Finally, the probability of a model is proportional to the likelihood of the data given that model
   *
   */
  virtual void normalizeJointFilterProbabilities();

  /**
   * Obtain a pointer to the most probable joint filter, given the likelihood of the last measurements for each joint filter and the prior
   * probability of each filter type
   */
  virtual JointFilterPtr getMostProbableJointFilter();

  /**
   * @brief Get the joint filter of the type given
   *
   * @param joint_type Type of the joint filter to get
   * @return JointFilterPtr Joint filter of the type given
   */
  virtual JointFilterPtr getJointFilter(JointFilterType joint_type) const;

  /**
   * Return the joint id
   */
  virtual JointCombinedFilterId getJointCombinedFilterId() const;

  virtual void setJointLikelihoodDisconnected(double disconnected_joint_likelihood);

  virtual void setLoopPeriodNS(double loop_period_ns);

  virtual void setNumSamplesForLikelihoodEstimation(int likelihood_sample_num);

  virtual void setNormalizingTerm(double normalizing_term);

  virtual void setCovarianceDeltaMeasurementLinear(double sigma_delta_meas_uncertainty_linear);

  virtual void setCovarianceDeltaMeasurementAngular(double sigma_delta_meas_uncertainty_angular);

  virtual void initialize();

  virtual void setMaxTranslationRigid(double rig_max_translation);

  virtual void setMaxRotationRigid(double rig_max_rotation);

  virtual void setCovariancePrior(const JointFilterType& joint_type, double prior_cov_vel);

  virtual void setCovarianceAdditiveSystemNoisePhi(const JointFilterType& joint_type, double sys_noise_phi);

  virtual void setCovarianceAdditiveSystemNoiseTheta(const JointFilterType& joint_type, double sys_noise_theta);

  virtual void setCovarianceAdditiveSystemNoisePx(const JointFilterType& joint_type, double sys_noise_px);

  virtual void setCovarianceAdditiveSystemNoisePy(const JointFilterType& joint_type, double sys_noise_py);

  virtual void setCovarianceAdditiveSystemNoisePz(const JointFilterType& joint_type, double sys_noise_pz);

  virtual void setCovarianceAdditiveSystemNoiseJointState(const JointFilterType& joint_type, double sys_noise_js);

  virtual void setCovarianceAdditiveSystemNoiseJointVelocity(const JointFilterType& joint_type, double sys_noise_jv);

  virtual void setCovarianceMeasurementNoise(const JointFilterType& joint_type, double meas_noise);

  virtual void setMinRotationRevolute(const double& value);

  virtual void setMaxRadiusDistanceRevolute(const double& value);

  virtual void setInitialMeasurement(const joint_measurement_t &initial_measurement,
                                          const Eigen::Twistd& rrb_pose_at_srb_birth_in_sf, const Eigen::Twistd& srb_pose_at_srb_birth_in_sf);

protected:

  static JointCombinedFilterId _joint_id_generator;

  JointCombinedFilterId _joint_id;

  double _normalizing_term;

  // Contains an instance of each joint type
  joint_filters_map _joint_filters;

  // Contains the probabilities of each joint type
  std::map<JointFilterType, double> _joint_normalized_prediction_errors;

  virtual JointCombinedFilter* doClone() const
  {
    return (new JointCombinedFilter(*this));
  }

};

}

#endif /* JOINT_H_ */
