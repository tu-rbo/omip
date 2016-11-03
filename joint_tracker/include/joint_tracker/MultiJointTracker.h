/*
 * MultiJointTracker.h
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

#ifndef JOINT_TRACKER_H_
#define JOINT_TRACKER_H_

#include "joint_tracker/JointCombinedFilter.h"

#include "omip_msgs/RigidBodyPosesAndVelsMsg.h"
#include "omip_msgs/RigidBodyPoseAndVelMsg.h"

#include "omip_msgs/RigidBodyTwistsWithCovMsg.h"

#include "omip_common/OMIPTypeDefs.h"

#include "omip_common/RecursiveEstimatorFilterInterface.h"

namespace omip
{

enum ks_analysis_t
{
    MOVING_BODIES_TO_STATIC_ENV = 1,
    BETWEEN_MOVING_BODIES = 2,
    FULL_ANALYSIS = 3
};

typedef std::map<std::pair<int, int>, JointCombinedFilterPtr> joint_combined_filters_map;

/**
 *
 */
class MultiJointTracker : public RecursiveEstimatorFilterInterface<ks_state_t, ks_measurement_t>
{
public:
  MultiJointTracker(double loop_period_ns, ks_analysis_t ks_analysis_type, double dj_ne);

  virtual ~MultiJointTracker();

  /**
   * @brief First step when updating the filter. The next state is predicted from current state and system model
   *
   */
  virtual void predictState(double time_interval_ns);

  /**
   * @brief Second step when updating the filter. The next measurement is predicted from the predicted next state
   *
   */
  virtual void predictMeasurement();

  /**
   * Set the last received poses of the RBs
   */
  virtual void setMeasurement(const ks_measurement_t & poses_and_vels, const double& measurement_timestamp_ns);

  /**
   * @brief Third and final step when updating the filter. The predicted next state is corrected based on the difference
   * between predicted and acquired measurement
   *
   */
  virtual void correctState();

  virtual void addPredictedState(const ks_state_t &predicted_state , const double& predicted_state_timestamp_ns)
  {
      std::cout << "Not implemented" << std::endl;
  }

  virtual void setNumSamplesLikelihoodEstimation(int likelihood_sample_num)
  {
      this->_likelihood_sample_num = likelihood_sample_num;
  }

  virtual void setSigmaDeltaMeasurementUncertaintyLinear(double sigma_delta_meas_uncertainty_linear)
  {
      this->_sigma_delta_meas_uncertainty_linear = sigma_delta_meas_uncertainty_linear;
  }

  virtual void setSigmaDeltaMeasurementUncertaintyAngular(double sigma_delta_meas_uncertainty_angular)
  {
      this->_sigma_delta_meas_uncertainty_angular = sigma_delta_meas_uncertainty_angular;
  }

  virtual void setPrismaticPriorCovarianceVelocity(const double& value)
  {
      this->_prism_prior_cov_vel = value;
  }

  virtual void setPrismaticSigmaSystemNoisePhi(const double& value)
  {
      this->_prism_sigma_sys_noise_phi = value;
  }

  virtual void setPrismaticSigmaSystemNoiseTheta(const double& value)
  {
      this->_prism_sigma_sys_noise_theta = value;
  }

  virtual void setPrismaticSigmaSystemNoisePV(const double& value)
  {
      this->_prism_sigma_sys_noise_pv = value;
  }

  virtual void setPrismaticSigmaSystemNoisePVd(const double& value)
  {
      this->_prism_sigma_sys_noise_pvd = value;
  }

  virtual void setPrismaticSigmaMeasurementNoise(const double& value)
  {
      this->_prism_sigma_meas_noise = value;
  }

  virtual void setRevolutePriorCovarianceVelocity(const double& value)
  {
      this->_rev_prior_cov_vel = value;
  }

  virtual void setRevoluteSigmaSystemNoisePhi(const double& value)
  {
      this->_rev_sigma_sys_noise_phi = value;
  }

  virtual void setRevoluteSigmaSystemNoiseTheta(const double& value)
  {
      this->_rev_sigma_sys_noise_theta = value;
  }

  virtual void setRevoluteSigmaSystemNoisePx(const double& value)
  {
      this->_rev_sigma_sys_noise_px = value;
  }

  virtual void setRevoluteSigmaSystemNoisePy(const double& value)
  {
      this->_rev_sigma_sys_noise_py = value;
  }

  virtual void setRevoluteSigmaSystemNoisePz(const double& value)
  {
      this->_rev_sigma_sys_noise_pz = value;
  }

  virtual void setRevoluteSigmaSystemNoiseRV(const double& value)
  {
      this->_rev_sigma_sys_noise_rv = value;
  }

  virtual void setRevoluteSigmaSystemNoiseRVd(const double& value)
  {
      this->_rev_sigma_sys_noise_rvd = value;
  }

  virtual void setRevoluteSigmaMeasurementNoise(const double& value)
  {
      this->_rev_sigma_meas_noise = value;
  }

  virtual void setRevoluteMinimumRotForEstimation(const double& value)
  {
      this->_rev_min_rot_for_ee = value;
  }

  virtual void setRevoluteMaximumJointDistanceForEstimation(const double& value)
  {
      this->_rev_max_joint_distance_for_ee = value;
  }

  virtual void setMinimumJointAgeForEE(const int& value)
  {
      this->_min_joint_age_for_ee = value;
  }

  virtual void setRigidMaxTranslation(const double& value)
  {
      this->_rig_max_translation = value;
  }

  virtual void setRigidMaxRotation(const double& value)
  {
      this->_rig_max_rotation = value;
  }

  virtual void setMinimumNumFramesForNewRB(const int& value)
  {
      this->_min_num_frames_for_new_rb = value;
  }

  virtual JointCombinedFilterPtr getCombinedFilter(int n);

  virtual void estimateJointFiltersProbabilities();

protected:

  /**
   * @brief Prepares the state variable to be returned (getState is constant and we cannot change the state there)
   * The estimated state is the kinemetic structure: for each pair of RBids the most likely JointFilter object,
   * which includes the joint parameters and the latent variables
   *
   */
  virtual void _reflectState();

  ks_analysis_t _ks_analysis_type;
  double _disconnected_j_ne;

  // One filter for each pair of RBs
  joint_combined_filters_map _joint_combined_filters;

  // Reject the first transformations so that the RBM is more stable
  std::map<std::pair<int, int>, int> _precomputing_counter;

  // Save the pose of the RRB in the SF when the SRB was born
  std::map<std::pair<int, int>, Eigen::Twistd > _rrb_pose_at_srb_birthday_in_sf;

  // Save the pose of the SRB in the SF when the SRB was born
  std::map<std::pair<int, int>, Eigen::Twistd > _srb_pose_at_srb_birthday_in_sf;

  omip_msgs::RigidBodyPosesAndVelsMsgPtr _last_rcvd_poses_and_vels;


  omip_msgs::RigidBodyPosesAndVelsMsgPtr _previous_rcvd_poses_and_vels;
  // New: 9.8.2016: With the new trajectory initialization of the rigid bodies the first time we receive
  // the pose of a new rigid body, this rigid body is already min_num_frames_for_new_rb frames old
  // Therefore, the previous pose of the other rigid bodies is not equal to their pose when the new rigid body was born
  // CHANGE: we accumulate a vector of poses with maximum lenght min_num_frames_for_new_rb
  std::list<omip_msgs::RigidBodyPosesAndVelsMsgPtr> _n_previous_rcvd_poses_and_vels;

  int _likelihood_sample_num;
  double _sigma_delta_meas_uncertainty_linear;
  double _sigma_delta_meas_uncertainty_angular;

  double _prism_prior_cov_vel;
  double _prism_sigma_sys_noise_phi;
  double _prism_sigma_sys_noise_theta;
  double _prism_sigma_sys_noise_pv;
  double _prism_sigma_sys_noise_pvd;
  double _prism_sigma_meas_noise;

  double _rev_prior_cov_vel;
  double _rev_sigma_sys_noise_phi;
  double _rev_sigma_sys_noise_theta;
  double _rev_sigma_sys_noise_px;
  double _rev_sigma_sys_noise_py;
  double _rev_sigma_sys_noise_pz;
  double _rev_sigma_sys_noise_rv;
  double _rev_sigma_sys_noise_rvd;
  double _rev_sigma_meas_noise;

  double _rev_min_rot_for_ee;
  double _rev_max_joint_distance_for_ee;

  double _rig_max_translation;
  double _rig_max_rotation;

  int _min_joint_age_for_ee;

  int _min_num_frames_for_new_rb;
};
}

#endif /* JOINT_TRACKER_H_ */
