/*
 * EndEffectorFilter.h
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

#ifndef ENDEFFECTOR_FILTER_H_
#define ENDEFFECTOR_FILTER_H_

#include "rb_tracker/RBFilter.h"

namespace omip
{

/**
 * Class EndEffectorFilter
 * Special type of filter to track the end effector
 * We can leverage additional priors about the interaction (i.e. forces, joint values, ee trajectories) to track it
 */
class EndEffectorFilter : public RBFilter
{
public:

  // Shared pointer type
  typedef boost::shared_ptr<EndEffectorFilter> Ptr;

  /**
   * Constructor
   * @param static_motion_threshold - Threshold to detect Features that move (do not support the static rigid body any longer)
   */
  EndEffectorFilter(double loop_period_ns,
                     const std::vector<Eigen::Matrix4d> &first_transform,
                     const Eigen::Twistd &initial_velocity,
                     FeaturesDataBase::Ptr feats_database,
                     double estimation_error_threshold);

  virtual ~EndEffectorFilter(){}

  virtual void setMeasurement(const Eigen::Matrix4d& current_eef_wrt_cf, double time_pose_ns);

  virtual void predictState(double time_interval_ns = -1.);

  //virtual void predictMeasurement();

  virtual void correctState();

protected:

  //virtual void _correctStateProprioception();

  virtual void _setInitialEndEffectorPose(const Eigen::Matrix4d& initial_ee_pose, double time_pose_ns);

  Eigen::Matrix4d _current_measured_pose_ee_wrt_cf;
  Eigen::Matrix4d _previous_eef_wrt_cf;

  Eigen::Twistd _current_measured_velocity_ee_wrt_cf;

  double _current_time_pose_ns;
  double _previous_time_pose_ns;

  Eigen::Matrix4d _delta_eef_wrt_cf;

  Eigen::Matrix4d _current_ee_pose_ht;
  Feature::Location _ee_location_wrt_cf;

  Eigen::Twistd _predicted_delta_eep_wrt_cf_ec_vh;  // Velocity hypothesis
  Eigen::Twistd _predicted_velocity_ee_wrt_cf_vh;  // Velocity hypothesis
  Eigen::Twistd _predicted_delta_eep_wrt_cf_ec_bh;  // Brake hypothesis
  Eigen::Twistd _predicted_velocity_eef_wrt_cf_ec_bh;  // Brake hypothesis

  Eigen::Matrix<double, 6, 6> _proprioception_meas_cov_eig;

  bool _invalid_measurement;
};
}

#endif /* ENDEFFECTOR_FILTER_H_ */
