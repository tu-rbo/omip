/*
 * RigidJointFilter.h
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

#ifndef RIGIDJOINTFILTER_H_
#define RIGIDJOINTFILTER_H_

#include "joint_tracker/JointFilter.h"

namespace omip
{

class RigidJointFilter;
typedef boost::shared_ptr<RigidJointFilter> RigidJointFilterPtr;

class RigidJointFilter : public JointFilter
{
public:

  /**
   * Constructor
   */
  RigidJointFilter();

  /**
   * Destructor
   */
  virtual ~RigidJointFilter();

  /**
   * Creates a new Joint object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
  RigidJointFilterPtr clone() const
  {
    return (RigidJointFilterPtr(doClone()));
  }

  /**
   * Copy constructor
   */
  RigidJointFilter(const RigidJointFilter &rigid_joint);

  /**
   * Generate a hypothesis about the pose of the second rigid body based on the pose of the
   * reference rigid body and the internal state (joint parameters and latent variable)
   */
  virtual void predictMeasurement();

  /**
   * Measure the error of the last joint parameters by testing them against the full observed trajectory
   * of the second RB relative to the reference RB
   */
  virtual void estimateMeasurementHistoryLikelihood();

  /**
   * Generate a hypothesis about the pose of the second rigid body based on the pose of the
   * reference rigid body and the internal state (joint parameters and latent variable)
   */
  virtual geometry_msgs::TwistWithCovariance getPredictedSRBPoseWithCovInSensorFrame();

  /**
 * @brief Generate a prediction about the change in pose of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
 * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
 *
 * @return geometry_msgs::TwistWithCovariance Change in pose of the second rigid body in form of a twist with covariance based on the model uncertainty
 */
  virtual geometry_msgs::TwistWithCovariance getPredictedSRBDeltaPoseWithCovInSensorFrame();

  /**
 * @brief Generate a prediction about the velocity of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
 * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
 *
 * @return geometry_msgs::TwistWithCovariance Velocity of the second rigid body in form of a twist with covariance based on the model uncertainty
 */
  virtual geometry_msgs::TwistWithCovariance getPredictedSRBVelocityWithCovInSensorFrame();


  /**
   * Return rviz markers that show the type and parameters of the estimated joint
   */
  virtual std::vector<visualization_msgs::Marker> getJointMarkersInRRBFrame() const;

  /**
   * Return the joint type as one of the possible JointFilterTypes
   */
  virtual JointFilterType getJointFilterType() const;

  virtual void estimateUnnormalizedModelProbability() ;

  /**
   * Return the joint type as an string
   */
  virtual std::string getJointFilterTypeStr() const;

  virtual void initialize();

  virtual void setMaxTranslationRigid(double max_trans);

  virtual void setMaxRotationRigid(double max_rot);

protected:

  /**
   * A Rigid Joint constrains the 6 dofs of the relative motion between RBs
   * Rigid Joints have no motion parameters
   * Rigid Joints have no latent variables
   */

  virtual RigidJointFilter* doClone() const
  {
    return (new RigidJointFilter(*this));
  }

  double _rig_max_translation;
  double _rig_max_rotation;

  double _motion_memory_prior;

};

}

#endif /* RIGIDJOINTFILTER_H_ */
