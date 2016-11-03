/*
 * DisconnectedJointFilter.h
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

#ifndef DISCONNECTEDJOINTFILTER_H_
#define DISCONNECTEDJOINTFILTER_H_

#include "joint_tracker/JointFilter.h"

namespace omip
{

class DisconnectedJointFilter;
typedef boost::shared_ptr<DisconnectedJointFilter> DisconnectedJointFilterPtr;

class DisconnectedJointFilter : public JointFilter
{
public:

  /**
   * Constructor
   */
  DisconnectedJointFilter();

  /**
   * Destructor
   */
  virtual ~DisconnectedJointFilter();

  /**
   * Creates a new Joint object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
  DisconnectedJointFilterPtr clone() const
  {
    return (DisconnectedJointFilterPtr(doClone()));
  }

  /**
   * Copy constructor
   */
  DisconnectedJointFilter(const DisconnectedJointFilter &rigid_joint);

  /**
   * @brief Generate a prediction about the pose-twist of the second rigid body (SRB) and its covariance using the frame of sensor (SF) as observation and ref frame
   * based on the predicted measurement and the predicted next pose of the reference rigid body (RRB) in the sensor frame (SF)
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

  /**
   * Return the joint type as an string
   */
  virtual std::string getJointFilterTypeStr() const;

  /**
   * @brief Return the normalized joint filter probability of the Disconnected Joint model, which is a constant value we set at the beginning and use
   * as quality threshold for the other models
   *
   * @return double Normalized probability of this joint filter
   */
  virtual double getProbabilityOfJointFilter() const;

  virtual void initialize();

protected:

  /**
   * A Disconnected Joint does not constrain any of the 6 dofs of the relative motion between RBs
   * Disconnected Joints have no motion parameters
   * Disconnected Joints have no latent variables
   */

  // Disconnected Joint is the only model where we ALWAYS know the normalized probability of the model given the data, because
  // it is a value that we use as quality threshold for all other joints (the normalized probability of any other joint should be higher than this
  // constant value to be selected as most probable joint)
  //double _unnormalized_disc_joint_probability;

  virtual DisconnectedJointFilter* doClone() const
  {
    return (new DisconnectedJointFilter(*this));
  }

};

}

#endif /* DISCONNECTEDJOINTFILTER_H_ */
