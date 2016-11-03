/*
 * PrismaticJointFilter.h
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

#ifndef PRISMATICJOINTFILTER_H_
#define PRISMATICJOINTFILTER_H_

#include "joint_tracker/JointFilter.h"

// Bayesian Filtering ROS
#include <pdf/linearanalyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <model/measurementmodel.h>
#include "joint_tracker/pdf/NonLinearPrismaticMeasurementPdf.h"

namespace omip
{

class PrismaticJointFilter;
typedef boost::shared_ptr<PrismaticJointFilter> PrismaticJointFilterPtr;

class PrismaticJointFilter : public JointFilter
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Constructor
   */
  PrismaticJointFilter();

  /**
   * Destructor
   */
  virtual ~PrismaticJointFilter();

  /**
   * Creates a new Joint object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
  PrismaticJointFilterPtr clone() const
  {
    return (PrismaticJointFilterPtr(doClone()));
  }

  /**
   * Copy constructor
   */
  PrismaticJointFilter(const PrismaticJointFilter &prismatic_joint);

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
   * Use the observed pose as measurement for the EKF -> Update joint parameters
   */
  virtual void correctState();

  /**
   * Measure the error of the last joint parameters by testing them against samples of the full observed trajectory
   * of the second RB relative to the reference RB
   */
  virtual void estimateMeasurementHistoryLikelihood();
  /**
   * @brief We use the kinematic structure to predict the next pose-twist of the second rigid body of the joint (with covariance based on the model uncertainty)
   * in sensor frame (the frame where we track the RBs). This values will be passed to the MultiRBTracker
   *
   * @return geometry_msgs::TwistWithCovariance Pose of the second rigid body in form of a twist with covariance based on the model uncertainty
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

  virtual void estimateUnnormalizedModelProbability() ;

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

  virtual void initialize();

  virtual void setCovarianceDeltaMeasurementLinear(double delta_meas);

protected:

  /**
   * A Prismatic Joint constrains 5 of the 6 dofs of the relative motion between RBs
   * Prismatic Joints have only joint orientation as motion parameters
   * The latent variable is the distance of the translation of the second RB wrt to the reference RB
   */

  double _sigma_delta_meas_uncertainty_linear;

  BFL::LinearAnalyticConditionalGaussian* _sys_PDF;
  BFL::LinearAnalyticSystemModelGaussianUncertainty* _sys_MODEL;

  BFL::NonLinearPrismaticMeasurementPdf* _meas_PDF;
  BFL::AnalyticMeasurementModelGaussianUncertainty* _meas_MODEL;

  BFL::ExtendedKalmanFilter* _ekf;

  void _initializeSystemModel();
  void _initializeMeasurementModel();
  void _initializeEKF();

  virtual PrismaticJointFilter* doClone() const
  {
    return (new PrismaticJointFilter(*this));
  }

};

}

#endif /* PRISMATICJOINTFILTER_H_ */
