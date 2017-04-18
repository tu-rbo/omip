/*
 * PerfectGraspFilter.h
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

#ifndef PerfectGraspFilter_H_
#define PerfectGraspFilter_H_

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
#include "joint_tracker/pdf/NonLinearGraspMeasurementPdf.h"

#include <tf/transform_listener.h>

namespace omip
{

class PerfectGraspFilter;
typedef boost::shared_ptr<PerfectGraspFilter> PerfectGraspFilterPtr;

class PerfectGraspFilter : public JointFilter
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
   * Constructor
   */
    PerfectGraspFilter();

    /**
   * Destructor
   */
    virtual ~PerfectGraspFilter();

    /**
   * Creates a new Joint object with the same values as this one and passes
   * its reference
   *
   * @return - A reference to the clone of this Feature object
   */
    PerfectGraspFilterPtr clone() const
    {
        return (PerfectGraspFilterPtr(doClone()));
    }

    /**
   * Copy constructor
   */
    PerfectGraspFilter(const PerfectGraspFilter &rev_joint);

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
   * Measure the error of the last joint parameters by testing them against the full observed trajectory
   * of the second RB relative to the reference RB
   */
    virtual void estimateMeasurementHistoryLikelihood();

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

    virtual void setProprioceptionWeight(double pp_weight)
    {
        _pp_weight = pp_weight;
    }

     virtual void estimateUnnormalizedModelProbability();

protected:

    /**
   * A Grasp Joint constrains 0 of the 6 dofs of the relative motion between RBs
   */

    BFL::LinearAnalyticConditionalGaussian* _sys_PDF;
    BFL::LinearAnalyticSystemModelGaussianUncertainty* _sys_MODEL;

    BFL::NonLinearGraspMeasurementPdf* _meas_PDF;
    BFL::AnalyticMeasurementModelGaussianUncertainty* _meas_MODEL;

    BFL::ExtendedKalmanFilter* _ekf;

    void _initializeSystemModel();
    void _initializeMeasurementModel();
    void _initializeEKF();

    BFL::Matrix _A;

    Eigen::Twistd _delta_eef_wrt_cf_ec;
    Eigen::Twistd _delta_eef_wrt_previous_eef_ec;

    Eigen::Twistd _ft_based_ee_grasping_point_pose_ec;

    virtual PerfectGraspFilter* doClone() const
    {
        return (new PerfectGraspFilter(*this));
    }

    double _pp_weight;

    tf::TransformListener _tf_listener;
};

}

#endif /* PerfectGraspFilter_H_ */
