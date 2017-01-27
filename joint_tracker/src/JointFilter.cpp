#include "joint_tracker/JointFilter.h"

#include "omip_common/OMIPUtils.h"

#include <tf/transform_listener.h>

using namespace omip;

JointFilter::JointFilter():
    _loop_period_ns(-1),
    _joint_id(-1),
    _likelihood_sample_num(-1),
    _externally_set_likelihood(false),
    _measurement_timestamp_ns(0),
    _unnormalized_model_probability(0),
    _rrb_id(-1),
    _inverted_delta_srb_pose_in_rrbf(false),
    _from_inverted_to_non_inverted(false),
    _from_non_inverted_to_inverted(false)
{
    this->_initVariables(_joint_id);
}

void JointFilter::setInitialMeasurement(const joint_measurement_t &initial_measurement,
                                        const Eigen::Twistd& rrb_pose_at_srb_birth_in_sf,
                                        const Eigen::Twistd& srb_pose_at_srb_birth_in_sf)
{
    // Extract current pose of the reference RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // PoseCoord({rrbf}|RRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.first.pose_wc.twist, this->_rrb_current_pose_in_sf);
    this->_rrb_previous_pose_in_sf = this->_rrb_current_pose_in_sf;

    // Extract current velocity of the reference RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({rrbf}|RRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.first.velocity_wc.twist, this->_rrb_current_vel_in_sf);

    // Extract current pose of the second RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({sfbf}|SRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.second.pose_wc.twist, this->_srb_current_pose_in_sf);

    // Extract current velocity of the second RB frame relative to the current sensor frame expressed in current sensor frame coordinates
    // TwistCoord({sfbf}|SRB,{sf}|S,[sf])(t)
    ROSTwist2EigenTwist(initial_measurement.second.velocity_wc.twist, this->_srb_current_vel_in_sf);

    // Extract covariance of the reference and the second RB wrt sensor frame
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            this->_rrb_pose_cov_in_sf(i, j) = initial_measurement.first.pose_wc.covariance[6 * i + j];
            this->_rrb_vel_cov_in_sf(i, j) = initial_measurement.first.velocity_wc.covariance[6 * i + j];
            this->_srb_pose_cov_in_sf(i, j) = initial_measurement.second.pose_wc.covariance[6 * i + j];
        }
    }

    // Estimate the initial pose of the second RB frame relative to the initial reference RB frame in initial reference RB frame coordinates
    // TwistCoord({srbf}|SRB,{rrbf}|RRB,[rrbf])(0)
    Eigen::Displacementd T_sf_srbf_t0 = srb_pose_at_srb_birth_in_sf.exp(1e-20);
    Eigen::Displacementd T_sf_rrbf_t0 = rrb_pose_at_srb_birth_in_sf.exp(1e-20);
    Eigen::Displacementd T_rrbf_sf_t0 = T_sf_rrbf_t0.inverse();
    Eigen::Displacementd T_rrbf_srbf_t0 = T_rrbf_sf_t0 * T_sf_srbf_t0;
    this->_srb_initial_pose_in_rrbf = T_rrbf_srbf_t0.log(1.0e-20);

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> rrb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf_t0, _rrb_pose_cov_in_sf, rrb_pose_cov_in_rrbf);
    Eigen::Matrix<double, 6, 6> srb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf_t0, _srb_pose_cov_in_sf, srb_pose_cov_in_rrbf);
    this->_srb_initial_pose_cov_in_rrbf = rrb_pose_cov_in_rrbf + srb_pose_cov_in_rrbf;

    // Estimate the current pose of the second RB frame relative to the current reference RB frame in current reference RB frame coordinates
    // TwistCoord({srbf}|SRB,{rrbf}|RRB,[rrbf])(t)
    Eigen::Displacementd T_sf_rrbf_t = this->_rrb_current_pose_in_sf.exp(1e-20);
    Eigen::Displacementd T_sf_srbf_t = this->_srb_current_pose_in_sf.exp(1e-20);
    Eigen::Displacementd T_rrbf_sf_t = T_sf_rrbf_t.inverse();
    Eigen::Displacementd T_rrbf_srbf_t = T_rrbf_sf_t * T_sf_srbf_t;
    this->_srb_current_pose_in_rrbf = T_rrbf_srbf_t.log(1.0e-20);
    this->_srb_previous_pose_in_rrbf = this->_srb_current_pose_in_rrbf;

    // Estimate the transformation between the initial and the current pose of the second RB expressed in the coordinates of the reference RB frame
    this->_current_delta_pose_in_rrbf = (_srb_current_pose_in_rrbf.exp(1e-12)*(_srb_initial_pose_in_rrbf.exp(1e-12).inverse())).log(1e-12);

    this->_previous_delta_pose_in_rrbf = this->_current_delta_pose_in_rrbf;
    this->_delta_poses_in_rrbf.push_back(this->_current_delta_pose_in_rrbf);

    _rrb_id = initial_measurement.first.rb_id;
    // Extract centroid of the reference RB in sensor frame
    if(_rrb_id == 0)
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d(0.,0.,0.);
    }
    else
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d( initial_measurement.first.centroid.x,
                                                     initial_measurement.first.centroid.y,
                                                     initial_measurement.first.centroid.z);
    }

    // Extract centroid of the second RB in sensor frame
    this->_srb_centroid_in_sf = Eigen::Vector3d( initial_measurement.second.centroid.x,
                                                 initial_measurement.second.centroid.y,
                                                 initial_measurement.second.centroid.z);
}

void JointFilter::_initVariables(JointCombinedFilterId joint_id)
{
    this->_joint_id = joint_id;

    this->_joint_state = 0;
    this->_joint_velocity = 0;

    this->_measurements_likelihood = 0.;
    this->_model_prior_probability = 1.0/4.0;
    this->_joint_position = Eigen::Vector3d(0,0,0);
    this->_uncertainty_joint_position = Eigen::Matrix3d::Identity();
    this->_joint_orientation_phi = 0;
    this->_joint_orientation_theta = 0;
    this->_joint_orientation = Eigen::Vector3d(1,0,0);
    this->_uncertainty_joint_orientation_phitheta = Eigen::Matrix2d::Identity();

    this->_rrb_current_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_rrb_previous_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_rrb_current_vel_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_current_pose_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_current_vel_in_sf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);

    this->_srb_initial_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_current_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_previous_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_predicted_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_srb_previous_predicted_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_current_delta_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_previous_delta_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
    this->_predicted_delta_pose_in_rrbf = Eigen::Twistd(0.,0.,0.,0.,0.,0.);

    this->_rrb_pose_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();
    this->_rrb_vel_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();
    this->_srb_pose_cov_in_sf =  Eigen::Matrix<double, 6, 6>::Zero();
    this->_srb_pose_cov_in_sf = Eigen::Matrix<double, 6, 6>::Zero();

    this->_normalizing_term = 1;
}

JointFilter::~JointFilter()
{

}

JointFilter::JointFilter(const JointFilter &joint)
{
    this->_joint_id = joint._joint_id;
    this->_measurements_likelihood = joint._measurements_likelihood;
    this->_model_prior_probability = joint._model_prior_probability;
    this->_rrb_centroid_in_sf = joint._rrb_centroid_in_sf;
    this->_srb_centroid_in_sf = joint._srb_centroid_in_sf;
    this->_joint_position = joint._joint_position;
    this->_uncertainty_joint_position = joint._uncertainty_joint_position;
    this->_joint_orientation_phi = joint._joint_orientation_phi;
    this->_joint_orientation_theta = joint._joint_orientation_theta;
    this->_joint_orientation = joint._joint_orientation;
    this->_uncertainty_joint_orientation_phitheta = joint._uncertainty_joint_orientation_phitheta;

    this->_rrb_current_pose_in_sf = joint._rrb_current_pose_in_sf;
    this->_rrb_previous_pose_in_sf = joint._rrb_previous_pose_in_sf;
    this->_srb_current_pose_in_sf = joint._srb_current_pose_in_sf;
    this->_rrb_current_vel_in_sf= joint._rrb_current_vel_in_sf;
    this->_srb_current_vel_in_sf = joint._srb_current_vel_in_sf;

    this->_srb_initial_pose_in_rrbf = joint._srb_initial_pose_in_rrbf;
    this->_srb_current_pose_in_rrbf= joint._srb_current_pose_in_rrbf;
    this->_srb_previous_pose_in_rrbf = joint._srb_previous_pose_in_rrbf;
    this->_srb_predicted_pose_in_rrbf = joint._srb_predicted_pose_in_rrbf;

    this->_srb_previous_predicted_pose_in_rrbf = joint._srb_previous_predicted_pose_in_rrbf;

    this->_current_delta_pose_in_rrbf = joint._current_delta_pose_in_rrbf;
    this->_previous_delta_pose_in_rrbf = joint._previous_delta_pose_in_rrbf;
    this->_predicted_delta_pose_in_rrbf = joint._predicted_delta_pose_in_rrbf;

    this->_delta_poses_in_rrbf = joint._delta_poses_in_rrbf;

    this->_rrb_pose_cov_in_sf = joint._rrb_pose_cov_in_sf;
    this->_srb_pose_cov_in_sf = joint._srb_pose_cov_in_sf;

    this->_loop_period_ns = joint._loop_period_ns;
}

void JointFilter::setMeasurement(joint_measurement_t acquired_measurement, const double &measurement_timestamp_ns)
{
    _measurement_timestamp_ns = measurement_timestamp_ns;

    // Extract RB ids
    _rrb_id = acquired_measurement.first.rb_id;
    int srb_id = acquired_measurement.second.rb_id;

    // Store the previous pose of the rrb
    this->_rrb_previous_pose_in_sf = this->_rrb_current_pose_in_sf;
    // Extract pose of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.first.pose_wc.twist,this->_rrb_current_pose_in_sf);
    // Extract velocity of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.first.velocity_wc.twist, this->_rrb_current_vel_in_sf);
    // Extract pose of the second RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.second.pose_wc.twist, this->_srb_current_pose_in_sf);
    // Extract velocity of the reference RB in sensor frame
    ROSTwist2EigenTwist(acquired_measurement.second.velocity_wc.twist, this->_srb_current_vel_in_sf);    
    // Extract covariance of the reference and the second RB wrt sensor frame
    for (unsigned int i = 0; i < 6; i++)
    {
        for (unsigned int j = 0; j < 6; j++)
        {
            this->_rrb_pose_cov_in_sf(i, j) = acquired_measurement.first.pose_wc.covariance[6 * i + j];
            this->_rrb_vel_cov_in_sf(i, j) = acquired_measurement.first.velocity_wc.covariance[6 * i + j];
            this->_srb_pose_cov_in_sf(i, j) = acquired_measurement.second.pose_wc.covariance[6 * i + j];
        }
    }

    Eigen::Displacementd T_sf_rrbf = this->_rrb_current_pose_in_sf.exp(1.0e-20);
    Eigen::Displacementd T_sf_srbf = this->_srb_current_pose_in_sf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_sf = T_sf_rrbf.inverse();
    Eigen::Displacementd T_rrbf_srbf = T_rrbf_sf * T_sf_srbf;
    this->_srb_current_pose_in_rrbf = T_rrbf_srbf.log(1e-20);

    // If the rigid body makes a turn of n x PI/2 the twist changes abruptly
    // We try to avoid this by comparing to the previous delta and enforcing a smooth change
    bool inverted = false;
    this->_srb_current_pose_in_rrbf = invertTwist(this->_srb_current_pose_in_rrbf, this->_srb_previous_pose_in_rrbf, inverted);

    //ROS_ERROR_STREAM("After unwrapping pose of second in ref: " << this->_srb_current_pose_in_rrbf);
    this->_srb_previous_pose_in_rrbf = this->_srb_current_pose_in_rrbf;

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> rrb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf, _rrb_pose_cov_in_sf, rrb_pose_cov_in_rrbf);
    Eigen::Matrix<double, 6, 6> srb_pose_cov_in_rrbf;
    adjointXcovXadjointT(T_rrbf_sf, _srb_pose_cov_in_sf, srb_pose_cov_in_rrbf);
    this->_srb_current_pose_cov_in_rrbf = rrb_pose_cov_in_rrbf + srb_pose_cov_in_rrbf;

    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd delta_displ = T_rrbf_srbf * (T_rrbf_srbf_t0.inverse());

    this->_current_delta_pose_in_rrbf = delta_displ.log(1.0e-20);

    // If the rigid body makes a turn of n x PI/2 the twist changes abruptly
    // We try to avoid this by comparing to the previous delta and enforcing a smooth change
    bool inverted_before = _inverted_delta_srb_pose_in_rrbf;
    this->_current_delta_pose_in_rrbf = invertTwist(this->_current_delta_pose_in_rrbf, this->_previous_delta_pose_in_rrbf, this->_inverted_delta_srb_pose_in_rrbf);
    this->_previous_delta_pose_in_rrbf = this->_current_delta_pose_in_rrbf;

    _from_inverted_to_non_inverted = false;
    _from_non_inverted_to_inverted = false;
    if(inverted_before != _inverted_delta_srb_pose_in_rrbf)
    {
        if(inverted_before && !_inverted_delta_srb_pose_in_rrbf)
        {
            _from_inverted_to_non_inverted = true;
        }else{
            _from_non_inverted_to_inverted = true;
        }
    }

    this->_delta_poses_in_rrbf.push_back(this->_current_delta_pose_in_rrbf);

    //See: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6727494 (48,55)
    //See http://ethaneade.com/lie.pdf 8.3
    Eigen::Matrix<double, 6, 6> transformed_cov;
    adjointXinvAdjointXcovXinvAdjointTXadjointT(T_rrbf_srbf, T_rrbf_srbf_t0, _srb_initial_pose_cov_in_rrbf, transformed_cov);
    this->_current_delta_pose_cov_in_rrbf = _srb_current_pose_cov_in_rrbf + transformed_cov;

    // Extract centroid of the reference RB in sensor frame
    if(_rrb_id == 0)
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d(0.,0.,0.);
    }
    else
    {
        this->_rrb_centroid_in_sf = Eigen::Vector3d( acquired_measurement.first.centroid.x,
                                                     acquired_measurement.first.centroid.y,
                                                     acquired_measurement.first.centroid.z);
    }

    // Extract centroid of the second RB in sensor frame
    this->_srb_centroid_in_sf = Eigen::Vector3d( acquired_measurement.second.centroid.x,
                                                 acquired_measurement.second.centroid.y,
                                                 acquired_measurement.second.centroid.z);
}

Eigen::Twistd JointFilter::getPredictedMeasurement() const
{
    return this->_srb_predicted_pose_in_rrbf;
}

void JointFilter::setLoopPeriodNS(double loop_period_ns)
{
    this->_loop_period_ns = loop_period_ns;
}

double JointFilter::getLoopPeriodNS() const
{
    return this->_loop_period_ns;
}

void JointFilter::setNumSamplesForLikelihoodEstimation(int likelihood_sample_num)
{
    this->_likelihood_sample_num = likelihood_sample_num;
}

int JointFilter::getNumSamplesForLikelihoodEstimation() const
{
    return this->_likelihood_sample_num;
}

void JointFilter::setNormalizingTerm(double normalizing_term)
{
    this->_normalizing_term = normalizing_term;
}

double JointFilter::getNormalizingTerm()
{
    return this->_normalizing_term;
}

void JointFilter::setModelPriorProbability(double model_prior_probability)
{
    this->_model_prior_probability = model_prior_probability;
}

double JointFilter::getModelPriorProbability() const
{
    return this->_model_prior_probability;
}

double JointFilter::getLikelihoodOfLastMeasurements() const
{
    return this->_measurements_likelihood;
}

void JointFilter::setLikelihoodOfLastMeasurements(double likelihood)
{
    this->_measurements_likelihood = likelihood;
    this->_externally_set_likelihood = true;
}

double JointFilter::getUnnormalizedProbabilityOfJointFilter() const
{
    return _unnormalized_model_probability;
}

double JointFilter::getProbabilityOfJointFilter() const
{
    return (_unnormalized_model_probability / this->_normalizing_term);
}

void JointFilter::setJointId(JointCombinedFilterId joint_id)
{
    this->_joint_id = joint_id;
}

JointCombinedFilterId JointFilter::getJointId() const
{
    return this->_joint_id;
}

double JointFilter::getJointState() const
{
    return _joint_state;
}

double JointFilter::getJointVelocity() const
{
    return _joint_velocity;
}

double JointFilter::getOrientationPhiInRRBFrame()  const
{
    return this->_joint_orientation_phi;
}

double JointFilter::getOrientationThetaInRRBFrame()  const
{
    return this->_joint_orientation_theta;
}

double JointFilter::getCovarianceOrientationPhiPhiInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(0,0);
}

double JointFilter::getCovarianceOrientationThetaThetaInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(1,1);
}

double JointFilter::getCovarianceOrientationPhiThetaInRRBFrame() const
{
    return this->_uncertainty_joint_orientation_phitheta(1,0);
}

Eigen::Vector3d JointFilter::getJointPositionInRRBFrame() const
{
    return this->_joint_position;
}

Eigen::Vector3d JointFilter::getJointOrientationRPYInRRBFrame() const
{
    double phi = std::acos(this->_joint_orientation.z() / this->_joint_orientation.norm());
    double theta = std::atan(this->_joint_orientation.y() / this->_joint_orientation.x());
    Eigen::Vector3d joint_ori_rpy(0., theta, phi);

    return joint_ori_rpy;
}

Eigen::Vector3d JointFilter::getJointOrientationUnitaryVector() const
{
    return this->_joint_orientation;
}

void JointFilter::setCovariancePrior(double  prior_cov_vel)
{
    this->_prior_cov_vel =  prior_cov_vel;
}

void JointFilter::setCovarianceAdditiveSystemNoisePhi(double  sigma_sys_noise_phi)
{
    this->_sigma_sys_noise_phi =  sigma_sys_noise_phi;
}

void JointFilter::setCovarianceAdditiveSystemNoiseTheta(double  sigma_sys_noise_theta)
{
    this->_sigma_sys_noise_theta =  sigma_sys_noise_theta;
}

void JointFilter::setCovarianceAdditiveSystemNoisePx(double  sigma_sys_noise_px)
{
    this->_sigma_sys_noise_px =  sigma_sys_noise_px;
}

void JointFilter::setCovarianceAdditiveSystemNoisePy(double  sigma_sys_noise_py)
{
    this->_sigma_sys_noise_py =  sigma_sys_noise_py;
}

void JointFilter::setCovarianceAdditiveSystemNoisePz(double  sigma_sys_noise_pz)
{
    this->_sigma_sys_noise_pz =  sigma_sys_noise_pz;
}

void JointFilter::setCovarianceAdditiveSystemNoiseJointState(double  sigma_sys_noise_pv)
{
    this->_sigma_sys_noise_jv =  sigma_sys_noise_pv;
}

void JointFilter::setCovarianceAdditiveSystemNoiseJointVelocity(double  sigma_sys_noise_pvd)
{
    this->_sigma_sys_noise_jvd =  sigma_sys_noise_pvd;
}

void JointFilter::setCovarianceMeasurementNoise(double  sigma_meas_noise)
{
    this->_sigma_meas_noise =  sigma_meas_noise;
}

void JointFilter::initialize()
{
#ifdef PUBLISH_PREDICTED_POSE_AS_PWC
    // This is used to visualize the predictions based on the joint hypothesis
    std::ostringstream oss_pwc_topic;
    oss_pwc_topic << "/joint_tracker/predpose_srb_j" << this->_joint_id << "_" << this->getJointFilterTypeStr();
    _predicted_next_pose_publisher = _predicted_next_pose_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(oss_pwc_topic.str(), 100);
#endif
}
