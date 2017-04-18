#include <boost/foreach.hpp>

#include "joint_tracker/MultiJointTracker.h"

#include <geometry_msgs/Vector3.h>

#include <cmath>

#include <wrappers/matrix/matrix_wrapper.h>

#include "omip_msgs/RigidBodyTwistWithCovMsg.h"
#include "omip_common/OMIPUtils.h"

using namespace omip;
using namespace MatrixWrapper;
using namespace BFL;

MultiJointTracker::MultiJointTracker(double loop_period_ns, ks_analysis_t ks_analysis_type, double dj_ne) :
    RecursiveEstimatorFilterInterface(loop_period_ns),
    _ks_analysis_type(ks_analysis_type),
    _disconnected_j_ne(dj_ne),
    _likelihood_sample_num(0),
    _sigma_delta_meas_uncertainty_linear(0),
    _sigma_delta_meas_uncertainty_angular(0)
{
    _initial_irb_pose = Eigen::Twistd(0,0,0,0,0,0);
    this->_filter_name = "KSFilter";
}

MultiJointTracker::~MultiJointTracker()
{

}

void MultiJointTracker::predictState(double time_interval_ns)
{
    BOOST_FOREACH(joint_combined_filters_map::value_type joint_combined_filter_it, this->_joint_combined_filters)
    {
        joint_combined_filter_it.second->predictState(time_interval_ns);
    }
}

void MultiJointTracker::predictMeasurement()
{
    // We query the predictions from each joint
    // In order to propagate a prediction to the lower level, it must fulfill:
    //  1) It is not a prediction from a disconnected joint (we could remove this condition, it would be ignored at the lower level)
    //  2) There should be only one prediction for a rigid body. The prediction with highest likelihood gets propagated
    this->_predicted_measurement.rb_poses_and_vels.clear();
    omip_msgs::RigidBodyPoseAndVelMsg predicted_pose_and_vel_in_SF;
    omip_msgs::RigidBodyPoseAndVelMsg predicted_pose_and_vel_in_RRBF;
    RB_id_t rbid_predicted;
    RB_id_t rbid_ref_of_prediction;

    std::map<int, double> rbid_predicted_and_likelihood;
    std::map<int, omip_msgs::RigidBodyPoseAndVelMsg> rbid_predicted_and_predictions_in_SF;
    std::map<int, omip_msgs::RigidBodyPoseAndVelMsg> rbid_predicted_and_predictions_in_RRBF;
    BOOST_FOREACH(joint_combined_filters_map::value_type joint_combined_filter_it, this->_joint_combined_filters)
    {
        joint_combined_filter_it.second->predictMeasurement();
        rbid_predicted = joint_combined_filter_it.first.second;
        rbid_ref_of_prediction = joint_combined_filter_it.first.first;
        JointFilterPtr most_probable_joint_hypothesis = joint_combined_filter_it.second->getMostProbableJointFilter();

        // The prediction is useful only if the bodies are not disconnected
        // Only one prediction per RB
        if(most_probable_joint_hypothesis->getJointFilterType() != DISCONNECTED_JOINT)
        {
            std::map<int, double>::iterator previous_prediction_it = rbid_predicted_and_likelihood.find(rbid_predicted);
            // If we already have a prediction for this (second) rigid body, we check which of the two generating models has higher probability
            if(previous_prediction_it != rbid_predicted_and_likelihood.end())
            {
                // If the new prediction comes from a model that is more likely, we take this prediction
                if(previous_prediction_it->second < most_probable_joint_hypothesis->getProbabilityOfJointFilter()
                        || most_probable_joint_hypothesis->getJointFilterType() == PERFECT_GRASP_JOINT
                        || most_probable_joint_hypothesis->getJointFilterType() == UNCONSTRAINED_RY_GRASP_JOINT
                        || most_probable_joint_hypothesis->getJointFilterType() == UNCONSTRAINED_RY_TY_GRASP_JOINT )
                {
                    ROS_INFO_STREAM_NAMED("MultiJointTracker.predictMeasurement", "Replacing previous prediction with prediction from "
                                          << most_probable_joint_hypothesis->getJointFilterTypeStr());
                    predicted_pose_and_vel_in_SF.ref_rb_id = 0;
                    predicted_pose_and_vel_in_SF.rb_id = rbid_predicted;
                    predicted_pose_and_vel_in_SF.pose_wc = most_probable_joint_hypothesis->getPredictedSRBDeltaPoseWithCovInSensorFrame();
                    predicted_pose_and_vel_in_SF.velocity_wc = most_probable_joint_hypothesis->getPredictedSRBVelocityWithCovInSensorFrame();
                    rbid_predicted_and_predictions_in_SF[rbid_predicted] = predicted_pose_and_vel_in_SF;

                    predicted_pose_and_vel_in_RRBF.ref_rb_id = rbid_ref_of_prediction;
                    predicted_pose_and_vel_in_RRBF.rb_id = rbid_predicted;
                    predicted_pose_and_vel_in_RRBF.pose_wc = most_probable_joint_hypothesis->getPredictedSRBPoseWithCovInRRBFrame();
                    //TODO: in RRBF!
                    predicted_pose_and_vel_in_RRBF.velocity_wc = most_probable_joint_hypothesis->getPredictedSRBVelocityWithCovInSensorFrame();
                    rbid_predicted_and_predictions_in_RRBF[rbid_predicted] = predicted_pose_and_vel_in_RRBF;

                    rbid_predicted_and_likelihood[rbid_predicted] = most_probable_joint_hypothesis->getProbabilityOfJointFilter();
                }
            }
            // If we don't have any prediction for this (second) rigid body, we use this prediction
            else{
                ROS_INFO_STREAM_NAMED("MultiJointTracker.predictMeasurement", "Use prediction from "
                                      << most_probable_joint_hypothesis->getJointFilterTypeStr() << " about the pose of " << rbid_predicted);
                predicted_pose_and_vel_in_SF.ref_rb_id = 0;
                predicted_pose_and_vel_in_SF.rb_id = rbid_predicted;
                predicted_pose_and_vel_in_SF.pose_wc = most_probable_joint_hypothesis->getPredictedSRBDeltaPoseWithCovInSensorFrame();
                predicted_pose_and_vel_in_SF.velocity_wc = most_probable_joint_hypothesis->getPredictedSRBVelocityWithCovInSensorFrame();
                rbid_predicted_and_predictions_in_SF[rbid_predicted] = predicted_pose_and_vel_in_SF;

                predicted_pose_and_vel_in_RRBF.ref_rb_id = rbid_ref_of_prediction;
                predicted_pose_and_vel_in_RRBF.rb_id = rbid_predicted;
                predicted_pose_and_vel_in_RRBF.pose_wc = most_probable_joint_hypothesis->getPredictedSRBPoseWithCovInRRBFrame();
                //TODO: in RRBF!
                predicted_pose_and_vel_in_RRBF.velocity_wc = most_probable_joint_hypothesis->getPredictedSRBVelocityWithCovInSensorFrame();
                rbid_predicted_and_predictions_in_RRBF[rbid_predicted] = predicted_pose_and_vel_in_RRBF;

                rbid_predicted_and_likelihood[rbid_predicted] = most_probable_joint_hypothesis->getProbabilityOfJointFilter();
            }
        }
    }

    // We collect all the best predictions into the vector of predicted measurements
    std::map<int, omip_msgs::RigidBodyPoseAndVelMsg>::iterator best_predictions_it = rbid_predicted_and_predictions_in_SF.begin();
    std::map<int, omip_msgs::RigidBodyPoseAndVelMsg>::iterator best_predictions_it_end = rbid_predicted_and_predictions_in_SF.end();
    for(; best_predictions_it != best_predictions_it_end; best_predictions_it++)
    {
        this->_predicted_measurement.rb_poses_and_vels.push_back(best_predictions_it->second);
    }
    best_predictions_it = rbid_predicted_and_predictions_in_RRBF.begin();
    best_predictions_it_end = rbid_predicted_and_predictions_in_RRBF.end();
    for(; best_predictions_it != best_predictions_it_end; best_predictions_it++)
    {
        this->_predicted_measurement.rb_poses_and_vels.push_back(best_predictions_it->second);
    }
}

void MultiJointTracker::setMeasurement(const ks_measurement_t &poses_and_vels, const double &measurement_timestamp_ns)
{
    this->_previous_measurement_timestamp_ns = this->_measurement_timestamp_ns;
    this->_measurement_timestamp_ns = measurement_timestamp_ns;

    this->_previous_rcvd_poses_and_vels.swap(this->_last_rcvd_poses_and_vels);
    this->_last_rcvd_poses_and_vels = boost::shared_ptr<ks_measurement_t>(new ks_measurement_t(poses_and_vels));

    _n_previous_rcvd_poses_and_vels.push_back(boost::shared_ptr<ks_measurement_t>(new ks_measurement_t(poses_and_vels)));
    if(_n_previous_rcvd_poses_and_vels.size() > _min_num_frames_for_new_rb)
    {
        _n_previous_rcvd_poses_and_vels.pop_front();
    }

    size_t rrb_idx_begin = 0;
    size_t rrb_idx_end = 0;

    switch(this->_ks_analysis_type)
    {
    case MOVING_BODIES_TO_STATIC_ENV:
        rrb_idx_begin = 0;
        if(_robot_interaction)
        {
            rrb_idx_end = 2;
        }
        else
        {
            rrb_idx_end = 1;
        }
        break;
    case BETWEEN_MOVING_BODIES:
        rrb_idx_begin = 1;
        rrb_idx_end = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size();
        break;
    case FULL_ANALYSIS:
        rrb_idx_begin = 0;
        rrb_idx_end = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size();
        break;
    }

    for (size_t rrb_idx = rrb_idx_begin; rrb_idx < rrb_idx_end; rrb_idx++)
    {
        for (size_t srb_idx = rrb_idx + 1; srb_idx < this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size(); srb_idx++)
        {
            // Extract RB ids
            int rrb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx).rb_id;
            int srb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).rb_id;

            std::pair<int, int> rrb_srb_ids = std::pair<int, int>(rrb_id, srb_id);

            // Check if this pair of rbs were observed before -> joint was estimated before
            std::map<std::pair<int, int>, JointCombinedFilterPtr>::iterator prev_joint_combined_filter_it =this->_joint_combined_filters.find(rrb_srb_ids);

            // If this is a new pair of RB not analyzed before
            if (prev_joint_combined_filter_it != this->_joint_combined_filters.end())
            { // It was previously stored -> Use it and pass the new measurements
                prev_joint_combined_filter_it->second->setMeasurement(joint_measurement_t(
                                                                          this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx),
                                                                          this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx)), measurement_timestamp_ns);
            }
        }
    }
}

void MultiJointTracker::setMeasurementFT(const std::vector<double>& ft_meas, const double& measurement_timestamp_ns)
{
    for (size_t rrb_idx = 0; rrb_idx < this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size(); rrb_idx++)
    {
        for (size_t srb_idx = rrb_idx + 1; srb_idx < this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size(); srb_idx++)
        {
            // Extract RB ids
            int rrb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx).rb_id;
            int srb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).rb_id;

            // If it is the grasp joint between the end effector rigid body and the interacted rigid body
            if(rrb_id == MULTIMODAL_EE_FILTER_ID && srb_id ==DEFORMED_END_EFFECTOR_FILTER_ID || rrb_id == DEFORMED_END_EFFECTOR_FILTER_ID && srb_id ==INTERACTED_RB_FILTER_ID )
            {
                std::pair<int, int> rrb_srb_ids = std::pair<int, int>(rrb_id, srb_id);

                // Check if this pair of rbs were observed before -> joint was estimated before
                std::map<std::pair<int, int>, JointCombinedFilterPtr>::iterator prev_joint_combined_filter_it =this->_joint_combined_filters.find(rrb_srb_ids);

                // If this is a new pair of RB not analyzed before
                if (prev_joint_combined_filter_it != this->_joint_combined_filters.end())
                { // It was previously stored -> Use it and pass the new measurements
                    prev_joint_combined_filter_it->second->setMeasurementFT(ft_meas, measurement_timestamp_ns);
                }
            }
        }
    }
}

void MultiJointTracker::slippageDetected(int slippage)
{
    std::pair<int, int> rrb_srb_ids = std::pair<int, int>(DEFORMED_END_EFFECTOR_FILTER_ID, INTERACTED_RB_FILTER_ID);

    // Check if this pair of rbs were observed before -> joint was estimated before
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::iterator grasp_joint_combined_filter_it =this->_joint_combined_filters.find(rrb_srb_ids);

    // If this is a new pair of RB not analyzed before
    if (grasp_joint_combined_filter_it != this->_joint_combined_filters.end())
    { // It was previously stored -> Use it and pass the new measurements
        grasp_joint_combined_filter_it->second->slippageDetected(slippage);
    }
}

void MultiJointTracker::setMeasurementEE2CP(const std::vector<double>& ee2cp_rel_pose_meas, const double& measurement_timestamp_ns)
{    
    std::pair<int, int> rrb_srb_ids = std::pair<int, int>(MULTIMODAL_EE_FILTER_ID, DEFORMED_END_EFFECTOR_FILTER_ID);

    // Check if this pair of rbs were observed before -> joint was estimated before
    std::map<std::pair<int, int>, JointCombinedFilterPtr>::iterator deformation_joint_combined_filter_it =this->_joint_combined_filters.find(rrb_srb_ids);

    // If this is a new pair of RB not analyzed before
    if (deformation_joint_combined_filter_it != this->_joint_combined_filters.end())
    { // It was previously stored -> Use it and pass the new measurements
        deformation_joint_combined_filter_it->second->setMeasurementEE2PC(ee2cp_rel_pose_meas, measurement_timestamp_ns);
    }
}

JointCombinedFilterPtr MultiJointTracker::getCombinedFilter(int n)
{
    joint_combined_filters_map::iterator it = this->_joint_combined_filters.begin();

    for(int k=0; k<n; k++)
    {
        it++;
    }
    return it->second;
}

void MultiJointTracker::correctState()
{
    // I create a new one so that the joints are either created, corrected, or deleted if there is no more information about them
    joint_combined_filters_map corrected_joint_combined_filters;

    size_t rrb_idx_begin = 0;
    size_t rrb_idx_end = 0;

    switch(this->_ks_analysis_type)
    {
    case MOVING_BODIES_TO_STATIC_ENV:
        rrb_idx_begin = 0;
        if(_robot_interaction)
        {
            rrb_idx_end = 2;
        }
        else
        {
            rrb_idx_end = 1;
        }
        break;
    case BETWEEN_MOVING_BODIES:
        rrb_idx_begin = 1;
        rrb_idx_end = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size();
        break;
    case FULL_ANALYSIS:
        rrb_idx_begin = 0;
        rrb_idx_end = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size();
        break;
    }

    for (size_t rrb_idx = rrb_idx_begin; rrb_idx < rrb_idx_end; rrb_idx++)
    {
        for (size_t srb_idx = rrb_idx + 1; srb_idx < this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size(); srb_idx++)
        {
            // Extract RB ids
            int rrb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx).rb_id;
            int srb_id = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).rb_id;

            std::pair<int, int> rrb_srb_ids = std::pair<int, int>(rrb_id, srb_id);

            // Check if this pair of rbs were observed before and we already have a joint filter for it
            std::map<std::pair<int, int>, JointCombinedFilterPtr>::iterator prev_joint_combined_filter_it =this->_joint_combined_filters.find(rrb_srb_ids);

            // If we don't have a joint filter for it
            if (prev_joint_combined_filter_it == this->_joint_combined_filters.end())
            {
                // Store the first pose of the interacted body and create a filter only when it starts to move
                if(_robot_interaction && srb_id == INTERACTED_RB_FILTER_ID && rrb_id != DEFORMED_END_EFFECTOR_FILTER_ID)
                {
                    if(_initial_irb_pose == Eigen::Twistd(0,0,0,0,0,0))
                    {
                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).pose_wc.twist, _initial_irb_pose);
                    }else{
                        Eigen::Twistd irb_pose_now;
                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).pose_wc.twist, irb_pose_now);

                        Eigen::Matrix4d pose_first_ht;
                        Twist2TransformMatrix(_initial_irb_pose, pose_first_ht);

                        Eigen::Matrix4d pose_now_ht;
                        Twist2TransformMatrix(irb_pose_now, pose_now_ht);

                        Eigen::Vector3d displacement_now = Eigen::Vector3d(pose_now_ht(0,3), pose_now_ht(1,3), pose_now_ht(2,3)) -
                                Eigen::Vector3d(pose_first_ht(0,3), pose_first_ht(1,3), pose_first_ht(2,3));

                        Eigen::AngleAxisd rotation_now;
                        rotation_now.fromRotationMatrix((pose_now_ht*pose_first_ht.inverse()).block<3,3>(0,0));

                        if(displacement_now.norm() < 0.05 && rotation_now.angle() < 0.2)
                        {
                            continue;
                        }
                    }
                }

                // If the robot is not interacting OR the robot is interacting but neither the reference nor the second rigid bodies are the end effector
                if(!_robot_interaction || (rrb_id != MULTIMODAL_EE_FILTER_ID && srb_id != MULTIMODAL_EE_FILTER_ID)
                        && (rrb_id != DEFORMED_END_EFFECTOR_FILTER_ID && srb_id != DEFORMED_END_EFFECTOR_FILTER_ID))
                {
                    // If we it is the very first time we see this pair rrb-srb (probably because it is the first frame we track the srb)
                    if (this->_precomputing_counter.find(rrb_srb_ids) == this->_precomputing_counter.end())
                    {
                        // 1)
                        // We create an entry for this pair rrb-srb into our counter and set the counter to zero
                        this->_precomputing_counter[rrb_srb_ids] = 0;

                        // 2)
                        // We store the pose of the reference rigid body at the frame/time step when the new (second) rb was detected
                        // Since we estimate an initial trajectory of length _min_num_frames_for_new_rb we query the pose of the
                        // reference rigid body _min_num_frames_for_new_rb frames before
                        // Actually, we are not sure that the reference rb existed _min_num_frames_for_new_rb before
                        // so we query the oldest in our accumulator
                        Eigen::Twistd rrb_pose_at_srb_birthday_in_sf_twist = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
                        std::list<omip_msgs::RigidBodyPosesAndVelsMsgPtr>::iterator acc_frames_it = this->_n_previous_rcvd_poses_and_vels.begin();
                        std::list<omip_msgs::RigidBodyPosesAndVelsMsgPtr>::iterator acc_frames_end = this->_n_previous_rcvd_poses_and_vels.end();

                        for(; acc_frames_it !=  acc_frames_end; acc_frames_it++)
                        {
                            for(int rb_prev_idx = 0; rb_prev_idx < (*acc_frames_it)->rb_poses_and_vels.size(); rb_prev_idx++)
                            {
                                // The RRB was existing in the previous frame
                                if((*acc_frames_it)->rb_poses_and_vels.at(rb_prev_idx).rb_id == rrb_id)
                                {
                                    ROSTwist2EigenTwist((*acc_frames_it)->rb_poses_and_vels.at(rb_prev_idx).pose_wc.twist, rrb_pose_at_srb_birthday_in_sf_twist);
                                }
                            }
                        }
                        this->_rrb_pose_at_srb_birthday_in_sf[rrb_srb_ids] = rrb_pose_at_srb_birthday_in_sf_twist;

                        // 3)
                        // We store the first pose of the second rigid body
                        // Due to the initial trajectory estimation, the first pose received is actually not really the first pose.
                        // Before, with the OMIP version that does NOT place the body frames at the centroid of the body we could assume that the first pose
                        // of the body wrt the sensor was the identity
                        // Now, with the OMIP version that places the body frames at the centroid of the body that assumption does not hold any longer
                        // SOLUTION: at the first frame/time step we get from the RBTracker in the field centroid of the rigid body the centroid we used for the trajectory estimation
                        // We can use this centroid as initial translation
                        // EXCEPTION! If the body is the interacted rigid body because we initialize its pose with the initial pose of the end effector
                        Eigen::Twistd srb_pose_at_srb_birthday_in_sf_twist = Eigen::Twistd(0.,0.,0.,0.,0.,0.);
                        for(int rb_prev_idx = 0; rb_prev_idx < this->_last_rcvd_poses_and_vels->rb_poses_and_vels.size(); rb_prev_idx++)
                        {
                            // The SRB was existing in the previous frame
                            if(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rb_prev_idx).rb_id == srb_id)
                            {
                                if(_robot_interaction && srb_id == INTERACTED_RB_FILTER_ID)
                                {
                                    ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rb_prev_idx).pose_wc.twist, srb_pose_at_srb_birthday_in_sf_twist);
                                }else{
                                    srb_pose_at_srb_birthday_in_sf_twist.vx() = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rb_prev_idx).centroid.x;
                                    srb_pose_at_srb_birthday_in_sf_twist.vy() = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rb_prev_idx).centroid.y;
                                    srb_pose_at_srb_birthday_in_sf_twist.vz() = this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rb_prev_idx).centroid.z;
                                }
                            }
                        }

                        this->_srb_pose_at_srb_birthday_in_sf[rrb_srb_ids] = srb_pose_at_srb_birthday_in_sf_twist;
                    }
                    else
                    {
                        Eigen::Twistd new_twist;
                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).pose_wc.twist, new_twist);

                        Eigen::Matrix4d at_birth;
                        Twist2TransformMatrix(this->_srb_pose_at_srb_birthday_in_sf[rrb_srb_ids], at_birth);

                        Eigen::Matrix4d now_ht;
                        Twist2TransformMatrix(new_twist, now_ht);

                        Eigen::Vector3d displacement = Eigen::Vector3d(now_ht(0,3), now_ht(1,3), now_ht(2,3)) -
                                Eigen::Vector3d(at_birth(0,3), at_birth(1,3), at_birth(2,3));

                        Eigen::AngleAxisd rotation;
                        rotation.fromRotationMatrix((now_ht*at_birth.inverse()).block<3,3>(0,0));

                        // We accumulate _min_joint_age_for_ee iterations (pairs of poses) to initialize the filters with a better estimation
                        // If the second rigid body is the interacted rigid body, we need to also see enough motion to start the estimation
                        if (this->_precomputing_counter[rrb_srb_ids] >= this->_min_joint_age_for_ee
                                && !(_robot_interaction && srb_id == INTERACTED_RB_FILTER_ID && displacement.norm() < 0.05 && rotation.angle() < 0.2))
                        {
                            corrected_joint_combined_filters[rrb_srb_ids] = JointCombinedFilterPtr(new JointCombinedFilter());
                            corrected_joint_combined_filters[rrb_srb_ids]->setLoopPeriodNS(this->_loop_period_ns);
                            corrected_joint_combined_filters[rrb_srb_ids]->setNumSamplesForLikelihoodEstimation(this->_likelihood_sample_num);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceDeltaMeasurementLinear(this->_sigma_delta_meas_uncertainty_linear);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceDeltaMeasurementAngular(this->_sigma_delta_meas_uncertainty_angular);
                            corrected_joint_combined_filters[rrb_srb_ids]->setMaxTranslationRigid(this->_rig_max_translation);
                            corrected_joint_combined_filters[rrb_srb_ids]->setMaxRotationRigid(this->_rig_max_rotation);
                            corrected_joint_combined_filters[rrb_srb_ids]->setJointLikelihoodDisconnected(this->_disconnected_j_ne);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovariancePrior(PRISMATIC_JOINT, this->_prism_prior_cov_vel);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoisePhi(PRISMATIC_JOINT, this->_prism_sigma_sys_noise_phi);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseTheta(PRISMATIC_JOINT, this->_prism_sigma_sys_noise_theta);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseJointState(PRISMATIC_JOINT, this->_prism_sigma_sys_noise_pv);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseJointVelocity(PRISMATIC_JOINT, this->_prism_sigma_sys_noise_pvd);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceMeasurementNoise(PRISMATIC_JOINT, this->_prism_sigma_meas_noise);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovariancePrior(REVOLUTE_JOINT, this->_rev_prior_cov_vel);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoisePhi(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_phi);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseTheta(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_theta);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseJointState(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_rv);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoiseJointVelocity(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_rvd);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoisePx(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_px);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoisePy(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_py);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceAdditiveSystemNoisePz(REVOLUTE_JOINT, this->_rev_sigma_sys_noise_pz);
                            corrected_joint_combined_filters[rrb_srb_ids]->setCovarianceMeasurementNoise(REVOLUTE_JOINT, this->_rev_sigma_meas_noise);
                            corrected_joint_combined_filters[rrb_srb_ids]->setMinRotationRevolute(this->_rev_min_rot_for_ee);
                            corrected_joint_combined_filters[rrb_srb_ids]->setMaxRadiusDistanceRevolute(this->_rev_max_joint_distance_for_ee);
                            corrected_joint_combined_filters[rrb_srb_ids]->setInitialMeasurement(joint_measurement_t(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx),
                                                                                                                     this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx)),
                                                                                                 this->_rrb_pose_at_srb_birthday_in_sf[rrb_srb_ids],
                                                                                                 this->_srb_pose_at_srb_birthday_in_sf[rrb_srb_ids]);
                            corrected_joint_combined_filters[rrb_srb_ids]->initialize();
                        }
                        else
                        {
                            this->_precomputing_counter[rrb_srb_ids] += 1;
                        }
                    }
                }else{                    
                    if(_robot_interaction && rrb_id == MULTIMODAL_EE_FILTER_ID && srb_id == DEFORMED_END_EFFECTOR_FILTER_ID)
                    {
                        ROS_WARN_STREAM_NAMED("MultiJointTracker.correctState",
                                              "Creating a Contact Point filter to predict the motion of the deformed end-effector (RB" <<
                                              srb_id << ") wrt the end effector (RB" << rrb_id << ") based " <<
                                              "on force torque measurements");
                        // For the end effector with any other rigid body, we use the Jacobian
                        corrected_joint_combined_filters[rrb_srb_ids] = JointCombinedFilterPtr(new JointCombinedFilter());

                        corrected_joint_combined_filters[rrb_srb_ids]->useDeformationFilter();

                        corrected_joint_combined_filters[rrb_srb_ids]->setLoopPeriodNS(this->_loop_period_ns);

                        Eigen::Twistd ee_initial_pose, cp_initial_pose;

                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx).pose_wc.twist, ee_initial_pose);
                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).pose_wc.twist, cp_initial_pose);

                        corrected_joint_combined_filters[rrb_srb_ids]->setInitialMeasurement(joint_measurement_t(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx),
                                                                                                                 this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx)),
                                                                                             ee_initial_pose,
                                                                                             cp_initial_pose);
                        //corrected_joint_combined_filters[rrb_srb_ids]->setModelPriorProbability(0.5);
                        corrected_joint_combined_filters[rrb_srb_ids]->initialize();
                    }
                    if(_robot_interaction && rrb_id == DEFORMED_END_EFFECTOR_FILTER_ID && srb_id == INTERACTED_RB_FILTER_ID)
                    {
                        ROS_WARN_STREAM_NAMED("MultiJointTracker.correctState", "Creating a Grasp filter to predict the motion of the interacted rigid body (RB" << srb_id
                                              << ") based on the motion of the deformed end-effector (RB"<<rrb_id << ")");
                        // For the end effector with any other rigid body, we use the Jacobian
                        corrected_joint_combined_filters[rrb_srb_ids] = JointCombinedFilterPtr(new JointCombinedFilter());

                        corrected_joint_combined_filters[rrb_srb_ids]->useGraspFilter(_type_of_grasp);

                        if(_type_of_grasp == 0)
                        {
                            corrected_joint_combined_filters[rrb_srb_ids]->setJointLikelihoodDisconnected(this->_disconnected_j_ne);
                        }
                        corrected_joint_combined_filters[rrb_srb_ids]->setLoopPeriodNS(this->_loop_period_ns);

                        Eigen::Twistd ee_initial_pose, irb_initial_pose;

                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx).pose_wc.twist, ee_initial_pose);
                        ROSTwist2EigenTwist(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx).pose_wc.twist, irb_initial_pose);

                        corrected_joint_combined_filters[rrb_srb_ids]->setInitialMeasurement(joint_measurement_t(this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(rrb_idx),
                                                                                                                 this->_last_rcvd_poses_and_vels->rb_poses_and_vels.at(srb_idx)),
                                                                                             ee_initial_pose,
                                                                                             irb_initial_pose);
                        //corrected_joint_combined_filters[rrb_srb_ids]->setModelPriorProbability(0.5);
                        corrected_joint_combined_filters[rrb_srb_ids]->initialize();
                    }
                }
            }else{
                prev_joint_combined_filter_it->second->correctState();
                corrected_joint_combined_filters[rrb_srb_ids] = prev_joint_combined_filter_it->second;
            }

        }
    }

    this->_joint_combined_filters = corrected_joint_combined_filters;
    this->_reflectState();
}

void MultiJointTracker::_reflectState()
{
    this->_state.clear();
    this->_state = this->_joint_combined_filters;
}

void MultiJointTracker::estimateJointFiltersProbabilities()
{
    BOOST_FOREACH(joint_combined_filters_map::value_type joint_combined_filter_it, this->_joint_combined_filters)
    {
        joint_combined_filter_it.second->estimateJointFilterProbabilities();
    }
}

