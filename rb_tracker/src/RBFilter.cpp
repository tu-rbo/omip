#include <boost/foreach.hpp>

#include "rb_tracker/RBFilter.h"
#include "omip_common/OMIPUtils.h"

#include <boost/numeric/ublas/matrix.hpp>

#include <iostream>
#include <fstream>

#include <Eigen/Cholesky>

#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

using namespace omip;

RB_id_t RBFilter::_rb_id_generator = 1;

using namespace MatrixWrapper;
using namespace BFL;

// Dimensions of each measurement (each feature location)
#define MEAS_DIM_RBF 3

// Value of the measurement noise (unused, only set at the beginning, but it will change for each concrete feature location)
#define COV_MEAS_RBF 1.0

RBFilter::RBFilter() :
    _estimation_error_threshold(-1.),
    _id(RBFilter::_rb_id_generator++),
    _velocity( 0., 0., 0., 0., 0., 0.),
    _features_database(),
    _loop_period_ns(0.),
    _min_num_supporting_feats_to_correct(5),
    _use_predicted_state_from_kh(false),
    _use_predicted_measurement_from_kh(false),
    _last_time_interval_ns(0.0)
{
    _pose = Eigen::Matrix4d::Identity();
    this->_initial_location_of_centroid = Feature::Location(0,0,0);
}

RBFilter::RBFilter(double loop_period_ns,
                   const std::vector<Eigen::Matrix4d>& trajectory,
                   const Eigen::Twistd &initial_velocity,
                   FeaturesDataBase::Ptr feats_database,
                   double estimation_error_threshold) :
    _loop_period_ns(loop_period_ns),
    _last_time_interval_ns(loop_period_ns),
    _estimation_error_threshold(estimation_error_threshold),
    _id(RBFilter::_rb_id_generator++),
    _pose(trajectory[trajectory.size()-1]),
    _velocity(initial_velocity),
    _features_database(feats_database),
    _min_num_supporting_feats_to_correct(5),
    _use_predicted_state_from_kh(false),
    _use_predicted_measurement_from_kh(false)
{
    this->_initial_location_of_centroid = Feature::Location(trajectory[0](0,3),trajectory[0](1,3),trajectory[0](2,3));
    this->_trajectory = std::vector<Eigen::Matrix4d>(trajectory.begin()+1, trajectory.end());
}

void RBFilter::Init()
{
    this->_initializeAuxiliarMatrices();
}

RBFilter::RBFilter(const RBFilter &rbm)
{
    this->_id = rbm._id;
    this->_pose = rbm._pose;
    this->_velocity = rbm._velocity;
    this->_trajectory = rbm._trajectory;
    this->_supporting_features_ids = rbm._supporting_features_ids;
    this->_features_database = rbm.getFeaturesDatabase();
    this->_predicted_measurement_pc_bh = rbm._predicted_measurement_pc_bh;
    this->_predicted_measurement_pc_vh = rbm._predicted_measurement_pc_vh;
    this->_predicted_measurement_map_bh = rbm._predicted_measurement_map_bh;
    this->_predicted_measurement_map_vh = rbm._predicted_measurement_map_vh;
    this->_predicted_pose_bh = rbm._predicted_pose_bh;
    this->_predicted_pose_vh = rbm._predicted_pose_vh;
    this->_min_num_supporting_feats_to_correct = rbm._min_num_supporting_feats_to_correct;
}

RBFilter::~RBFilter()
{
}

void RBFilter::predictState(double time_interval_ns)
{
    //Sanity check.
    if(time_interval_ns < 0)
    {
        std::cout << "Time interval negative!" << std::endl;
        this->_last_time_interval_ns = time_interval_ns;
        getchar();
    }

    // Estimate the new cov matrix depending on the time elapsed between the previous and the current measurement
    Eigen::Matrix<double, 6, 6> sys_noise_COV = Eigen::Matrix<double, 6, 6>::Zero();
    sys_noise_COV(0, 0) = this->_cov_sys_acc_tx;//* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(1, 1) = this->_cov_sys_acc_ty;//* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(2, 2) = this->_cov_sys_acc_tz;//* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(3, 3) = this->_cov_sys_acc_rx;//* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(4, 4) = this->_cov_sys_acc_ry;//* (std::pow((time_interval_ns/1e9),3) / 3.0);
    sys_noise_COV(5, 5) = this->_cov_sys_acc_rz;//* (std::pow((time_interval_ns/1e9),3) / 3.0);

    // We update the pose with a non-linear rule:
    // The predicted pose is the belief pose pre-multiplied by the corresponding
    // delta in pose based on the belief velocity and the time elapsed
    // This is slightly different to just adding the belief pose and the
    // belief delta pose in exponential coordinates, but the difference is small
    // because the delta pose is small when the velocity is small (it is the first linear approximation)
    Eigen::Twistd belief_delta_pose_ec = (time_interval_ns/1e9)*_velocity;
    _predicted_pose_vh = belief_delta_pose_ec.exp(1e-12).toHomogeneousMatrix()*_pose;

    // We also update the covariance based on Barfoot et al.
    Eigen::Matrix<double, 6, 6> delta_pose_covariance = (time_interval_ns/1e9)*_velocity_covariance;

    Eigen::Twistd pose_ec;
    TransformMatrix2Twist(_pose, pose_ec);
    Eigen::Matrix<double,6,6> adjoint = pose_ec.exp(1e-12).adjoint();

    // pose_covariance_updated is 6x6
    _predicted_pose_cov_vh = _pose_covariance + adjoint*delta_pose_covariance*adjoint.transpose();

    // Finally we add the system noise
    _predicted_pose_cov_vh += sys_noise_COV;

    // The additive noise we use in the filter based on braking hypothesis is larger
    this->_predicted_pose_bh = _pose;
    this->_predicted_velocity_bh = Eigen::Twistd(0,0,0,0,0,0);
    this->_predicted_pose_cov_bh = _pose_covariance + sys_noise_COV;
    this->_predicted_velocity_cov_bh =  Eigen::Matrix<double, 6, 6>::Zero();
}

// This only predicts the location of the Features that are supporting the RBFilter
// at this moment.
// This happens at the end of the iteration
void RBFilter::predictMeasurement()
{
    this->_predicted_measurement_pc_vh->points.clear();
    this->_predicted_measurement_map_vh.clear();
    this->_predicted_measurement_pc_bh->points.clear();
    this->_predicted_measurement_map_bh.clear();
    this->_predicted_measurement_pc_kh->points.clear();
    this->_predicted_measurement_map_kh.clear();

    // We need to do this outside because if we do not have any features we still want to use the predictions from kh
    if(_use_predicted_state_from_kh)
    {
        this->_use_predicted_measurement_from_kh = true;
    }

    Feature::Location predicted_location;
    FeaturePCLwc predicted_location_pcl;
    Feature::Ptr supporting_feat_ptr;
    BOOST_FOREACH(Feature::Id supporting_feat_id, this->_supporting_features_ids)
    {
        if(this->_features_database->isFeatureStored(supporting_feat_id))
        {
            supporting_feat_ptr = this->_features_database->getFeature(supporting_feat_id);

            // Predict feature location based on velocity hypothesis
            this->PredictFeatureLocation(supporting_feat_ptr,this->_predicted_pose_vh, true, predicted_location);
            LocationAndId2FeaturePCLwc(predicted_location, supporting_feat_id, predicted_location_pcl);
            this->_predicted_measurement_pc_vh->points.push_back(predicted_location_pcl);
            this->_predicted_measurement_map_vh[supporting_feat_id] = predicted_location;

            // Predict feature location based on brake hypothesis (same as last)
            this->PredictFeatureLocation(supporting_feat_ptr,this->_predicted_pose_bh, true, predicted_location);
            LocationAndId2FeaturePCLwc(predicted_location, supporting_feat_id, predicted_location_pcl);
            this->_predicted_measurement_pc_bh->points.push_back(predicted_location_pcl);
            this->_predicted_measurement_map_bh[supporting_feat_id] = predicted_location;

            if(_use_predicted_state_from_kh)
            {
                // Predict feature location based on kinematic hypothesis
                this->PredictFeatureLocation(supporting_feat_ptr,this->_predicted_pose_kh, true, predicted_location);
                LocationAndId2FeaturePCLwc(predicted_location, supporting_feat_id, predicted_location_pcl);
                this->_predicted_measurement_pc_kh->points.push_back(predicted_location_pcl);
                this->_predicted_measurement_map_kh[supporting_feat_id] = predicted_location;
                this->_use_predicted_state_from_kh = false;
            }
        }
    }
}

void RBFilter::correctState()
{

    // Resize matrices
    int num_supporting_feats = (int)this->_supporting_features_ids.size();

    if(num_supporting_feats > this->_min_num_supporting_feats_to_correct)
    {
        int meas_dimension = 3 * num_supporting_feats;

        double feature_depth = 0.;

        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > G_t((this->_G_t_memory.data()), 6, meas_dimension);
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > R_inv_times_innovation((this->_R_inv_times_innovation_memory.data()), meas_dimension, 1);

        // We need to set them to zero because they are pointing to existing memory regions!
        G_t.setZero();
        R_inv_times_innovation.setZero();

        Feature::Location location_at_birth;

        int feat_idx = 0;

        Eigen::Matrix3d R_seq = Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 6, 6> P_minus_seq = _predicted_pose_covariance;

        //std::cout << _predicted_pose_covariance << std::endl;
        Eigen::Matrix<double, 6, 3> P_HT_seq = Eigen::Matrix<double, 6, 3>::Zero();
        Eigen::Matrix3d S_seq = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d S_inv_seq = Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 6, 3> K_seq = Eigen::Matrix<double, 6, 3>::Zero();

        Eigen::Matrix<double, 6, 6> temp_seq1 = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> temp_seq2 = Eigen::Matrix<double, 6, 6>::Zero();

        Eigen::Vector4d feature_innovation;
        Eigen::Vector4d location_at_birth_eig;

        Eigen::Vector4d predicted_location;
        Eigen::Vector4d current_location;

        Feature::Ptr supporting_feat_ptr;

        BOOST_FOREACH(Feature::Id supporting_feat_id, this->_supporting_features_ids)
        {
            supporting_feat_ptr = this->_features_database->getFeature(supporting_feat_id);

            if(supporting_feat_ptr->getFeatureAge() > 1)
                this->GetLocationOfFeatureAtBirthOfRB( supporting_feat_ptr, false, location_at_birth);
            LocationOfFeature2EigenVectorHomogeneous(location_at_birth, location_at_birth_eig);

            Feature::Location current_loc = supporting_feat_ptr->getLastLocation();
            LocationOfFeature2EigenVectorHomogeneous(current_loc, current_location);

            predicted_location = _predicted_pose*location_at_birth_eig;

            feature_innovation = current_location - predicted_location;

            feature_depth = supporting_feat_ptr->getLastZ();

            R_seq( 0, 0) =  std::max(this->_min_cov_meas_x, feature_depth / this->_meas_depth_factor);
            R_seq( 1, 1) =  std::max(this->_min_cov_meas_y, feature_depth / this->_meas_depth_factor);
            R_seq( 2, 2) =  std::max(this->_min_cov_meas_z, feature_depth / this->_meas_depth_factor);

            R_inv_times_innovation(3 * feat_idx ) = feature_innovation.x()/(std::max(this->_min_cov_meas_x, feature_depth / this->_meas_depth_factor));
            R_inv_times_innovation(3 * feat_idx + 1) = feature_innovation.y()/(std::max(this->_min_cov_meas_y, feature_depth / this->_meas_depth_factor));
            R_inv_times_innovation(3 * feat_idx + 2) = feature_innovation.z()/(std::max(this->_min_cov_meas_z, feature_depth / this->_meas_depth_factor));

            Eigen::Vector4d predicted_location_feat = _predicted_pose*location_at_birth_eig;

            _D_T_p0_circle(0,4) = predicted_location_feat[2];
            _D_T_p0_circle(0,5) = -predicted_location_feat[1];
            _D_T_p0_circle(1,3) = -predicted_location_feat[2];
            _D_T_p0_circle(1,5) = predicted_location_feat[0];
            _D_T_p0_circle(2,3) = predicted_location_feat[1];
            _D_T_p0_circle(2,4) = -predicted_location_feat[0];

            G_t.block<6,3>(0, 3*feat_idx) = _D_T_p0_circle.transpose();

            P_HT_seq = P_minus_seq * _D_T_p0_circle.transpose();

            S_seq = _D_T_p0_circle * P_HT_seq;

            S_seq += R_seq;

            invert3x3MatrixEigen2(S_seq, S_inv_seq );

            K_seq = P_HT_seq * S_inv_seq;

            temp_seq1 = K_seq*_D_T_p0_circle;

            temp_seq2 = temp_seq1 * P_minus_seq;

            P_minus_seq -= temp_seq2;

            // copy elements
            for ( unsigned int i=0; i<P_minus_seq.rows(); i++ )
            {
                for ( unsigned int j=0; j<=i; j++ )
                {
                    P_minus_seq(j,i) = P_minus_seq(i,j);
                }
            }

            feat_idx++;
        }

        _pose_covariance = P_minus_seq;

        Eigen::Matrix<double, 6, 1> pose_update = P_minus_seq * G_t * R_inv_times_innovation;

        Eigen::Twistd pose_update_ec(pose_update[3], pose_update[4], pose_update[5], pose_update[0], pose_update[1], pose_update[2]);
        _pose = pose_update_ec.exp(1e-12).toHomogeneousMatrix()*_predicted_pose;
    }else{
        ROS_ERROR_STREAM_NAMED("RBFilter::correctState","RB" << this->_id << ": using predicted state as corrected state!");
        _pose = _predicted_pose;
        _pose_covariance = _predicted_pose_covariance;
    }

    Eigen::Matrix4d delta_pose = _pose*_trajectory.at(_trajectory.size()-1).inverse();

    Eigen::Twistd delta_pose_ec;
    TransformMatrix2Twist(delta_pose, delta_pose_ec);
    this->_velocity = delta_pose_ec/(_last_time_interval_ns/1e9);
    this->_trajectory.push_back(this->_pose);
}

bool isPredictionYBetterThanX(boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> > X, boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> > Y)
{
    // If the number of supporting features based on X is smaller than based on Y
    // then YBetterThanX = true
    if(boost::tuples::get<0>(X) < boost::tuples::get<0>(Y))
    {
        return true;
    }
    // If the number of supporting features based on X is larger than based on Y
    // then YBetterThanX = false
    else if(boost::tuples::get<0>(X) > boost::tuples::get<0>(Y))
    {
        return false;
    }
    // If the number of supporting features based on X is the same as based on Y
    // then we check the accumulated error
    else{
        // If the accumulated error based on X is larger than based on Y
        // then YBetterThanX = true
        if(boost::tuples::get<1>(X) > boost::tuples::get<1>(Y))
        {
            return true;
        }
        // If the accumulated error based on X is smaller than based on Y
        // then YBetterThanX = false
        else if(boost::tuples::get<1>(X) < boost::tuples::get<1>(Y))
        {
            return false;
        }
        // If the accumulated error based on X is the same as based on Y
        // then we check the covariances
        else
        {
            int num_larger_x_covariance = 0;
            for(int i =0; i<6; i++)
            {
                if(boost::tuples::get<2>(X)(i,i) > boost::tuples::get<2>(Y)(i,i))
                {
                    num_larger_x_covariance +=1;
                }
            }
            if(num_larger_x_covariance > 3)
            {
                return true;
            }
            else{
                return false;
            }
        }
    }
}

//TODO: Decide if I want to test ALL features or only the previously supporting features
//NOW: Only supporting features
// This function is called when there are new Features added
void RBFilter::estimateBestPredictionAndSupportingFeatures()
{
    // 1: Update the list of supporting features to only contain features that are still alive
    std::vector<Feature::Id> alive_supporting_features_ids;
    BOOST_FOREACH(Feature::Id supporting_feat_id, this->_supporting_features_ids)
    {
        if(this->_features_database->isFeatureStored(supporting_feat_id))
        {
            alive_supporting_features_ids.push_back(supporting_feat_id);
        }
    }
    this->_supporting_features_ids.swap(alive_supporting_features_ids);

    std::vector<boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> > > num_supporting_feats_and_error_and_pose_cov;

    // 2: Count the supporting features by comparing the received feature location to the predicted location based on each
    // hypothesis (velocity based, brake event based, kinematic state based)
    std::vector<Feature::Id> supporting_feats_based_on_vh;
    std::vector<Feature::Id> supporting_feats_based_on_bh;
    std::vector<Feature::Id> supporting_feats_based_on_kh;
    double error_vh = 0.0;
    double error_bh = 0.0;
    double error_kh = 0.0;
    double acc_error_vh = 0.0;
    double acc_error_bh = 0.0;
    double acc_error_kh = 0.0;
    BOOST_FOREACH(Feature::Id supporting_feat_id, this->_supporting_features_ids)
    {
        Feature::Location last_location = this->_features_database->getFeatureLastLocation(supporting_feat_id);

        Feature::Location predicted_location_vh = this->_predicted_measurement_map_vh[supporting_feat_id];
        error_vh = L2Distance(last_location, predicted_location_vh);
        if (error_vh < this->_estimation_error_threshold)
        {
            supporting_feats_based_on_vh.push_back(supporting_feat_id);
            acc_error_vh += error_vh;
        }

        Feature::Location predicted_location_bh = this->_predicted_measurement_map_bh[supporting_feat_id];
        error_bh = L2Distance(last_location, predicted_location_bh);
        if (error_bh < this->_estimation_error_threshold)
        {
            supporting_feats_based_on_bh.push_back(supporting_feat_id);
            acc_error_bh += error_bh;
        }

        // If we had a predicted state from kinematics that we used to predict measurements
        if(this->_use_predicted_measurement_from_kh)
        {
            Feature::Location predicted_location_kh = this->_predicted_measurement_map_kh[supporting_feat_id];
            error_kh = L2Distance(last_location, predicted_location_kh);
            if (error_kh < this->_estimation_error_threshold)
            {
                supporting_feats_based_on_kh.push_back(supporting_feat_id);
                acc_error_kh += error_kh;
            }
        }
    }

    num_supporting_feats_and_error_and_pose_cov.push_back(boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> >(supporting_feats_based_on_vh.size(), acc_error_vh, _predicted_pose_cov_vh));
    num_supporting_feats_and_error_and_pose_cov.push_back(boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> >(supporting_feats_based_on_bh.size(), acc_error_bh, _predicted_pose_cov_bh));

    // If we had a predicted state from kinematics that we used to predict measurements
    if(this->_use_predicted_measurement_from_kh)
    {
        num_supporting_feats_and_error_and_pose_cov.push_back(boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> >(supporting_feats_based_on_kh.size(), acc_error_kh, _predicted_pose_cov_kh));
        this->_use_predicted_measurement_from_kh = false;
    }
    // Find the best hypothesis based on the number of supporting features and the accumulated error of them
    // The criteria to select the best hypothesis is in the function ComparePredictions
    std::vector<boost::tuple<size_t, double, Eigen::Matrix<double, 6, 6> > >::iterator hyp_with_max_support_it =
            std::max_element(num_supporting_feats_and_error_and_pose_cov.begin(),
                             num_supporting_feats_and_error_and_pose_cov.end(),
                             isPredictionYBetterThanX);
    int hyp_with_max_support_idx = std::distance(num_supporting_feats_and_error_and_pose_cov.begin(), hyp_with_max_support_it);

    switch(hyp_with_max_support_idx)
    {

    case 0:
    {
        std::cout << "RB" << _id<<" Using velocity prediction" << std::endl;
        this->_predicted_pose = _predicted_pose_vh;
        this->_predicted_pose_covariance = _predicted_pose_cov_vh;
        this->_supporting_features_ids = supporting_feats_based_on_vh;
        this->_predicted_measurement = this->_predicted_measurement_pc_vh;
        this->_predicted_measurement_map = this->_predicted_measurement_map_vh;
        break;
    }

    case 1:
    {
        std::cout << "RB" << _id<<" Using brake prediction" << std::endl;
        this->_predicted_pose = _predicted_pose_bh;
        this->_predicted_pose_covariance = _predicted_pose_cov_bh;
        this->_supporting_features_ids = supporting_feats_based_on_bh;
        this->_predicted_measurement = this->_predicted_measurement_pc_bh;
        this->_predicted_measurement_map = this->_predicted_measurement_map_bh;
        break;
    }

    case 2:
    {
        ROS_ERROR_STREAM_NAMED("RBFilter.estimateBestPredictionAndSupportingFeatures",
                               "RB" << _id << " Using prediction from higher level!");
        this->_predicted_pose = _predicted_pose_kh;
        this->_predicted_pose_covariance = _predicted_pose_cov_kh;
        this->_supporting_features_ids = supporting_feats_based_on_kh;
        this->_predicted_measurement = this->_predicted_measurement_pc_kh;
        this->_predicted_measurement_map = this->_predicted_measurement_map_kh;
        break;
    }
    default:
        break;
    }
}

void RBFilter::addSupportingFeature(Feature::Id supporting_feat_id)
{
    this->_supporting_features_ids.push_back(supporting_feat_id);
}

void RBFilter::setPredictedState(const omip_msgs::RigidBodyPoseAndVelMsg hypothesis)
{
    this->_use_predicted_state_from_kh = true;

    Eigen::Twistd predicted_pose_kh_ec(hypothesis.pose_wc.twist.angular.x,
                                                   hypothesis.pose_wc.twist.angular.y,
                                                   hypothesis.pose_wc.twist.angular.z,
                                                   hypothesis.pose_wc.twist.linear.x,
                                                   hypothesis.pose_wc.twist.linear.y,
                                                   hypothesis.pose_wc.twist.linear.z);
    this->_predicted_pose_kh = predicted_pose_kh_ec.exp(1e-12).toHomogeneousMatrix();

    this->_predicted_velocity_kh = Eigen::Twistd(hypothesis.velocity_wc.twist.angular.x,
                                                 hypothesis.velocity_wc.twist.angular.y,
                                                 hypothesis.velocity_wc.twist.angular.z,
                                                 hypothesis.velocity_wc.twist.linear.x,
                                                 hypothesis.velocity_wc.twist.linear.y,
                                                 hypothesis.velocity_wc.twist.linear.z);
    for(int i =0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            this->_predicted_pose_cov_kh(i,j) = hypothesis.pose_wc.covariance[6*i + j];
            this->_predicted_velocity_cov_kh(i,j) = hypothesis.velocity_wc.covariance[6*i + j];
        }
    }
}

void RBFilter::integrateShapeBasedPose(const geometry_msgs::TwistWithCovariance twist_refinement, double pose_time_elapsed_ns)
{
    Eigen::Twistd shape_based_pose_ec = Eigen::Twistd(twist_refinement.twist.angular.x, twist_refinement.twist.angular.y, twist_refinement.twist.angular.z,
                                                  twist_refinement.twist.linear.x, twist_refinement.twist.linear.y, twist_refinement.twist.linear.z);

    if(pose_time_elapsed_ns < this->_loop_period_ns || _supporting_features_ids.size() < _min_num_supporting_feats_to_correct)
    {
        ROS_INFO_STREAM_NAMED("RBFilter.integrateShapeBasedPose", "Using shape-based pose (time elapsed=" << pose_time_elapsed_ns/1e9 <<
                              " s, loop_period="<< _loop_period_ns/1e9 << " s)");
        //ROS_INFO_STREAM_NAMED("RBFilter.integrateShapeBasedPose", "Pose before: " << std::endl <<  this->_pose);

        this->_pose = shape_based_pose_ec.exp(1e-12).toHomogeneousMatrix();
        //ROS_INFO_STREAM_NAMED("RBFilter.integrateShapeBasedPose", "Pose after: " << std::endl << this->_pose);

        this->_trajectory.pop_back();
        this->_trajectory.push_back(this->_pose);

        Eigen::Matrix4d delta_pose_ht = this->_trajectory[this->_trajectory.size()-1]*(this->_trajectory[this->_trajectory.size()-2].inverse());
        Eigen::Twistd delta_pose_ec;
        TransformMatrix2Twist(delta_pose_ht, delta_pose_ec);
        this->_velocity = delta_pose_ec/(this->_loop_period_ns/1e9);
    }else{
        ROS_INFO_STREAM_NAMED("RBFilter.integrateShapeBasedPose", "Shape-based pose is too old (time elapsed=" << pose_time_elapsed_ns/1e9 <<
                              " s, loop_period="<< _loop_period_ns/1e9 << " s). Try reducing the framerate");
    }
}

void RBFilter::addPredictedFeatureLocation(const Feature::Location& predicted_location_velocity,const Feature::Location& predicted_location_brake , const Feature::Id feature_id)
{
    FeaturePCLwc predicted_feature_pcl;

    LocationAndId2FeaturePCLwc(predicted_location_velocity, feature_id, predicted_feature_pcl);
    this->_predicted_measurement->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map[feature_id] = predicted_location_velocity;

    this->_predicted_measurement_pc_vh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_vh[feature_id] = predicted_location_velocity;

    this->_predicted_measurement_pc_kh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_kh[feature_id] = predicted_location_brake;

    LocationAndId2FeaturePCLwc(predicted_location_brake, feature_id, predicted_feature_pcl);
    this->_predicted_measurement_pc_bh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_bh[feature_id] = predicted_location_brake;
}

void RBFilter::setFeaturesDatabase(FeaturesDataBase::Ptr feats_database)
{
    this->_features_database = feats_database;
}

void RBFilter::PredictFeatureLocation(Feature::Ptr feature,
                                      const Eigen::Matrix4d& predicted_pose,
                                      bool contemporary_last_feat_loc_and_last_pose_in_trajectory,
                                      Feature::Location& predicted_location,
                                      bool publish_locations) const
{
    Feature::Location location_at_birth_of_rb;
    this->GetLocationOfFeatureAtBirthOfRB(feature, contemporary_last_feat_loc_and_last_pose_in_trajectory, location_at_birth_of_rb);


    TransformLocation(location_at_birth_of_rb, predicted_pose, predicted_location);

    if(publish_locations)
    {
        if(this->_id != 0)
        {
            Feature::Location at_birth_location = location_at_birth_of_rb + _initial_location_of_centroid;
            boost::shared_ptr<FeaturePCLwc> at_birth_feat(new FeaturePCLwc());
            at_birth_feat->x = boost::tuples::get<0>(at_birth_location);
            at_birth_feat->y = boost::tuples::get<1>(at_birth_location);
            at_birth_feat->z = boost::tuples::get<2>(at_birth_location);
            at_birth_feat->label = feature->getId();

            _features_at_birth->points.push_back(*at_birth_feat);
        }else{
            boost::shared_ptr<FeaturePCLwc> at_birth_feat(new FeaturePCLwc());
            at_birth_feat->x = boost::tuples::get<0>(location_at_birth_of_rb);
            at_birth_feat->y = boost::tuples::get<1>(location_at_birth_of_rb);
            at_birth_feat->z = boost::tuples::get<2>(location_at_birth_of_rb);
            at_birth_feat->label = feature->getId();

            _features_at_birth->points.push_back(*at_birth_feat);
        }
        boost::shared_ptr<FeaturePCLwc> predicted_feat(new FeaturePCLwc());
        predicted_feat->x = boost::tuples::get<0>(predicted_location);
        predicted_feat->y = boost::tuples::get<1>(predicted_location);
        predicted_feat->z = boost::tuples::get<2>(predicted_location);
        predicted_feat->label = feature->getId();
        _features_predicted->points.push_back(*predicted_feat);
    }
}

void RBFilter::GetLocationOfFeatureAtBirthOfRB(Feature::Ptr feature,
                                               bool contemporary_last_feat_loc_and_last_pose_in_trajectory,
                                               Feature::Location& location_at_birth) const
{
    const Feature::Trajectory& feat_traj = feature->getTrajectory();

    // Feature age counts the last step (this)
    size_t feature_age = feature->getFeatureAge();
    size_t rigid_body_age = this->_trajectory.size();

    // If the last feature location and the last pose are not contemporary (because we are at the beginning of a loop and we have added the new
    // feature locations to the database but we do not added a new trajectory because that only happens after the correction step), then we add
    // one to the rigid_body_age
    if(!contemporary_last_feat_loc_and_last_pose_in_trajectory)
    {
        rigid_body_age += 1;
    }

    // Feature is younger than RB -> Two options:
    //    - Estimate the motion between feat birthday and current frame as the transformation between RB's birthday
    //    and current "minus" the transformation between RB's birthday and feat's birthday
    //    - Move the initial location of the feature to the RB's birthday using the inverse of the transform between
    //    RB's birthday and feat's birthday. Then move this location to current
    // Using second option
    if (feature_age < rigid_body_age + 1)
    {
        Eigen::Matrix4d transform_from_rb_birth_to_feat_birth = this->_trajectory.at(rigid_body_age - feature_age);

        // This moves the feature from its initial location to where it would be when the rb was born
        Eigen::Matrix4d inverse_transform_from_rb_birth_to_feat_birth = transform_from_rb_birth_to_feat_birth.inverse();
        TransformLocation(feat_traj.at(0), inverse_transform_from_rb_birth_to_feat_birth, location_at_birth);
    }
    else
    {
        location_at_birth = feat_traj.at(feature_age - (rigid_body_age + 1));
        if(this->_id != 0)
        {
            location_at_birth = location_at_birth - _initial_location_of_centroid;
        }
    }


}

RB_id_t RBFilter::getId() const
{
    return this->_id;
}

Eigen::Matrix4d RBFilter::getPose() const
{
    return _pose;
}

Eigen::Matrix<double, 6, 6> RBFilter::getPoseCovariance() const
{
    return _pose_covariance;
}

Eigen::Twistd RBFilter::getVelocity() const
{
    return _velocity;
}

Eigen::Matrix<double, 6, 6> RBFilter::getVelocityCovariance() const
{
    return _velocity_covariance;
}

geometry_msgs::PoseWithCovariancePtr RBFilter::getPoseWithCovariance() const
{
    geometry_msgs::PoseWithCovariancePtr pose_wc_ptr = geometry_msgs::PoseWithCovariancePtr(new geometry_msgs::PoseWithCovariance());
    pose_wc_ptr->pose.position.x = _pose(0,3);
    pose_wc_ptr->pose.position.y = _pose(1,3);
    pose_wc_ptr->pose.position.z = _pose(2,3);
    Eigen::Quaterniond quaternion(_pose.block<3,3>(0,0));
    pose_wc_ptr->pose.orientation.x = quaternion.x();
    pose_wc_ptr->pose.orientation.y = quaternion.y();
    pose_wc_ptr->pose.orientation.z = quaternion.z();
    pose_wc_ptr->pose.orientation.w = quaternion.w();

    for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
            pose_wc_ptr->covariance[6 * i + j] = _pose_covariance(i, j);

    return pose_wc_ptr;
}

geometry_msgs::TwistWithCovariancePtr RBFilter::getPoseECWithCovariance() const
{
    geometry_msgs::TwistWithCovariancePtr pose_wc_ptr = geometry_msgs::TwistWithCovariancePtr(new geometry_msgs::TwistWithCovariance());
    Eigen::Twistd pose_ec;
    TransformMatrix2Twist(_pose, pose_ec);

    pose_wc_ptr->twist.linear.x = pose_ec.vx();
    pose_wc_ptr->twist.linear.y = pose_ec.vy();
    pose_wc_ptr->twist.linear.z = pose_ec.vz();
    pose_wc_ptr->twist.angular.x = pose_ec.rx();
    pose_wc_ptr->twist.angular.y = pose_ec.ry();
    pose_wc_ptr->twist.angular.z = pose_ec.rz();

    for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
            pose_wc_ptr->covariance[6 * i + j] = _pose_covariance(i, j);

    return pose_wc_ptr;
}

geometry_msgs::TwistWithCovariancePtr RBFilter::getVelocityWithCovariance() const
{
    // Error in the library. I cannot assign to rx, ry and rz if the function is const
    Eigen::Twistd velocity_copy = _velocity;
    geometry_msgs::TwistWithCovariancePtr vel_t_wc_ptr = geometry_msgs::TwistWithCovariancePtr(new geometry_msgs::TwistWithCovariance());
    vel_t_wc_ptr->twist.linear.x = _velocity.vx();
    vel_t_wc_ptr->twist.linear.y = _velocity.vy();
    vel_t_wc_ptr->twist.linear.z = _velocity.vz();
    vel_t_wc_ptr->twist.angular.x = velocity_copy.rx();
    vel_t_wc_ptr->twist.angular.y = velocity_copy.ry();
    vel_t_wc_ptr->twist.angular.z = velocity_copy.rz();

    for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 6; j++)
            vel_t_wc_ptr->covariance[6 * i + j] = _velocity_covariance(i, j);

    return vel_t_wc_ptr;
}

std::vector<Eigen::Matrix4d> RBFilter::getTrajectory() const
{
    return this->_trajectory;
}

FeatureCloudPCLwc RBFilter::getPredictedMeasurement(PredictionHypothesis hypothesis)
{
    FeatureCloudPCLwc predicted_measurement;
    switch(hypothesis)
    {
    case BASED_ON_VELOCITY:
        predicted_measurement = *(this->_predicted_measurement_pc_vh);
        break;
    case BASED_ON_BRAKING_EVENT:
        predicted_measurement = *(this->_predicted_measurement_pc_bh);
        break;
    default:
        ROS_ERROR_STREAM_NAMED("RBFilter::getPredictedMeasurement", "Wrong hypothesis. Value " << hypothesis);
        break;
    }
    return predicted_measurement;
}

std::vector<Feature::Id> RBFilter::getSupportingFeatures() const
{
    return this->_supporting_features_ids;
}

int RBFilter::getNumberSupportingFeatures() const
{
    return (int)this->_supporting_features_ids.size();
}

FeaturesDataBase::Ptr RBFilter::getFeaturesDatabase() const
{
    return this->_features_database;
}

void RBFilter::_initializeAuxiliarMatrices()
{
    _pose_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    _velocity_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; i++)
    {
        _pose_covariance(i, i) = this->_prior_cov_pose;
        _velocity_covariance(i, i) = this->_prior_cov_vel;
    }

    this->_predicted_measurement_pc_vh = FeatureCloudPCLwc::Ptr(new FeatureCloudPCLwc());
    this->_predicted_measurement_pc_bh = FeatureCloudPCLwc::Ptr(new FeatureCloudPCLwc());
    this->_predicted_measurement_pc_kh = FeatureCloudPCLwc::Ptr(new FeatureCloudPCLwc());
    this->_features_at_birth= FeatureCloudPCLwc::Ptr(new FeatureCloudPCLwc());
    this->_features_predicted = FeatureCloudPCLwc::Ptr(new FeatureCloudPCLwc());
    this->_predicted_pose_vh = Eigen::Matrix4d::Identity();
    this->_predicted_pose_bh = Eigen::Matrix4d::Identity();
    this->_predicted_pose_kh = Eigen::Matrix4d::Identity();

    _G_t_memory = Eigen::MatrixXd(6, 3*_num_tracked_feats);
    _R_inv_times_innovation_memory = Eigen::VectorXd(3*_num_tracked_feats);

    _D_T_p0_circle = Eigen::Matrix<double, 3, 6>::Zero();
    _D_T_p0_circle.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    _D_T_p0_circle.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
}
