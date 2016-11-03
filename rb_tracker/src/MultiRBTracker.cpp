#include <boost/foreach.hpp>

#include "rb_tracker/MultiRBTracker.h"

#include "omip_common/OMIPUtils.h"

#include <cmath>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl_conversions/pcl_conversions.h>

#if ROS_VERSION_MINIMUM(1, 11, 1) // if current ros version is >= 1.11.1 (indigo)
#else
#include "rb_tracker/pcl_conversions_indigo.h"
#endif

#include <pcl/console/print.h>

using namespace omip;

inline void estimateRigidTransformation(
        const FeaturesDataBase::MapOfFeatureLocationPairs& last2locations,
        Eigen::Matrix4d &transformation_matrix)
{
    // Convert to Eigen format
    const int npts = static_cast<int>(last2locations.size());

    Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_src(3, npts);
    Eigen::Matrix<double, 3, Eigen::Dynamic> cloud_tgt(3, npts);

    FeaturesDataBase::MapOfFeatureLocationPairs::const_iterator locations_it =
            last2locations.begin();
    FeaturesDataBase::MapOfFeatureLocationPairs::const_iterator locations_it_end =
            last2locations.end();
    for (int i = 0; locations_it != locations_it_end; locations_it++, i++)
    {
        cloud_src(0, i) = locations_it->second.second.get<0>();
        cloud_src(1, i) = locations_it->second.second.get<1>();
        cloud_src(2, i) = locations_it->second.second.get<2>();

        cloud_tgt(0, i) = locations_it->second.first.get<0>();
        cloud_tgt(1, i) = locations_it->second.first.get<1>();
        cloud_tgt(2, i) = locations_it->second.first.get<2>();
    }

    // Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
    transformation_matrix = Eigen::umeyama(cloud_src, cloud_tgt, false);
}

MultiRBTracker::MultiRBTracker(int max_num_rb,
                               double loop_period_ns,
                               int ransac_iterations,
                               double estimation_error_threshold,
                               double static_motion_threshold,
                               double new_rbm_error_threshold,
                               double max_error_to_reassign_feats,
                               int supporting_features_threshold,
                               int min_num_feats_for_new_rb,
                               int min_num_frames_for_new_rb,
                               int initial_cam_motion_constraint,
                               static_environment_tracker_t static_environment_tracker_type) :
    RecursiveEstimatorFilterInterface(loop_period_ns),
    _max_num_rb(max_num_rb),
    _ransac_iterations(ransac_iterations),
    _estimation_error_threshold(estimation_error_threshold),
    _static_motion_threshold(static_motion_threshold),
    _new_rbm_error_threshold(new_rbm_error_threshold),
    _max_error_to_reassign_feats(max_error_to_reassign_feats),
    _supporting_features_threshold(supporting_features_threshold),
    _min_num_feats_for_new_rb(min_num_feats_for_new_rb),
    _min_num_frames_for_new_rb(min_num_frames_for_new_rb),
    _min_num_points_in_segment(0),
    _min_probabilistic_value(0.0),
    _max_fitness_score(0.0),
    _min_num_supporting_feats_to_correct(7),
    _max_distance_ee(-1),
    _max_distance_irb(-1)
{

    //_really_free_feats_file.open("really_free_feats_file.txt");
    this->_filter_name = "RBMFilter";

    if (this->_features_db)
        this->_features_db.reset();

    this->_features_db = FeaturesDataBase::Ptr(new FeaturesDataBase());

    this->_predicted_measurement = rbt_measurement_t(new FeatureCloudPCLwc());

    // The static environment exists always as initial hypothesis
    // We can track the static enviroment (visual odometry) using different computation methods: ICP or EKF
    _static_environment_filter = StaticEnvironmentFilter::Ptr(new StaticEnvironmentFilter(this->_loop_period_ns,
                                                                                      this->_features_db,
                                                                                      this->_static_motion_threshold));
    _static_environment_filter->setComputationType(static_environment_tracker_type);
    _static_environment_filter->setFeaturesDatabase(this->_features_db);
    _static_environment_filter->setMotionConstraint(initial_cam_motion_constraint);

    this->_kalman_filters.push_back(_static_environment_filter);
}

void MultiRBTracker::Init()
{
    _static_environment_filter->setMeasurementDepthFactor(this->_meas_depth_factor);
    _static_environment_filter->setMinCovarianceMeasurementX(this->_min_cov_meas_x);
    _static_environment_filter->setMinCovarianceMeasurementY(this->_min_cov_meas_y);
    _static_environment_filter->setMinCovarianceMeasurementZ(this->_min_cov_meas_z);
    _static_environment_filter->setPriorCovariancePose(this->_prior_cov_pose);
    _static_environment_filter->setPriorCovarianceVelocity(this->_prior_cov_vel);
    _static_environment_filter->setCovarianceSystemAccelerationTx(this->_cov_sys_acc_tx);
    _static_environment_filter->setCovarianceSystemAccelerationTy(this->_cov_sys_acc_ty);
    _static_environment_filter->setCovarianceSystemAccelerationTz(this->_cov_sys_acc_tz);
    _static_environment_filter->setCovarianceSystemAccelerationRx(this->_cov_sys_acc_rx);
    _static_environment_filter->setCovarianceSystemAccelerationRy(this->_cov_sys_acc_ry);
    _static_environment_filter->setCovarianceSystemAccelerationRz(this->_cov_sys_acc_rz);
    _static_environment_filter->setNumberOfTrackedFeatures(this->_num_tracked_feats);
    _static_environment_filter->setMinNumberOfSupportingFeaturesToCorrectPredictedState(this->_min_num_supporting_feats_to_correct);
    _static_environment_filter->Init();
}

MultiRBTracker::~MultiRBTracker()
{

}

void MultiRBTracker::setMeasurement(rbt_measurement_t acquired_measurement, const double& measurement_timestamp)
{
    this->_previous_measurement_timestamp_ns = this->_measurement_timestamp_ns;

    {
        boost::mutex::scoped_lock(this->_measurement_timestamp_ns_mutex);
        this->_measurement_timestamp_ns = measurement_timestamp;
    }

    this->_features_db->clearListOfAliveFeatureIds();
    this->_measurement = acquired_measurement;

    int num_tracked_features = acquired_measurement->points.size();

    int tracked_features_index = 0;
    uint32_t feature_id;
    for (; tracked_features_index < num_tracked_features; tracked_features_index++)
    {
        feature_id = acquired_measurement->points[tracked_features_index].label;
        if(feature_id != 0)
        {
            Feature::Location feature_location = Feature::Location(acquired_measurement->points[tracked_features_index].x, acquired_measurement->points[tracked_features_index].y,
                                                                   acquired_measurement->points[tracked_features_index].z);
            this->addFeatureLocation(feature_id, feature_location);
        }
    }
    this->_features_db->step();
}

void MultiRBTracker::predictState(double time_interval_ns)
{
    //First we update the filters using the internal state (system update)
    for (std::vector<RBFilter::Ptr>::iterator filter_it = this->_kalman_filters.begin(); filter_it != this->_kalman_filters.end(); filter_it++)
    {
        (*filter_it)->predictState(time_interval_ns);
        (*filter_it)->doNotUsePredictionFromHigherLevel();
    }
}

// NOTE: The location of the Features predicted by the static RBFilter is the initial location of these Features
// When these predictions are used for the visual tracking the searching area for the Features is static until these Features can
// be assigned to an existing RB (or a group of them create a new one)
// The consequence is that the Features could go out of the searching area before they are detected as moving
// There are two solutions:
//      1- Use the static_motion_threshold in the estimation of the searching area. The searching area should cover motions of static_motion_threshold
//         at the depth of the Feature (COMPLEX)
//      2- Only for the static RBFilter the predicted locations retrieved here are the latest locations of the Features. This is ugly (treats the static
//         RBFilter as an special one) but easier
void MultiRBTracker::predictMeasurement()
{
    std::vector<RBFilter::Ptr>::iterator filter_it = this->_kalman_filters.begin();
    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        (*filter_it)->predictMeasurement();
    }

    this->_predicted_measurement->points.clear();

    std::vector<Feature::Id> feature_ids;
    FeatureCloudPCLwc one_filter_predicted_measurement;
    filter_it = this->_kalman_filters.begin();
    filter_it++;
    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        one_filter_predicted_measurement = (*filter_it)->getPredictedMeasurement();
        for (size_t feat_idx = 0; feat_idx < one_filter_predicted_measurement.points.size(); feat_idx++)
        {
            this->_predicted_measurement->points.push_back(one_filter_predicted_measurement.points.at(feat_idx));
            feature_ids.push_back(one_filter_predicted_measurement.points.at(feat_idx).label);
        }
    }

    // The free Features are not in any RBFilter. We collect their latest locations
    std::vector<Feature::Id> free_features_ids = this->EstimateFreeFeatures(feature_ids);
    FeaturePCLwc free_feat;
    for (size_t feat_idx = 0; feat_idx < free_features_ids.size(); feat_idx++)
    {
        free_feat.x = this->_features_db->getFeatureLastX(free_features_ids.at(feat_idx));
        free_feat.y = this->_features_db->getFeatureLastY(free_features_ids.at(feat_idx));
        free_feat.z = this->_features_db->getFeatureLastZ(free_features_ids.at(feat_idx));
        free_feat.label = free_features_ids.at(feat_idx);
        free_feat.covariance[0] = 20;
        this->_predicted_measurement->points.push_back(free_feat);
    }
}

rbt_measurement_t MultiRBTracker::getPredictedMeasurement() const
{
    this->_predicted_measurement->header.stamp = pcl_conversions::toPCL(ros::Time(this->_measurement_timestamp_ns/1e9+this->_loop_period_ns/1e9));
    return this->_predicted_measurement;
}

std::vector<Feature::Id> MultiRBTracker::estimateBestPredictionsAndSupportingFeatures()
{
    std::vector<Feature::Id> all_filters_supporting_features;
    for (std::vector<RBFilter::Ptr>::iterator filter_it = this->_kalman_filters.begin(); filter_it != this->_kalman_filters.end();)
    {
        (*filter_it)->estimateBestPredictionAndSupportingFeatures();
        std::vector<Feature::Id> this_filter_supporting_features = (*filter_it)->getSupportingFeatures();

        // If there are no features supporting to the EKF we delete it (no re-finding of RBs/EKFs possible)
        // Don't delete the filter of the environment!!! (filter_it == this->_kalman_filters.begin())
        if (this_filter_supporting_features.size()  < (size_t)this->_supporting_features_threshold
                && *filter_it != this->_static_environment_filter )
        {
            Eigen::Twistd last_pose_ec;
            TransformMatrix2Twist((*filter_it)->getPose(), last_pose_ec);
            ROS_ERROR_NAMED( "MultiRBTracker.estimateBestPredictionsAndSupportingFeatures",
                             "RBFilter %2d: Supporting feats:%3d (<%3d), Pose (vx,vy,vz,rx,ry,rz): % 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f,% 2.2f. REMOVED",
                             (int)(*filter_it)->getId(),
                             (int)this_filter_supporting_features.size() ,
                             (int)(this->_supporting_features_threshold),
                             last_pose_ec.vx(),
                             last_pose_ec.vy(),
                             last_pose_ec.vz(),
                             last_pose_ec.rx(),
                             last_pose_ec.ry(),
                             last_pose_ec.rz());
            filter_it = this->_kalman_filters.erase(filter_it);
        }
        else
        {
            all_filters_supporting_features.insert(all_filters_supporting_features.end(),
                                                   this_filter_supporting_features.begin(),
                                                   this_filter_supporting_features.end());
            filter_it++;
        }
    }
    return all_filters_supporting_features;
}

void MultiRBTracker::correctState()
{
    // 1: Estimate the supporting features of each filter
    std::vector<Feature::Id> all_filters_supporting_features = this->estimateBestPredictionsAndSupportingFeatures();

    // 2: Estimate the free features
    std::vector<Feature::Id> free_feat_ids = this->EstimateFreeFeatures(all_filters_supporting_features);

    // 3: Try to assign free features to existing RBs
    std::vector<Feature::Id> really_free_feat_ids = this->ReassignFreeFeatures(free_feat_ids);

    ROS_INFO_NAMED("MultiRBTracker::correctState","Moving bodies: %3d, free features: %3d", (int)this->_kalman_filters.size(), (int)really_free_feat_ids.size());

    // 4: Correct the predicted state of each RBFilter using the last acquired Measurement
    for (std::vector<RBFilter::Ptr>::iterator filter_it = this->_kalman_filters.begin(); filter_it != this->_kalman_filters.end(); filter_it++)
    {
        (*filter_it)->correctState();
    }

    // 5: Try to create new filters for the "really free" features
    // NOTE: Keep this step after the correctState of the RBFilters, otherwise you will create a filter and correct it in the first iteration!
    if(this->_kalman_filters.size() < this->_max_num_rb)
    {
        this->TryToCreateNewFilter(really_free_feat_ids);
    }
}

void MultiRBTracker::ReflectState()
{
    this->_state.rb_poses_and_vels.clear();

    std::vector<geometry_msgs::TwistWithCovariancePtr> tracked_poses_twist_with_cov = this->getPosesECWithCovariance();
    std::vector<geometry_msgs::TwistWithCovariancePtr> tracked_vels_twist_with_cov = this->getVelocitiesWithCovariance();
    std::vector<Eigen::Vector3d> centroids = this->getCentroids();

    RB_id_t RB_id;

    for (size_t poses_idx = 0; poses_idx < tracked_poses_twist_with_cov.size(); poses_idx++)
    {
        RB_id = this->getRBId(poses_idx);

        geometry_msgs::Point centroid;
        centroid.x = centroids.at(poses_idx).x();
        centroid.y = centroids.at(poses_idx).y();
        centroid.z = centroids.at(poses_idx).z();

        omip_msgs::RigidBodyPoseAndVelMsg rbpose_and_vel;
        rbpose_and_vel.rb_id = RB_id;
        rbpose_and_vel.pose_wc = *(tracked_poses_twist_with_cov.at(poses_idx));
        rbpose_and_vel.velocity_wc = *(tracked_vels_twist_with_cov.at(poses_idx));
        rbpose_and_vel.centroid = centroid;
        this->_state.rb_poses_and_vels.push_back(rbpose_and_vel);
    }
}

rbt_state_t MultiRBTracker::getState() const
{
    return this->_state;
}

void MultiRBTracker::setDynamicReconfigureValues(rb_tracker::RBTrackerDynReconfConfig &config)
{
    _static_environment_filter->setMotionConstraint(config.cam_motion_constraint);
    this->_min_num_feats_for_new_rb = config.min_feats_new_rb;
    this->_supporting_features_threshold = config.min_feats_survive_rb;
}

void MultiRBTracker::processMeasurementFromShapeTracker(const omip_msgs::ShapeTrackerStates &meas_from_st)
{
    for(int idx = 0; idx < meas_from_st.shape_tracker_states.size(); idx++)
    {
        BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
        {
            if (filter->getId() == meas_from_st.shape_tracker_states.at(idx).rb_id )
            {
                //Check if the flag indicates that this refinement is reliable
                if((meas_from_st.shape_tracker_states.at(idx).number_of_points_of_model > _min_num_points_in_segment) &&
                        (meas_from_st.shape_tracker_states.at(idx).number_of_points_of_current_pc > _min_num_points_in_segment) &&
                        (meas_from_st.shape_tracker_states.at(idx).probabilistic_value > _min_probabilistic_value) &&
                        (meas_from_st.shape_tracker_states.at(idx).fitness_score < _max_fitness_score))
                {
                    ROS_ERROR_STREAM("Integrating shape-based tracked pose RB " << meas_from_st.shape_tracker_states.at(idx).rb_id);
                    filter->integrateShapeBasedPose(meas_from_st.shape_tracker_states.at(idx).pose_wc, this->_measurement_timestamp_ns - meas_from_st.header.stamp.toNSec() );
                }else{
                    ROS_ERROR_STREAM("Ignoring shape-based tracked pose RB " << meas_from_st.shape_tracker_states.at(idx).rb_id);
                }
            }
        }
    }
}

void MultiRBTracker::CreateNewFilter(const std::vector<Eigen::Matrix4d>& initial_trajectory,
                                     const Eigen::Twistd& initial_velocity,
                                     const std::vector<Feature::Id>& initial_supporting_feats)
{
    RBFilter::Ptr new_kf = RBFilter::Ptr(new RBFilter(this->_loop_period_ns,
                                                      initial_trajectory,
                                                      initial_velocity,
                                                      this->_features_db,
                                                      this->_estimation_error_threshold));

    new_kf->setMeasurementDepthFactor(this->_meas_depth_factor);
    new_kf->setMinCovarianceMeasurementX(this->_min_cov_meas_x);
    new_kf->setMinCovarianceMeasurementY(this->_min_cov_meas_y);
    new_kf->setMinCovarianceMeasurementZ(this->_min_cov_meas_z);
    new_kf->setPriorCovariancePose(this->_prior_cov_pose);
    new_kf->setPriorCovarianceVelocity(this->_prior_cov_vel);
    new_kf->setCovarianceSystemAccelerationTx(this->_cov_sys_acc_tx);
    new_kf->setCovarianceSystemAccelerationTy(this->_cov_sys_acc_ty);
    new_kf->setCovarianceSystemAccelerationTz(this->_cov_sys_acc_tz);
    new_kf->setCovarianceSystemAccelerationRx(this->_cov_sys_acc_rx);
    new_kf->setCovarianceSystemAccelerationRy(this->_cov_sys_acc_ry);
    new_kf->setCovarianceSystemAccelerationRz(this->_cov_sys_acc_rz);
    new_kf->setNumberOfTrackedFeatures(this->_num_tracked_feats);
    new_kf->setMinNumberOfSupportingFeaturesToCorrectPredictedState(this->_min_num_supporting_feats_to_correct);

    new_kf->Init();

    this->_kalman_filters.push_back(new_kf);
    BOOST_FOREACH(Feature::Id supporting_feat_id, initial_supporting_feats)
    {
        new_kf->addSupportingFeature(supporting_feat_id);
    }
}

std::vector<Feature::Id> MultiRBTracker::EstimateFreeFeatures(const std::vector<Feature::Id>& supporting_features)
{
    // We check the set of free features = alive features - supporting features
    std::vector<Feature::Id> alive_feat_ids = this->_features_db->getListOfAliveFeatureIds();
    std::vector<Feature::Id> free_feat_ids;
    BOOST_FOREACH(Feature::Id alive_feat_id, alive_feat_ids)
    {
        // If the feature ID is not in the list of supporting features -> Free feature
        if (std::find(supporting_features.begin(), supporting_features.end(),alive_feat_id) == supporting_features.end())
        {
            free_feat_ids.push_back(alive_feat_id);
        }
    }
    _free_feat_ids = free_feat_ids;
    return free_feat_ids;
}

// NOTE: Here the features have an "extra" location compared to the moment when we predict the measurement for each filter (at the end of the iteration) (predictMeasurement)
// Therefore, the "age of the feature" is +1 the age it would have when predicting the measurement
// EXCEPT if the measurement prediction has to be called again in the loop!!!!!
std::vector<Feature::Id> MultiRBTracker::ReassignFreeFeatures(const std::vector<Feature::Id>& free_feat_ids)
{
    std::vector<Feature::Id> really_free_feat_ids;

    Feature::Ptr free_feat;
    Feature::Location free_feature_last_location;
    Feature::Location free_feature_pre_last_location;
    Feature::Location free_feature_predicted_location;
    double feat_error = 0.0;
    double prediction_error = 0.0;
    int best_ekf_idx = -1;
    Eigen::Matrix4d predicted_pose;

    // Check for each free feature if we can add it to one of the existing vision-based rigid bodies
    BOOST_FOREACH(Feature::Id free_feat_id, free_feat_ids)
    {
        free_feature_last_location = this->_features_db->getFeatureLastLocation(free_feat_id);
        free_feature_pre_last_location = this->_features_db->getFeatureNextToLastLocation(free_feat_id);
        free_feat = this->_features_db->getFeature(free_feat_id);
        feat_error = (double)this->_max_error_to_reassign_feats;
        best_ekf_idx = -1;
        Feature::Location best_predicted_location;

        // Get the prediction from each of the existing vision-based rigid bodies
        // TODO: Use the Mahalanobis distance between the uncertain feature location and its predicted location
        for (size_t filter_idx = 0; filter_idx < this->_kalman_filters.size();filter_idx++)
        {
            // Get the most likely prediction about the next feature state (the likelihood is estimated in estimateSupportingFeatures and copied to the others
            // so it is equivalent to get the belief pose from any hypothesis)
            //prediction_error = this->_kalman_filters.at(filter_idx)->PredictFeatureLocationNew(free_feat, false, true);

            predicted_pose = this->_kalman_filters.at(filter_idx)->getPose();
            this->_kalman_filters.at(filter_idx)->PredictFeatureLocation(free_feat, predicted_pose, false, free_feature_predicted_location,true);
            prediction_error = L2Distance(free_feature_last_location, free_feature_predicted_location);

            if (prediction_error < feat_error)
            {
                feat_error = prediction_error;
                best_ekf_idx = filter_idx;
                best_predicted_location = free_feature_predicted_location;
            }
        }

        if (best_ekf_idx != -1)
        {
            this->_kalman_filters.at(best_ekf_idx)->addSupportingFeature(free_feat_id);
            this->_kalman_filters.at(best_ekf_idx)->addPredictedFeatureLocation(best_predicted_location,free_feature_pre_last_location, free_feat_id);
        }
        else
        {
            really_free_feat_ids.push_back(free_feat_id);
        }
    }
    _really_free_feat_ids = really_free_feat_ids;
    return really_free_feat_ids;
}

void MultiRBTracker::TryToCreateNewFilter(const std::vector<Feature::Id>& really_free_feat_ids)
{
    // First check that the number of free features is over the minimum number to estimate motion
    if(really_free_feat_ids.size() > this->_min_num_feats_for_new_rb)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::Correspondences initial_correspondeces, filtered_correspondences;
        int index = 0;
        std::map<int, int> index_to_feat_id;
        PointPCL point_pcl;

        //Iterate over the free features
        BOOST_FOREACH(Feature::Id free_feat_id, really_free_feat_ids)
        {
            // If the free feature is old enough, we add it to the source-target point clouds and add a correspondence
            if ((int)this->_features_db->getFeatureAge(free_feat_id) > this->_min_num_frames_for_new_rb)
            {
                Feature::Location first = this->_features_db->getFeatureNToLastLocation(free_feat_id, this->_min_num_frames_for_new_rb);
                Feature::Location second = this->_features_db->getFeatureLastLocation(free_feat_id);
                Location2PointPCL(first, point_pcl);
                source->push_back(point_pcl);
                Location2PointPCL(second, point_pcl);
                target->push_back(point_pcl);
                initial_correspondeces.push_back(pcl::Correspondence(index, index, 0.0));
                index_to_feat_id[index] = free_feat_id;
                index++;
            }
        }

        // If enough of the free features are old enough, we try to estimate the motion
        if(index > this->_min_num_feats_for_new_rb)
        {
            //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
            // PCL RANSAC
            pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
            rejector.setInputSource(source);
            rejector.setInputTarget(target);
            rejector.setInlierThreshold(this->_new_rbm_error_threshold);
            rejector.setMaximumIterations(this->_ransac_iterations);
            rejector.getRemainingCorrespondences(initial_correspondeces, filtered_correspondences);

            Eigen::Matrix4f best_transformation = rejector.getBestTransformation();

            // If the number of free features that are inliers of the best transformation is over the minimum number to estimate motion
            // AND if the best transformation is not just noise:
            //      - Translation is larger than min_amount_translation meters
            //      - Rotation around the axis is larger than min_amount_rotation degrees
            Eigen::Transform<float, 3, Eigen::Affine> best_transformation2(best_transformation);
            double amount_translation = best_transformation2.translation().norm();
            Eigen::AngleAxisf best_axis_angle;
            double amount_rotation = best_axis_angle.fromRotationMatrix(best_transformation2.rotation()).angle();

            if ((int)filtered_correspondences.size() > this->_min_num_feats_for_new_rb && (
                        amount_translation > _min_amount_translation_for_new_rb ||
                        amount_rotation > _min_amount_rotation_for_new_rb*(M_PI/180.0) ) )
            {
                std::vector<Eigen::Matrix4d> trajectory;
                Feature::Location centroid_in_cf;

                //Collect the ids of the matched features
                //and compute the centroid of the supporting features N frames ago
                //The centroid will be used to "place" the frame of the rigid body -> Initial transformation of the rigid body
                //is a pure translation to the location of the centroid
                std::vector<Feature::Id> ransac_best_trial_inliers;
                BOOST_FOREACH(pcl::Correspondence filter_correspondence, filtered_correspondences)
                {
                    int feature_id = index_to_feat_id.at(filter_correspondence.index_match);
                    ransac_best_trial_inliers.push_back(feature_id);
                    Feature::Location feat_loc = this->_features_db->getFeatureNToLastLocation(feature_id, this->_min_num_frames_for_new_rb-1);
                    boost::tuples::get<0>(centroid_in_cf) = boost::tuples::get<0>(centroid_in_cf) + boost::tuples::get<0>(feat_loc);
                    boost::tuples::get<1>(centroid_in_cf) = boost::tuples::get<1>(centroid_in_cf) + boost::tuples::get<1>(feat_loc);
                    boost::tuples::get<2>(centroid_in_cf) = boost::tuples::get<2>(centroid_in_cf) + boost::tuples::get<2>(feat_loc);
                }

                boost::tuples::get<0>(centroid_in_cf) = boost::tuples::get<0>(centroid_in_cf)/(double)filtered_correspondences.size();
                boost::tuples::get<1>(centroid_in_cf) = boost::tuples::get<1>(centroid_in_cf)/(double)filtered_correspondences.size();
                boost::tuples::get<2>(centroid_in_cf) = boost::tuples::get<2>(centroid_in_cf)/(double)filtered_correspondences.size();

                Eigen::Matrix4d pre_translation = Eigen::Matrix4d::Identity();
                pre_translation(0,3) = boost::tuples::get<0>(centroid_in_cf);
                pre_translation(1,3) = boost::tuples::get<1>(centroid_in_cf);
                pre_translation(2,3) = boost::tuples::get<2>(centroid_in_cf);

                trajectory.push_back(pre_translation);

                // Estimate the trajectory in the last _min_num_frames_for_new_rb frames
                for(int frame = this->_min_num_frames_for_new_rb-2; frame >= 0; frame--)
                {
                    FeaturesDataBase::MapOfFeatureLocationPairs one_frame_best_subset;
                    BOOST_FOREACH(Feature::Id matched_feat_id, ransac_best_trial_inliers)
                    {
                        one_frame_best_subset[matched_feat_id] = Feature::LocationPair(this->_features_db->getFeatureNToLastLocation(matched_feat_id, frame),
                                                                                       this->_features_db->getFeatureNToLastLocation(matched_feat_id, this->_min_num_frames_for_new_rb-1)
                                                                                       - centroid_in_cf);
                    }
                    Eigen::Matrix4d one_frame_best_transformation_matrix;
                    estimateRigidTransformation(one_frame_best_subset,one_frame_best_transformation_matrix);
                    trajectory.push_back(one_frame_best_transformation_matrix);
                }

                Eigen::Matrix4d last_delta = trajectory.at(trajectory.size()-1)*(trajectory.at(trajectory.size()-2).inverse());
                Eigen::Twistd initial_velocity;
                TransformMatrix2Twist(last_delta, initial_velocity);

                double measured_loop_period_ns= this->_measurement_timestamp_ns - this->_previous_measurement_timestamp_ns;
                initial_velocity /=(measured_loop_period_ns/1e9);

                this->CreateNewFilter(trajectory, initial_velocity, ransac_best_trial_inliers);
            }else
            {
                ROS_INFO_STREAM_NAMED("RBMTracker._createNewFilters", "Not enough Consensus to create a new Filter with the free Features ("<<
                                      filtered_correspondences.size() <<" inliers are too few, or the the amount of translation " <<
                                      amount_translation << " < " << _min_amount_translation_for_new_rb << " and the amount of rotation " <<
                                      amount_rotation << " < " << _min_amount_rotation_for_new_rb*(M_PI/180.0));
                return;
            }
        }
    }
}

std::vector<Eigen::Matrix4d> MultiRBTracker::getPoses() const
{
    std::vector<Eigen::Matrix4d> return_filter_results;

    BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
    {
        return_filter_results.push_back(filter->getPose());
    }
    return return_filter_results;
}

std::vector<geometry_msgs::PoseWithCovariancePtr> MultiRBTracker::getPosesWithCovariance() const
{
    std::vector<geometry_msgs::PoseWithCovariancePtr> return_filter_results;
    BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
    {
        return_filter_results.push_back(filter->getPoseWithCovariance());
    }
    return return_filter_results;
}

std::vector<geometry_msgs::TwistWithCovariancePtr> MultiRBTracker::getPosesECWithCovariance() const
{
    std::vector<geometry_msgs::TwistWithCovariancePtr> return_filter_results;
    BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
    {
        return_filter_results.push_back(filter->getPoseECWithCovariance());
    }
    return return_filter_results;
}

std::vector<geometry_msgs::TwistWithCovariancePtr> MultiRBTracker::getVelocitiesWithCovariance() const
{
    std::vector<geometry_msgs::TwistWithCovariancePtr> return_filter_results;
    BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
    {
        return_filter_results.push_back(filter->getVelocityWithCovariance());
    }
    return return_filter_results;
}

void MultiRBTracker::addFeatureLocation(Feature::Id f_id, Feature::Location f_loc)
{
    if (this->_features_db->addFeatureLocation(f_id, f_loc))
    {
        // addFeature::LocationOfFeature returns true if the feature is new
        _static_environment_filter->addSupportingFeature(f_id);
    }
}

RB_id_t MultiRBTracker::getRBId(int n) const
{
    return this->_kalman_filters.at(n)->getId();
}

int MultiRBTracker::getNumberSupportingFeatures(int n) const
{
    return this->_kalman_filters.at(n)->getNumberSupportingFeatures();
}

FeatureCloudPCL MultiRBTracker::getLabelledSupportingFeatures()
{
    FeatureCloudPCL colored_pc;
    std::vector<RBFilter::Ptr>::iterator filter_it =this->_kalman_filters.begin();
    // TODO: The next line is to not show the supporting features of the static rigid body.
    // I commented it out for the ICRA paper: we want to plot all points

    filter_it++;
    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        std::vector<Feature::Id> this_filter_supporting_features = (*filter_it)->getSupportingFeatures();
        for (size_t feat_idx = 0; feat_idx < this_filter_supporting_features.size();feat_idx++)
        {
            if (this->_features_db->isFeatureStored(this_filter_supporting_features.at(feat_idx)))
            {
                FeaturePCL temp_point;
                RB_id_t label = (*filter_it)->getId();
                temp_point.x = this->_features_db->getFeatureLastX(this_filter_supporting_features.at(feat_idx));
                temp_point.y = this->_features_db->getFeatureLastY(this_filter_supporting_features.at(feat_idx));
                temp_point.z = this->_features_db->getFeatureLastZ(this_filter_supporting_features.at(feat_idx));

                temp_point.label = label;
                colored_pc.points.push_back(temp_point);
            }
        }
    }
    return colored_pc;
}

FeatureCloudPCL MultiRBTracker::getFreeFeatures()
{
    FeatureCloudPCL colored_pc;
    for (size_t feat_idx = 0; feat_idx < _really_free_feat_ids.size();feat_idx++)
    {
        if (this->_features_db->isFeatureStored(_really_free_feat_ids.at(feat_idx)))
        {
            FeaturePCL temp_point;
            temp_point.x = this->_features_db->getFeatureLastX(_really_free_feat_ids.at(feat_idx));
            temp_point.y = this->_features_db->getFeatureLastY(_really_free_feat_ids.at(feat_idx));
            temp_point.z = this->_features_db->getFeatureLastZ(_really_free_feat_ids.at(feat_idx));

            temp_point.label = _really_free_feat_ids.at(feat_idx);
            colored_pc.points.push_back(temp_point);
        }
    }
    return colored_pc;
}

FeatureCloudPCL MultiRBTracker::getPredictedFeatures()
{
    FeatureCloudPCL colored_pc;
    std::vector<RBFilter::Ptr>::iterator filter_it =this->_kalman_filters.begin();
    filter_it++;
    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        FeatureCloudPCLwc::Ptr predicted_locations_pc = (*filter_it)->getFeaturesPredicted();
        for (size_t feat_idx = 0; feat_idx < predicted_locations_pc->points.size();feat_idx++)
        {
            FeaturePCL temp_point;
            temp_point.x = predicted_locations_pc->points[feat_idx].x;
            temp_point.y = predicted_locations_pc->points[feat_idx].y;
            temp_point.z = predicted_locations_pc->points[feat_idx].z;
            temp_point.label = predicted_locations_pc->points[feat_idx].label;
            colored_pc.points.push_back(temp_point);
        }
        (*filter_it)->resetFeaturesPredicted();
    }
    return colored_pc;
}

FeatureCloudPCL MultiRBTracker::getAtBirthFeatures()
{
    FeatureCloudPCL colored_pc;
    std::vector<RBFilter::Ptr>::iterator filter_it =this->_kalman_filters.begin();
    filter_it++;
    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        FeatureCloudPCLwc::Ptr at_birth_locations_pc = (*filter_it)->getFeaturesAtBirth();
        for (size_t feat_idx = 0; feat_idx < at_birth_locations_pc->points.size();feat_idx++)
        {
            FeaturePCL temp_point;
            temp_point.x = at_birth_locations_pc->points[feat_idx].x;
            temp_point.y = at_birth_locations_pc->points[feat_idx].y;
            temp_point.z = at_birth_locations_pc->points[feat_idx].z;
            temp_point.label = at_birth_locations_pc->points[feat_idx].label;
            colored_pc.points.push_back(temp_point);
        }
        (*filter_it)->resetFeaturesAtBirth();
    }
    return colored_pc;
}

std::vector<Eigen::Vector3d> MultiRBTracker::getCentroids() const
{
    std::vector<Eigen::Vector3d> centroids;
    centroids.push_back(Eigen::Vector3d(0., 0., 0.));
    std::vector<RBFilter::Ptr>::const_iterator filter_it = this->_kalman_filters.begin();
    filter_it++;    //Ignore the static environment

    for (; filter_it != this->_kalman_filters.end(); filter_it++)
    {
        //NEW: the first time we send as centroid the centroid we used for the trajectory estimation
        //In this way we can inform the next level of the initial pose of the rigid body
        if((*filter_it)->getTrajectory().size() < this->_min_num_frames_for_new_rb)
        {
            Feature::Location initial_location = (*filter_it)->getIntialLocationOfCentroid();
            centroids.push_back(Eigen::Vector3d(boost::tuples::get<0>(initial_location),
                                                boost::tuples::get<1>(initial_location),
                                                boost::tuples::get<2>(initial_location)));
        }else{
            Eigen::Vector3d temp_centroid = Eigen::Vector3d(0., 0., 0.);
            int supporting_feats_ctr = 0;
            std::vector<Feature::Id> this_filter_supporting_features = (*filter_it)->getSupportingFeatures();
            for (size_t feat_idx = 0; feat_idx < this_filter_supporting_features.size();feat_idx++)
            {
                if (this->_features_db->isFeatureStored(this_filter_supporting_features.at(feat_idx)))
                {
                    temp_centroid.x() += this->_features_db->getFeatureLastX(this_filter_supporting_features.at(feat_idx));
                    temp_centroid.y() += this->_features_db->getFeatureLastY(this_filter_supporting_features.at(feat_idx));
                    temp_centroid.z() += this->_features_db->getFeatureLastZ(this_filter_supporting_features.at(feat_idx));
                    supporting_feats_ctr++;
                }
            }
            centroids.push_back(temp_centroid / (double)supporting_feats_ctr);
        }
    }
    return centroids;
}

void MultiRBTracker::addPredictedState(const rbt_state_t& predicted_state, const double &predicted_state_timestamp_ns)
{    
    BOOST_FOREACH(omip_msgs::RigidBodyPoseAndVelMsg predicted_pose_and_vel, predicted_state.rb_poses_and_vels)
    {
        BOOST_FOREACH(RBFilter::Ptr filter, this->_kalman_filters)
        {
            if (filter->getId() == predicted_pose_and_vel.rb_id)
            {
                //ROS_WARN_STREAM_NAMED("MultiRBTracker.addPredictedState", "Prediction for body " << predicted_pose_and_vel.rb_id);
                filter->setPredictedState(predicted_pose_and_vel);
            }
        }
    }
}

FeaturesDataBase::Ptr MultiRBTracker::getFeaturesDatabase()
{
    return this->_features_db;
}

