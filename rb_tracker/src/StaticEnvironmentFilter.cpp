#include "rb_tracker/StaticEnvironmentFilter.h"
#include "omip_common/OMIPUtils.h"
#include <boost/foreach.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace omip;

using namespace MatrixWrapper;
using namespace BFL;

tf::Transform Eigen2Tf(Eigen::Matrix4f trans)
{
    tf::Matrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2));
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

tf::Transform Eigen2Tf(Eigen::Matrix4d trans)
{
    tf::Matrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2));
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

StaticEnvironmentFilter::StaticEnvironmentFilter(double loop_period_ns, FeaturesDataBase::Ptr feats_database,
                                                 double environment_motion_threshold) :
    RBFilter()
{
    this->_loop_period_ns = loop_period_ns;
    this->_id = 0;
    this->_features_database = feats_database;
    this->_estimation_error_threshold = environment_motion_threshold;

    this->_tf_epsilon_linear = 1e-8;
    this->_tf_epsilon_angular = 1.7e-9;
    this->_max_iterations = 100;

    this->_pose = Eigen::Matrix4d::Identity();
    this->_velocity = Eigen::Twistd(0.,0.,0.,0.,0.,0);

    this->_motion_constraint = NO_MOTION;

    this->_computation_type = STATIC_ENVIRONMENT_EKF_TRACKER;

    // To avoid segmentation fault because the features are as old as the rigid body
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
    this->_trajectory.push_back(this->_pose);
}

void StaticEnvironmentFilter::Init()
{
    RBFilter::Init();
    _pose_covariance.setZero();
    _velocity_covariance.setZero();
}

void StaticEnvironmentFilter::predictState(double time_interval_ns)
{   
    if(this->_motion_constraint == NO_MOTION)
    {
        this->_predicted_pose_vh = _pose;
        this->_predicted_pose_bh = _pose;
        return;
    }

    switch(this->_computation_type)
    {

    case STATIC_ENVIRONMENT_ICP_TRACKER:
    {
        Eigen::Twistd predicted_delta_pose_vh = (this->_velocity*time_interval_ns/1e9);
        this->_predicted_pose_vh = predicted_delta_pose_vh.exp(1e-12).toHomogeneousMatrix()*_pose;
        this->_predicted_pose_bh = _pose;
    }
        break;
    case STATIC_ENVIRONMENT_EKF_TRACKER:
    {
        RBFilter::predictState(time_interval_ns);
    }
        break;

    default:
        ROS_ERROR_STREAM_NAMED("StaticEnvironmentFilter.predictState","Wrong StaticEnvironment computation_type!");
        break;
    }
}

void StaticEnvironmentFilter::correctState()
{
    // The problem is that the StaticEnvironmentFilter is created at the beggining when only one location per feature is available
    // There is no way to correct the state with only one location per feature
    static bool first_time = true;
    if(first_time)
    {
        this->_trajectory.push_back(this->_pose);
        first_time = false;
        return;
    }

    if(this->_motion_constraint == NO_MOTION)
    {
        this->_velocity = Eigen::Twistd(0,0,0,0,0,0);
        this->_trajectory.push_back(_pose);
        return;
    }

    switch(this->_computation_type)
    {

    case STATIC_ENVIRONMENT_ICP_TRACKER:
    {
        // I follow the convention of "Introduction to robotics" by Craig
        // The first fram is the reference (left upper index) and the second frame is the referred (left lower index)
        // se -> static environment
        // set -> static environment at time t
        // Example: setnext_set_T is describes the frame "static environment at time t" wrt the frame "static environment at time tnext"
        tf::Transform set_setnext_Tf;
        this->estimateDeltaMotion(this->_supporting_features_ids, set_setnext_Tf);

        Eigen::Matrix4d set_setnext_T;
        set_setnext_Tf.getOpenGLMatrix(set_setnext_T.data());

        Eigen::Twistd set_setnext_Twist;
        TransformMatrix2Twist(set_setnext_T, set_setnext_Twist);

        this->_velocity = set_setnext_Twist/(this->_loop_period_ns/1e9);

        // Accumulate the new delta into the absolute pose of the static environment wrt the camera
        this->_pose = this->_pose*set_setnext_T;

        this->_trajectory.push_back(this->_pose);

        constrainMotion();
    }
        break;
    case STATIC_ENVIRONMENT_EKF_TRACKER:
    {
        RBFilter::correctState();

        constrainMotion();
    }
        break;

    default:
        ROS_ERROR_STREAM_NAMED("StaticEnvironmentFilter.correctState","Wrong StaticEnvironment computation_type!");
        break;
    }
}

void StaticEnvironmentFilter::setMotionConstraint(int motion_constraint)
{
    this->_motion_constraint = (MotionConstraint)motion_constraint;
}

void StaticEnvironmentFilter::setComputationType(static_environment_tracker_t computation_type)
{
    this->_computation_type = computation_type;
}

void StaticEnvironmentFilter::addSupportingFeature(Feature::Id supporting_feat_id)
{
    this->_supporting_features_ids.push_back(supporting_feat_id);

    Feature::Location predicted_location_velocity;
    Feature::Location predicted_location_brake;
    FeaturePCLwc predicted_feature_pcl;
    // Predict feature location based on velocity hypothesis

    predicted_location_velocity = this->_features_database->getFeatureLastLocation(supporting_feat_id);
    LocationAndId2FeaturePCLwc(predicted_location_velocity, supporting_feat_id, predicted_feature_pcl);
    this->_predicted_measurement_pc_vh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_vh[supporting_feat_id] = predicted_location_velocity;

    // Predict feature location based on brake hypothesis (same as last)
    predicted_location_brake = this->_features_database->getFeatureLastLocation(supporting_feat_id);
    LocationAndId2FeaturePCLwc(predicted_location_brake, supporting_feat_id, predicted_feature_pcl);
    this->_predicted_measurement_pc_bh->points.push_back(predicted_feature_pcl);
    this->_predicted_measurement_map_bh[supporting_feat_id] = predicted_location_brake;
}

void StaticEnvironmentFilter::estimateDeltaMotion(std::vector<Feature::Id>& supporting_features_ids, tf::Transform& previous_current_Tf)
{
    // Prepare the 2 point clouds with the locations of the features in current and previous frames
    pcl::PointCloud<pcl::PointXYZ> previous_locations, current_locations;
    BOOST_FOREACH(Feature::Id supporting_feat_id, supporting_features_ids)
    {
        if (this->_features_database->isFeatureStored(supporting_feat_id) && this->_features_database->getFeatureAge(supporting_feat_id) > 2)
        {
            Feature::Location previous_location = this->_features_database->getFeatureNextToLastLocation(supporting_feat_id);
            Feature::Location current_location = this->_features_database->getFeatureLastLocation(supporting_feat_id);
            previous_locations.push_back(pcl::PointXYZ(previous_location.get<0>(), previous_location.get<1>() ,previous_location.get<2>()));
            current_locations.push_back(pcl::PointXYZ(current_location.get<0>(), current_location.get<1>() ,current_location.get<2>()));
        }
    }

    if (previous_locations.size() ==0)
    {
        ROS_ERROR_STREAM_NAMED("StaticEnvironmentFilter.estimateDeltaMotion","There are no features supporting the static environment!");
        previous_current_Tf.setIdentity();
    }else{
        iterativeICP(previous_locations, current_locations, previous_current_Tf);
    }
}

void StaticEnvironmentFilter::iterativeICP( pcl::PointCloud<pcl::PointXYZ>& previous_locations,
                                            pcl::PointCloud<pcl::PointXYZ>& current_locations, tf::Transform& previous_current_Tf)
{
    // initialize the result transform
    Eigen::Matrix4f previous_current_T;
    previous_current_T.setIdentity();

    // create indices
    std::vector<int> previous_indices, current_indices;
    for(int i=0; i<previous_locations.size();i++)
    {
        previous_indices.push_back(i);
        current_indices.push_back(i);
    }

    for (int iteration = 0; iteration < this->_max_iterations; ++iteration)
    {
        // estimate transformation
        Eigen::Matrix4f previous_current_T_new;
        _svdecomposer.estimateRigidTransformation (previous_locations, previous_indices,
                                                   current_locations, current_indices,
                                                   previous_current_T_new);

        // transform
        pcl::transformPointCloud(previous_locations, previous_locations, previous_current_T_new);

        // accumulate incremental tf
        previous_current_T = previous_current_T_new * previous_current_T;

        // check for convergence
        double linear, angular;

        previous_current_Tf = Eigen2Tf(previous_current_T);
        linear = previous_current_Tf.getOrigin().length();
        double trace = previous_current_Tf.getBasis()[0][0] + previous_current_Tf.getBasis()[1][1] + previous_current_Tf.getBasis()[2][2];
        angular = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
        if (linear  < this->_tf_epsilon_linear && angular < this->_tf_epsilon_angular)
        {
            break;
        }
    }
}

void StaticEnvironmentFilter::constrainMotion()
{
    // The last estimated and unconstrained pose is contained in the variable _pose = cam_setnext_T

//    // The previous pose is contained in the vector _trajectory
//    Eigen::Matrix4d cam_set_T = _trajectory.at(_trajectory.size() - 2);

//    switch(this->_motion_constraint)
//    {
//    case NO_ROLL_PITCH:
//    {
//        this->_pose.rz() = cam_set_Twist.rz();
//        this->_pose.ry() = cam_set_Twist.ry();

//        this->_velocity.rz() = 0;
//        this->_velocity.ry() = 0;
//    }
//        break;
//    case NO_ROLL_PITCH_TZ:
//    {
//        this->_pose.rz() = cam_set_Twist.rz();
//        this->_pose.ry() = cam_set_Twist.ry();
//        this->_pose.vz() = cam_set_Twist.vz();

//        this->_velocity.rz() = 0;
//        this->_velocity.ry() = 0;
//        this->_velocity.vz() = 0;
//    }
//        break;
//    case NO_TRANSLATION:
//    {
//        this->_pose.vx() = cam_set_Twist.vx();
//        this->_pose.vy() = cam_set_Twist.vy();
//        this->_pose.vz() = cam_set_Twist.vz();

//        this->_velocity.vx() = 0;
//        this->_velocity.vy() = 0;
//        this->_velocity.vz() = 0;
//    }
//        break;
//    case NO_TRANSLATION_ROLL_YAW:
//    {
//        this->_pose.vx() = cam_set_Twist.vx();
//        this->_pose.vy() = cam_set_Twist.vy();
//        this->_pose.vz() = cam_set_Twist.vz();
//        this->_pose.rz() = cam_set_Twist.rz();
//        this->_pose.rx() = cam_set_Twist.rx();

//        this->_velocity.vx() = 0;
//        this->_velocity.vy() = 0;
//        this->_velocity.vz() = 0;
//        this->_velocity.rz() = 0;
//        this->_velocity.rx() = 0;
//    }
//        break;
//    case NO_ROTATION:
//    {
//        this->_pose.rx() = cam_set_Twist.rx();
//        this->_pose.ry() = cam_set_Twist.ry();
//        this->_pose.rz() = cam_set_Twist.rz();\
//        this->_velocity.rx() = 0;
//        this->_velocity.ry() = 0;
//        this->_velocity.rz() = 0;
//    }
//        break;
//    case NO_MOTION:
//    {
//        this->_pose.rx() = cam_set_Twist.rx();
//        this->_pose.ry() = cam_set_Twist.ry();
//        this->_pose.rz() = cam_set_Twist.rz();
//        this->_pose.vx() = cam_set_Twist.vx();
//        this->_pose.vy() = cam_set_Twist.vy();
//        this->_pose.vz() = cam_set_Twist.vz();
//        this->_velocity.rx() = 0;
//        this->_velocity.ry() = 0;
//        this->_velocity.rz() = 0;
//        this->_velocity.vx() = 0;
//        this->_velocity.vy() = 0;
//        this->_velocity.vz() = 0;
//    }
//        break;
//    case ROBOT_XY_BASELINK_PLANE:
//    {
//        // Query the current pose of the camera wrt the baselink frame
//        tf::StampedTransform bl_cam_TF;
//        Eigen::Matrix4d bl_cam_T;

//        try{

//            this->_tf_listener.lookupTransform("/base_link", "/camera_rgb_optical_frame", ros::Time(0), bl_cam_TF);
//            bl_cam_TF.getOpenGLMatrix(bl_cam_T.data());
//        }
//        catch (tf::TransformException ex)
//        {
//            ROS_ERROR("waitForTransform in StaticEnvironmentFilter failed!");
//            ROS_ERROR("%s",ex.what());
//            bl_cam_T = Eigen::Matrix4d::Identity();
//        }

//        // The matrix of the pose of the static environment to the camera in the next time step
//        Eigen::Matrix4d cam_se_T;
//        Twist2TransformMatrix(_pose, cam_se_T);

//        Eigen::Matrix4d bl_se_T = bl_cam_T * cam_se_T;

//        Eigen::Matrix4d bl_se_T_constrained = bl_se_T;

//        // Eliminate translation along the z axis of the base link frame
//        bl_se_T_constrained(2,3) = bl_cam_T(2,3);

//        // Restrict the rotation to be only around the z axis of the base link frame
//        Eigen::Quaterniond bl_se_R(bl_se_T.block<3,3>(0,0));
//        Eigen::Vector3d bl_se_RPY = bl_se_R.toRotationMatrix().eulerAngles(2, 1, 0);

//        Eigen::Quaterniond bl_cam_R(bl_cam_T.block<3,3>(0,0));
//        Eigen::Vector3d bl_cam_RPY = bl_cam_R.toRotationMatrix().eulerAngles(2, 1, 0);

//        Eigen::Vector3d bl_se_RPY_constrained;

//        bl_se_RPY_constrained[0] = bl_se_RPY[0];    //Rotation around the baselink_z is the estimated one (unconstrained)
//        bl_se_RPY_constrained[1] = bl_cam_RPY[1];   //Rotation around the baselink_y is constrained, must be the same as for the cam
//        bl_se_RPY_constrained[2] = bl_cam_RPY[2];   //Rotation around the baselink_x is constrained, must be the same as for the cam

//        // Build the constrained rotation matrix
//        Eigen::Matrix3f bl_se_R_constrained;
//        bl_se_R_constrained = Eigen::AngleAxisf(bl_se_RPY_constrained[0], Eigen::Vector3f::UnitZ())
//                *Eigen::AngleAxisf(bl_se_RPY_constrained[1], Eigen::Vector3f::UnitY())
//                *Eigen::AngleAxisf(bl_se_RPY_constrained[2], Eigen::Vector3f::UnitX());

//        // Assign the rotation part to the constrained transformation matrix
//        bl_se_T_constrained.block<3,3>(0,0) = bl_se_R_constrained.cast<double>();

//        // Calculate the constrained pose of the static environment to the camera
//        Eigen::Matrix4d cam_se_T_constrained = bl_cam_T.inverse() * bl_se_T_constrained;
//        TransformMatrix2Twist(cam_se_T_constrained, _pose);

//        // Calculate the constrained velocity of the static environment to the camera
//        Eigen::Matrix4d cam_set_T = cam_set_Twist.exp(1e-12).toHomogeneousMatrix();
//        Eigen::Matrix4d set_setnext_T_constrained = cam_set_T.inverse()*cam_se_T_constrained;
//        Eigen::Twistd set_setnext_Twist_constrained;
//        TransformMatrix2Twist(set_setnext_T_constrained, set_setnext_Twist_constrained);

//        _velocity = set_setnext_Twist_constrained / _loop_period_ns/1e9;
//    }
//        break;
//    default:
//        break;

//    }

//    // Reconfigure the variables in the filters
//    ColumnVector constrained_state(12);
//    constrained_state(1) = this->_pose.vx();
//    constrained_state(2) = this->_pose.vy();
//    constrained_state(3) = this->_pose.vz();
//    constrained_state(4) = this->_pose.rx();
//    constrained_state(5) = this->_pose.ry();
//    constrained_state(6) = this->_pose.rz();
//    constrained_state(7) = this->_velocity.vx();
//    constrained_state(8) = this->_velocity.vy();
//    constrained_state(9) = this->_velocity.vz();
//    constrained_state(10) = this->_velocity.rx();
//    constrained_state(11) = this->_velocity.ry();
//    constrained_state(12) = this->_velocity.rz();

//    this->_ekf_velocity->PostGet()->ExpectedValueSet(constrained_state);
//    this->_ekf_brake->PostGet()->ExpectedValueSet(constrained_state);

//    // Remove the last estimated pose (unconstrained) from the trajectory vector and replace it with the constrained one
//    this->_trajectory.pop_back();
//    this->_trajectory.push_back(this->_pose);
}

//void StaticEnvironmentFilter::constrainMotion(tf::Transform& motion)
//{
//    // **** degree-of-freedom constraints

//    switch(this->_motion_constraint)
//    {
//    case NO_ROLL_PITCH:
//    {
//        tf::Quaternion q;
//        q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));

//        motion.setRotation(q);

//    }
//        break;
//    case NO_ROLL_PITCH_TZ:
//    {
//        tf::Quaternion q;
//        q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));

//        tf::Vector3 p(motion.getOrigin().getX(), motion.getOrigin().getY(), 0.0);

//        motion.setOrigin(p);
//        motion.setRotation(q);
//    }
//        break;
//    case NO_TRANSLATION:
//    {
//        tf::Vector3 p(0.0,0.0, 0.0);
//        motion.setOrigin(p);
//    }
//        break;
//    case NO_TRANSLATION_ROLL_YAW:
//    {
//        tf::Quaternion q_set;
//        tf::Quaternion q(motion.getRotation().x(),
//                         motion.getRotation().y(),
//                         motion.getRotation().z(),
//                         motion.getRotation().w());
//        double roll, pitch, yaw;
//        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//        q_set.setRPY(0.0, pitch, 0.0);
//        tf::Vector3 p(0.0,0.0, 0.0);
//        motion.setOrigin(p);
//        motion.setRotation(q_set);
//    }
//        break;
//    case NO_ROTATION:
//    {
//        tf::Quaternion q;
//        q.setRPY(0.0, 0.0 , 0.0);
//        motion.setRotation(q);
//    }
//        break;
//    case NO_MOTION:
//    {
//        tf::Vector3 p(0.,0., 0.0);
//        tf::Quaternion q;
//        q.setRPY(0.0, 0.0 , 0.0);

//        motion.setOrigin(p);
//        motion.setRotation(q);
//    }
//        break;
//    case ROBOT_XY_BASELINK_PLANE:
//    {
//        // Query the current pose of the camera wrt the baselink frame
//        tf::StampedTransform Tf_camt_bl;
//        Eigen::Matrix4d T_camt_bl;
//        try{

//            this->_tf_listener.lookupTransform("/base_link", "/camera_rgb_optical_frame", ros::Time(0), Tf_camt_bl);
//            Tf_camt_bl.getOpenGLMatrix(T_camt_bl.data());
//        }
//        catch (tf::TransformException ex)
//        {
//            ROS_ERROR("waitForTransform in StaticEnvironmentICPFilter failed!");
//            ROS_ERROR("%s",ex.what());
//            T_camt_bl = Eigen::Matrix4d::Identity();
//            getchar();
//        }

//        // Apply the estimated delta motion to the pose
//        // Verbose!

//        Eigen::Matrix4d T_camtnext_camt;
//        motion.inverse().getOpenGLMatrix(T_camtnext_camt.data());

//        Eigen::Matrix4d T_camt_camtnext = T_camtnext_camt.inverse();

//        Eigen::Matrix4d T_camtnext_bl = T_camt_bl*T_camt_camtnext.inverse();

//        Eigen::Matrix4d T_camtnext_bl_constrained = T_camtnext_bl;

//        // Eliminate translation along the z axis
//        T_camtnext_bl_constrained(2,3) = T_camt_bl(2,3);

//        // Restrict the rotation to be only around the z axis
//        Eigen::Quaterniond R_camtnext_bl(T_camtnext_bl.block<3,3>(0,0));
//        Eigen::Vector3d RPY_camtnext_bl = R_camtnext_bl.toRotationMatrix().eulerAngles(2, 1, 0);

//        Eigen::Quaterniond R_camt_bl(T_camt_bl.block<3,3>(0,0));
//        Eigen::Vector3d RPY_camt_bl = R_camt_bl.toRotationMatrix().eulerAngles(2, 1, 0);

//        Eigen::Vector3d RPY_camtnext_bl_constrained;
//        RPY_camtnext_bl_constrained.x() = RPY_camtnext_bl.x();  //Unconstrained rotation around z
//        RPY_camtnext_bl_constrained.y() = RPY_camt_bl.y();  // Constrained rotation around y
//        RPY_camtnext_bl_constrained.z() = RPY_camt_bl.z();  // Constrained rotation around x

//        // Build the constrained pose
//        Eigen::Matrix3f R_camtnext_bl_constrained;
//        R_camtnext_bl_constrained = Eigen::AngleAxisf(RPY_camtnext_bl_constrained[0], Eigen::Vector3f::UnitZ())
//                *Eigen::AngleAxisf(RPY_camtnext_bl_constrained[1], Eigen::Vector3f::UnitY())
//                *Eigen::AngleAxisf(RPY_camtnext_bl_constrained[2], Eigen::Vector3f::UnitX());

//        T_camtnext_bl_constrained.block<3,3>(0,0) = R_camtnext_bl_constrained.cast<double>();

//        Eigen::Matrix4d T_camt_camtnext_constrained = ((T_camt_bl.inverse())*T_camtnext_bl_constrained).inverse();

//        Eigen::Matrix4d T_camtnext_camt_constrained = T_camt_camtnext_constrained.inverse();

//        motion = Eigen2Tf(T_camtnext_camt_constrained);
//    }
//        break;
//    default:
//        break;

//    }
//}
