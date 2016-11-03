#include "shape_tracker/ShapeTracker.h"

#include <pcl_conversions/pcl_conversions.h>

#include <shape_reconstruction/RangeImagePlanar.hpp>

// Need to include the pcl ros utilities
#include "pcl_ros/point_cloud.h"

#include "pcl/filters/approximate_voxel_grid.h"

#include "opencv2/imgproc/imgproc.hpp"

using namespace omip;

#define BB_FILTER_BEFORE_ICP

// Extract the indices that correspond to non-zero pixels of the image
void Image8u2IndicesOfOrganizedPC(const cv::Mat& image_8u, pcl::PointIndices::Ptr indices_ptr)
{
    indices_ptr->indices.clear();
    for(int j=0; j<image_8u.rows; j++)
    {
        for(int i=0; i<image_8u.cols; i++)
        {
            if(image_8u.at<uchar>(j,i))
            {
                indices_ptr->indices.push_back(j*image_8u.cols + i);
            }
        }
    }
}

// Generate a binary image with non-zero for the given indices
void IndicesOfOrganizedPC2Image8u(pcl::PointIndices::Ptr indices_ptr, cv::Mat& image_8u)
{
    for(int k=0; k<indices_ptr->indices.size(); k++)
    {
        image_8u.at<uchar>(std::floor(indices_ptr->indices[k]/640), indices_ptr->indices[k]%640) = 255;
    }
}

ShapeTracker::ShapeTracker(omip::RB_id_t rb_id)
    : _rb_id(rb_id),
      _new_model(false),
      _current_dm(480, 640, CV_32FC1),
      _segment_of_current(480,640,CV_8U),
      _segment_of_current_dilated(480,640,CV_8U)
{

    _rb_model.reset(new pcl::PointCloud<pcl::PointXYZ >());

    _icp.setMaxCorrespondenceDistance(1);
    _icp.setEuclideanFitnessEpsilon(1e-6);
    _icp.setMaximumIterations(1000);
    _icp.setTransformationEpsilon(0.0);

    std::stringstream pub_topic_name;
    pub_topic_name << "/shape_tracker/segment_rb" << this->_rb_id;
    _segment_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(pub_topic_name.str(), 1);

    int dilation_type = cv::MORPH_RECT;
    int dilation_size = 2;
    _dilation_element = cv::getStructuringElement( dilation_type,
                                                   cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                   cv::Point( dilation_size, dilation_size ) );

#ifdef USE_LIBPOINTMATCHER_ICP
    _icp_lpm = new PM::ICP();

    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    name = "MaxPointCountDataPointsFilter";
    params["maxCount"] = "1000";
    PM::DataPointsFilter* maxCount_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "BoundingBoxDataPointsFilter";
    params["xMin"] = "-inf";
    params["xMax"] = "inf";
    params["yMin"] = "-inf";
    params["yMax"] = "inf";
    params["zMin"] = "-inf";
    params["zMax"] = "inf";
    params["removeInside"] = "0";
    PM::DataPointsFilter* bb_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare matching function
    name = "KDTreeMatcher";
    params["knn"] = "1";
    params["epsilon"] = "0";
    params["maxDist"] = "0.02";
    params["searchType"] = "1";
    PM::Matcher* kdtree =
            PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    // Prepare outlier filters
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    PM::OutlierFilter* trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();

    // Prepare error minimization
    name = "PointToPointErrorMinimizer";
    PM::ErrorMinimizer* pointToPoint =
            PM::get().ErrorMinimizerRegistrar.create(name);

    // Prepare outlier filters
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "10";
    PM::TransformationChecker* maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "DifferentialTransformationChecker";
    params["minDiffRotErr"] = "0.01";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "1";
    PM::TransformationChecker* diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    // Prepare inspector
    PM::Inspector* nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");

    // Prepare transformation
    PM::Transformation* rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    _icp_lpm->readingDataPointsFilters.push_back(maxCount_read);
    _icp_lpm->referenceDataPointsFilters.push_back(bb_read);
    _icp_lpm->referenceDataPointsFilters.push_back(maxCount_read);
    _icp_lpm->matcher.reset(kdtree);
    _icp_lpm->outlierFilters.push_back(trim);
    _icp_lpm->errorMinimizer.reset(pointToPoint);
    _icp_lpm->transformationCheckers.push_back(maxIter);
    _icp_lpm->transformationCheckers.push_back(diff);
    _icp_lpm->inspector.reset(nullInspect);
    _icp_lpm->transformations.push_back(rigidTrans);
#endif
}

ShapeTracker::ShapeTracker(const ShapeTracker &sr)
{
    ROS_ERROR_STREAM_NAMED("ShapeTracker.ShapeTracker", "Do not use this copy constructor! It is not complete!");

    this->_rb_id = sr._rb_id;
    this->_rb_model = sr._rb_model;
}

ShapeTracker::~ShapeTracker()
{
}

void ShapeTracker::setCameraInfo(const sensor_msgs::CameraInfo& camera_info)
{
    this->_ci = sensor_msgs::CameraInfo(camera_info);
}

void OrganizedPC2DepthMap2(const pcl::PointCloud<pcl::PointXYZ >::Ptr organized_pc, cv::Mat& depth_map_mat)
{
    float* data_ptr = (float*)depth_map_mat.data;
    size_t elem_step = depth_map_mat.step / sizeof(float);

    for(int v=0; v< organized_pc->height; v++)
    {
        for(int u=0; u< organized_pc->width; u++)
        {
            if(pcl::isFinite(organized_pc->points[v*organized_pc->width + u]))
                data_ptr[v*elem_step + u] = organized_pc->points[v*organized_pc->width + u].z;
            else
                data_ptr[v*elem_step + u] = nanf("");
        }
    }
}


void ShapeTracker::step(const sensor_msgs::PointCloud2ConstPtr &pc_msg,
                        const omip_msgs::RigidBodyPoseAndVelMsg& tracked_rbm,
                        omip_msgs::ShapeTrackerState& st_state)
{
    if(this->_rb_model->points.size() != 0)
    {
        ros::WallTime t1 = ros::WallTime::now();

        // Convert from exponential coordinates to transformation matrix
        Eigen::Twistd pose_twist2;
        ROSTwist2EigenTwist(tracked_rbm.pose_wc.twist, pose_twist2);
        Eigen::Matrix4d pose_matrix;
        Twist2TransformMatrix(pose_twist2, pose_matrix);

        // I can move the model to the current pose (using the pose_matrix) and estimate its bounding box. Then update the
        // parameters of the bounding box filter for the input point cloud
        pcl::PointCloud<pcl::PointXYZ >::Ptr rb_model_current_pose(new pcl::PointCloud<pcl::PointXYZ >);
        pcl::transformPointCloud<pcl::PointXYZ>(*this->_rb_model, *rb_model_current_pose, pose_matrix.cast<float>());
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;
        pcl::getMinMax3D<pcl::PointXYZ>(*rb_model_current_pose, min_pt, max_pt);
        double safety_margin = 0.01;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["xMin"] = min_pt.x - safety_margin;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["xMax"] = max_pt.x + safety_margin;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["yMin"] = min_pt.y - safety_margin;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["yMax"] = max_pt.y + safety_margin;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["zMin"] = min_pt.z - safety_margin;
        _icp_lpm->referenceDataPointsFilters.at(0)->parameters["zMax"] = max_pt.z + safety_margin;

        sensor_msgs::PointCloud2 model_now;
        pcl::toROSMsg(*rb_model_current_pose, model_now);
        model_now.header.frame_id = "/camera_rgb_optical_frame";
        this->_segment_pub.publish(model_now);

        ros::WallTime t4 = ros::WallTime::now();

        ros::WallTime t5 = ros::WallTime::now();

        bool lpm_converged = true;

        // Convert current point cloud into libpointmatcher
        boost::shared_ptr<DP> current_segment_of_pc_extended_lpm(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(*pc_msg)));

        ros::WallTime t6 = ros::WallTime::now();

        // Compute the transformation to express data in ref
        PM::TransformationParameters T;
        try{
            std::cout << pose_matrix << std::endl;
            // Call libpointmatcher passing as initial guess for the transformation the received one
            T = _icp_lpm->operator()(*_rb_model_lpm, *current_segment_of_pc_extended_lpm, pose_matrix.cast<float>());
            std::cout << T << std::endl;
        }catch(...)
        {
            ROS_ERROR_STREAM("[RB" << this->_rb_id << "] Libmatchpoint ICP did not converge!");
            lpm_converged = false;
            st_state.fitness_score = 1;
        }

        ros::WallTime t7 = ros::WallTime::now();

        ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsFromRBModel",
                              "[RB" << this->_rb_id << " Estimated LPM ICP refinement: " << std::endl << T);

        ROS_INFO_STREAM_NAMED("ShapeReconstruction.step",
                              "t4 - t1: " <<  t4 - t1  );
        ROS_INFO_STREAM_NAMED("ShapeReconstruction.step",
                              "t5 - t4: " <<  t5 - t4  );
        ROS_INFO_STREAM_NAMED("ShapeReconstruction.step",
                              "t6 - t5: " <<  t6 - t5  );
        ROS_INFO_STREAM_NAMED("ShapeReconstruction.step",
                              "t7 - t6: " <<  t7 - t6  );

        // Using LPM ICP results
        if(lpm_converged)
        {
            Eigen::Twistd compound_twist;
            TransformMatrix2Twist( T.cast<double>(), compound_twist);

            EigenTwist2GeometryMsgsTwist(compound_twist, st_state.pose_wc.twist);

            st_state.rb_id = this->_rb_id;
            //st_state.fitness_score = this->_icp.getFitnessScore();
            st_state.fitness_score = 0;

            for (unsigned int i = 0; i < 6; i++)
            {
                for (unsigned int j = 0; j < 6; j++)
                {
                    st_state.pose_wc.covariance[6 * i + j] = this->_icp_lpm->errorMinimizer->getCovariance()(i, j);
                }
            }
            //std::cout  << this->_icp_lpm->errorMinimizer->getWeightedPointUsedRatio() << std::endl;

            //st_state.number_of_points_of_model = _icp.getInputSource()->points.size();
            st_state.number_of_points_of_model = this->_icp_lpm->getPrefilteredReadingPtsCount();
            std::cout << this->_icp_lpm->getPrefilteredReadingPtsCount() << std::endl;
            //st_state.number_of_points_of_current_pc = _icp.getInputTarget()->points.size();
            st_state.number_of_points_of_current_pc = this->_icp_lpm->getPrefilteredReferencePtsCount();
            std::cout << this->_icp_lpm->getPrefilteredReferencePtsCount() << std::endl;

        }else{
            st_state.pose_wc.twist = tracked_rbm.pose_wc.twist;
            st_state.number_of_points_of_model = 0;
            st_state.number_of_points_of_current_pc =0;
        }

        ROS_WARN_STREAM("Before refinement: \t" << tracked_rbm.pose_wc.twist.linear.x << " " <<
                        tracked_rbm.pose_wc.twist.linear.y << " " <<
                        tracked_rbm.pose_wc.twist.linear.z << " " <<
                        tracked_rbm.pose_wc.twist.angular.x << " " <<
                        tracked_rbm.pose_wc.twist.angular.y << " " <<
                        tracked_rbm.pose_wc.twist.angular.z );

        ROS_WARN_STREAM("After refinement: \t" <<st_state.pose_wc.twist.linear.x << " " <<
                        st_state.pose_wc.twist.linear.y << " " <<
                        st_state.pose_wc.twist.linear.z << " " <<
                        st_state.pose_wc.twist.angular.x << " " <<
                        st_state.pose_wc.twist.angular.y << " " <<
                        st_state.pose_wc.twist.angular.z );

        st_state.probabilistic_value = 1.0;
    }else{
        st_state.pose_wc.twist = tracked_rbm.pose_wc.twist;
        ROS_WARN_STREAM("Model is empty! Same twist as the input: " << st_state.pose_wc.twist.linear.x << " " <<
                        st_state.pose_wc.twist.linear.y << " " <<
                        st_state.pose_wc.twist.linear.z << " " <<
                        st_state.pose_wc.twist.angular.x << " " <<
                        st_state.pose_wc.twist.angular.y << " " <<
                        st_state.pose_wc.twist.angular.z );

        st_state.rb_id = this->_rb_id;
        st_state.fitness_score = std::numeric_limits<float>::infinity();

        st_state.probabilistic_value = 1.0;

        st_state.number_of_points_of_model = 0;
        st_state.number_of_points_of_current_pc = 0;
    }
}

void ShapeTracker::setShapeModel(const sensor_msgs::PointCloud2& model)
{
    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(model, *this->_rb_model);

    // If the model is new we convert it into libpointmatcher
    sensor_msgs::PointCloud2 rb_model_ros;
    pcl::toROSMsg(*this->_rb_model, rb_model_ros);
    _rb_model_lpm.reset(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(rb_model_ros)));
}

void ShapeTracker::_RemoveInconsistentPointsFromRBModel(pcl::PointCloud<pcl::PointXYZ >::Ptr model_current_pose,
                                                        pcl::PointCloud<pcl::PointXYZ >::Ptr current_pc,
                                                        pcl::PointCloud<pcl::PointXYZ >::Ptr& rb_segment,
                                                        pcl::PointCloud<pcl::PointXYZ >::Ptr& current_pc_extended_segment)
{
    // Estimate the depth map from the current point cloud (organized)
    OrganizedPC2DepthMap2(current_pc, this->_current_dm);

    pcl::PointIndicesPtr indices_to_remove(new pcl::PointIndices);

    // These are the points in the current pc that are consistent with the moved model
    // -> the current visible segment
    pcl::PointIndicesPtr indices_matching_in_current(new pcl::PointIndices);

    // These are the points of the model that are visible now
    pcl::PointIndicesPtr indices_matching_in_model(new pcl::PointIndices);

    // Find points that are inconsistent between moved rb_shape and current depth image
    // Detect the points of the current PC and of the moved model that have neighbors
    _FindInconsistentPoints(model_current_pose, // input
                            this->_current_dm, // input
                            indices_to_remove, // output
                            indices_matching_in_model, // output
                            indices_matching_in_current // output
                            );

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsFromRBModel",
                          "Matched points in current point cloud: " << indices_matching_in_current->indices.size());
    ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsFromRBModel",
                          "Matched points in model: " << indices_matching_in_model->indices.size());

    // Convert the indices in a binary image (white == current segment)
    this->_segment_of_current.setTo(0);
    IndicesOfOrganizedPC2Image8u(indices_matching_in_current, this->_segment_of_current);

    // Dilate the current segment -> Covers more area for ICP
    cv::dilate(this->_segment_of_current, this->_segment_of_current_dilated, this->_dilation_element);

    // Convert the dilated binary image into a set of point indices (of the current pc)
    pcl::PointIndicesPtr indices_matching_in_current_extended(new pcl::PointIndices);
    Image8u2IndicesOfOrganizedPC(this->_segment_of_current_dilated, indices_matching_in_current_extended);

    ROS_INFO_STREAM_NAMED("ShapeReconstruction._RemoveInconsistentPointsFromRBModel",
                          "Matched points in current point cloud extended: " << indices_matching_in_current_extended->indices.size());

    // Extract the part of the model that have neighbors (moved to current frame)
    _extractor.setNegative(false);
    _extractor.setInputCloud(model_current_pose);
    _extractor.setIndices(indices_matching_in_model);
    _extractor.setKeepOrganized(false);
    _extractor.filter(*rb_segment);

    // Extract the part of the point cloud that have neighbors, extended with the dilate operation
    _extractor.setInputCloud(current_pc);
    _extractor.setIndices(indices_matching_in_current_extended);
    _extractor.setKeepOrganized(false);
    _extractor.filter(*current_pc_extended_segment);
}

void ShapeTracker::_FindInconsistentPoints(const pcl::PointCloud<pcl::PointXYZ >::Ptr& pc_source,
                                           const cv::Mat & dm_true,
                                           pcl::PointIndicesPtr& indices_to_remove,
                                           pcl::PointIndicesPtr& indices_matching_in_true,
                                           pcl::PointIndicesPtr& indices_matching_in_dm,
                                           const double min_depth_error)
{
    indices_to_remove->indices.clear();

    using ::shape_reconstruction::RangeImagePlanar;
    RangeImagePlanar::Ptr dm_source_rip(new RangeImagePlanar);

    Eigen::Affine3f sensor_pose;
    sensor_pose.matrix() = Eigen::Matrix4f::Identity();
    pcl::RangeImagePlanar::CoordinateFrame coordinate_frame = pcl::RangeImagePlanar::CAMERA_FRAME;

    int width = dm_true.cols, height = dm_true.rows;
    dm_source_rip->matchPointCloudAndImage (*pc_source,
                                            width,
                                            height,
                                            this->_ci.P[2], //width/2 -0.5, //319.5, // 329.245223575443,
            this->_ci.P[6], //height/2 - 0.5, // 239.5, //  239.458
            this->_ci.P[0], // fx
            this->_ci.P[5], // fy
            sensor_pose,
            coordinate_frame,
            dm_true,
            min_depth_error,
            indices_matching_in_true,
            indices_matching_in_dm,
            indices_to_remove
            );

}

/*************************************
 * The inputs for this function are
 * data_pi --> the corresponding points in the data_cloud or source cloud or Pi
 * model_qi --> the correspondences in the model or target or Qi
 * RT --> the transformation matrix as returned by ICP
 * Output is a 6x6 matrix which is the COVARIANCE of ICP in [x,y,z,a,b,c]
 * [a b c] are Yaw, Pitch and Roll respectively
***************************************/
void calculate_ICP_COV(pcl::PointCloud<pcl::PointXYZ>& data_pi, pcl::PointCloud<pcl::PointXYZ>& model_qi, Eigen::Matrix4f& transform, Eigen::MatrixXd& ICP_COV)
{

    double Tx = transform(0,3);
    double Ty = transform(1,3);
    double Tz = transform(2,3);
    double roll  = atan2f(transform(2,1), transform(2,2));
    double pitch = asinf(-transform(2,0));
    double yaw   = atan2f(transform(1,0), transform(0,0));

    double x, y, z, a, b, c;
    x = Tx; y = Ty; z = Tz;
    a = yaw; b = pitch; c = roll;// important // According to the rotation matrix I used and after verification, it is Yaw Pitch ROLL = [a,b,c]== [R] matrix used in the MatLab also :)

    /* Flushing out in the form of XYZ ABC */
    std::cout << "\nPrinting out [x, y, z, a, b, c] =  " <<x<<"    "<<y<<"    "<<z<<"    "<<a<<"    "<<b<<"    "<<c<<std::endl;

    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(6,6);
    d2J_dX2 = Eigen::MatrixXd::Zero(6,6);

    /****  Calculating d2J_dX2  ****/
    for (size_t s = 0; s < data_pi.points.size(); ++s )
    {
        double pix = data_pi.points[s].x;
        double piy = data_pi.points[s].y;
        double piz = data_pi.points[s].z;
        double qix = model_qi.points[s].x;
        double qiy = model_qi.points[s].y;
        double qiz = model_qi.points[s].z;

        /************************************************************

d2J_dX2 -- X is the [R|T] in the form of [x, y, z, a, b, c]
x, y, z is the translation part
a, b, c is the rotation part in Euler format
[x, y, z, a, b, c] is acquired from the Transformation Matrix returned by ICP.

Now d2J_dX2 is a 6x6 matrix of the form

d2J_dx2
d2J_dxdy    d2J_dy2
d2J_dxdz    d2J_dydz    d2J_dz2
d2J_dxda    d2J_dyda    d2J_dzda   d2J_da2
d2J_dxdb    d2J_dydb    d2J_dzdb   d2J_dadb   d2J_db2
d2J_dxdc    d2J_dydc    d2J_dzdc   d2J_dadc   d2J_dbdc   d2J_dc2


*************************************************************/

        double d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dcdx,
                d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;

        // These terms are generated from the provided Matlab scipts. We just have to copy
        // the expressions from the matlab output with two very simple changes.
        // The first one being the the sqaure of a number 'a' is shown as a^2 in matlab,
        // which is converted to pow(a,2) in the below expressions.
        // The second change is to add ';' at the end of each expression :)
        // In this way, matlab can be used to generate these terms for various objective functions of ICP
        // and they can simply be copied to the C++ files and with appropriate changes to ICP estimation,
        // its covariance can be easily estimated.
        d2J_dx2 = 2;

        d2J_dy2 = 2;

        d2J_dz2 = 2;

        d2J_dydx = 0;

        d2J_dxdy = 0;

        d2J_dzdx = 0;

        d2J_dxdz = 0;

        d2J_dydz = 0;

        d2J_dzdy = 0;

        d2J_da2 = (piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - (2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));

        d2J_db2 = (pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c))*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - (2*piz*cos(b)*cos(c) - 2*pix*sin(b) + 2*piy*cos(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - (2*pix*cos(a)*cos(b) + 2*piz*cos(a)*cos(c)*sin(b) + 2*piy*cos(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (2*pix*cos(b)*sin(a) + 2*piz*cos(c)*sin(a)*sin(b) + 2*piy*sin(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c));

        d2J_dc2 = (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) - (2*piz*cos(b)*cos(c) + 2*piy*cos(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (piy*cos(b)*cos(c) - piz*cos(b)*sin(c))*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - (2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dxda = 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*pix*cos(b)*sin(a);

        d2J_dadx = 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*pix*cos(b)*sin(a);

        d2J_dyda = 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b);

        d2J_dady = 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b);

        d2J_dzda = 0;

        d2J_dadz = 0;

        d2J_dxdb = 2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c);

        d2J_dbdx = 2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c);

        d2J_dydb = 2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c);

        d2J_dbdy = 2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c);

        d2J_dzdb = - 2*pix*cos(b) - 2*piz*cos(c)*sin(b) - 2*piy*sin(b)*sin(c);

        d2J_dbdz = - 2*pix*cos(b) - 2*piz*cos(c)*sin(b) - 2*piy*sin(b)*sin(c);

        d2J_dxdc = 2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));

        d2J_dcdx = 2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));

        d2J_dydc = - 2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));

        d2J_dcdy = - 2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));

        d2J_dzdc = 2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c);

        d2J_dcdz = 2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c);

        d2J_dadb = (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));

        d2J_dbda = (piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - (piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) + (2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));

        d2J_dbdc = (2*piy*cos(a)*cos(b)*cos(c) - 2*piz*cos(a)*cos(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (2*piy*cos(b)*cos(c)*sin(a) - 2*piz*cos(b)*sin(a)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c))*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) - (2*piy*cos(c)*sin(b) - 2*piz*sin(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)) - (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c));

        d2J_dcdb = (2*piy*cos(a)*cos(b)*cos(c) - 2*piz*cos(a)*cos(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + (2*piy*cos(b)*cos(c)*sin(a) - 2*piz*cos(b)*sin(a)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (piy*cos(b)*cos(c) - piz*cos(b)*sin(c))*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - (2*piy*cos(c)*sin(b) - 2*piz*sin(b)*sin(c))*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c));

        d2J_dcda = (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dadc = (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)))*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) + (2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        Eigen::MatrixXd d2J_dX2_temp(6,6);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dzdx,
                d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;

        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!

    std::cout << "\n**************\n Successfully Computed d2J_dX2 \n**************\n" << std::endl;

    // Now its time to calculate d2J_dZdX , where Z are the measurements Pi and Qi, X = [x,y,z,a,b,c]

    // n is the number of correspondences
    int n = data_pi.points.size();

    if (n > 200) n = 200;

    std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;

    Eigen::MatrixXd d2J_dZdX(6,6*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        double pix = data_pi.points[k].x;
        double piy = data_pi.points[k].y;
        double piz = data_pi.points[k].z;
        double qix = model_qi.points[k].x;
        double qiy = model_qi.points[k].y;
        double qiz = model_qi.points[k].z;

        Eigen::MatrixXd d2J_dZdX_temp(6,6);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                d2J_dpix_dz,    d2J_dpiy_dz,    d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                d2J_dpix_db,    d2J_dpiy_db,    d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                d2J_dpix_dc,    d2J_dpiy_dc,    d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;

        d2J_dpix_dx = 2*cos(a)*cos(b);

        d2J_dpix_dy = 2*cos(b)*sin(a);

        d2J_dpix_dz = -2*sin(b);

        d2J_dpix_da = cos(b)*sin(a)*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) - cos(a)*cos(b)*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - 2*cos(b)*sin(a)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(a)*cos(b)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dpix_db = sin(b)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) - 2*cos(b)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + cos(a)*cos(b)*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - 2*sin(a)*sin(b)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + cos(b)*sin(a)*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - 2*cos(a)*sin(b)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b));

        d2J_dpix_dc = cos(a)*cos(b)*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - sin(b)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - cos(b)*sin(a)*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)));

        d2J_dpiy_dx = 2*cos(a)*sin(b)*sin(c) - 2*cos(c)*sin(a);

        d2J_dpiy_dy = 2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c);

        d2J_dpiy_dz = 2*cos(b)*sin(c);

        d2J_dpiy_da = (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(c)*sin(a) - 2*cos(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dpiy_db = (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - 2*sin(b)*sin(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - cos(b)*sin(c)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) + 2*cos(a)*cos(b)*sin(c)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(b)*sin(a)*sin(c)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dpiy_dc = (2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(a)*sin(c) - 2*cos(c)*sin(a)*sin(b))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) - (cos(a)*cos(c) + sin(a)*sin(b)*sin(c))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) - (cos(c)*sin(a) - cos(a)*sin(b)*sin(c))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + 2*cos(b)*cos(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) + cos(b)*sin(c)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c));

        d2J_dpiz_dx = 2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b);

        d2J_dpiz_dy = 2*cos(c)*sin(a)*sin(b) - 2*cos(a)*sin(c);

        d2J_dpiz_dz = 2*cos(b)*cos(c);

        d2J_dpiz_da = (2*cos(a)*sin(c) - 2*cos(c)*sin(a)*sin(b))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a)) - (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*pix*cos(a)*cos(b)) + (2*sin(a)*sin(c) + 2*cos(a)*cos(c)*sin(b))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dpiz_db = (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piz*cos(a)*cos(b)*cos(c) - 2*pix*cos(a)*sin(b) + 2*piy*cos(a)*cos(b)*sin(c)) - (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piz*cos(b)*cos(c)*sin(a) - 2*pix*sin(a)*sin(b) + 2*piy*cos(b)*sin(a)*sin(c)) - 2*cos(c)*sin(b)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)) - cos(b)*cos(c)*(2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c)) + 2*cos(a)*cos(b)*cos(c)*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + 2*cos(b)*cos(c)*sin(a)*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a));

        d2J_dpiz_dc = (2*cos(c)*sin(a) - 2*cos(a)*sin(b)*sin(c))*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) - (2*cos(a)*cos(c) + 2*sin(a)*sin(b)*sin(c))*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + (sin(a)*sin(c) + cos(a)*cos(c)*sin(b))*(2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) + (cos(a)*sin(c) - cos(c)*sin(a)*sin(b))*(2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + cos(b)*cos(c)*(2*piy*cos(b)*cos(c) - 2*piz*cos(b)*sin(c)) - 2*cos(b)*sin(c)*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c));

        d2J_dqix_dx = -2;

        d2J_dqix_dy = 0;

        d2J_dqix_dz = 0;

        d2J_dqix_da = 2*piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*pix*cos(b)*sin(a);

        d2J_dqix_db = 2*pix*cos(a)*sin(b) - 2*piz*cos(a)*cos(b)*cos(c) - 2*piy*cos(a)*cos(b)*sin(c);

        d2J_dqix_dc = - 2*piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c));

        d2J_dqiy_dx = 0;

        d2J_dqiy_dy = -2;

        d2J_dqiy_dz = 0;

        d2J_dqiy_da = 2*piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - 2*piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*pix*cos(a)*cos(b);

        d2J_dqiy_db = 2*pix*sin(a)*sin(b) - 2*piz*cos(b)*cos(c)*sin(a) - 2*piy*cos(b)*sin(a)*sin(c);

        d2J_dqiy_dc = 2*piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c));

        d2J_dqiz_dx = 0;

        d2J_dqiz_dy = 0;

        d2J_dqiz_dz = -2;

        d2J_dqiz_da = 0;

        d2J_dqiz_db =  2*pix*cos(b) + 2*piz*cos(c)*sin(b) + 2*piy*sin(b)*sin(c);

        d2J_dqiz_dc =  2*piz*cos(b)*sin(c) - 2*piy*cos(b)*cos(c);

        d2J_dZdX_temp <<    d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                d2J_dpix_dz,    d2J_dpiy_dz,        d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                d2J_dpix_da,    d2J_dpiy_da,        d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                d2J_dpix_db,    d2J_dpiy_db,        d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                d2J_dpix_dc,    d2J_dpiy_dc,        d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;


        d2J_dZdX.block<6,6>(0,6*k) = d2J_dZdX_temp;
    }

    /**************************************
     *
     * Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper, "Covariance of ICP with 3D Point to Point and Point to Plane Error Metrics"
     *
     * ************************************/

    Eigen::MatrixXd cov_z(6*n,6*n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(6*n,6*n);

    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();

    std::cout << "\n\n********************** \n\n" << "ICP_COV = \n" << ICP_COV <<"\n*******************\n\n"<< std::endl;

    std::cout << "\nSuccessfully Computed the ICP's Covariance !!!\n" << std::endl;
}
