/*
 * OMIPUtils.h
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

#ifndef OMIP_UTILS_H_
#define OMIP_UTILS_H_

#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <lgsm/Lgsm>
#include <boost/shared_ptr.hpp>

#include "omip_common/Feature.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "wrappers/matrix/matrix_wrapper.h"

#include "omip_common/OMIPTypeDefs.h"

namespace omip
{

/**
 * Convert an Eigen::Affine3d matrix into translation and rotation (Euler angles)
 * @param t - Eigen Affine3d (input)
 * @param x - Translation in x (output)
 * @param y - Translation in y (output)
 * @param z - Translation in z (output)
 * @param roll - Rotation roll (output)
 * @param pitch - Rotation pitch (output)
 * @param yaw - Rotation yaw (output)
 */
void EigenAffine2TranslationAndEulerAngles(const Eigen::Affine3d& t, double& x,
                                  double& y, double& z, double& roll,
                                  double& pitch, double& yaw);

/**
 * Convert translation and rotation (Euler angles) into an Eigen::Affine3d matrix
 * @param x - Translation in x (input)
 * @param y - Translation in x (input)
 * @param z - Translation in x (input)
 * @param roll - Rotation roll (input)
 * @param pitch - Rotation pitch (input)
 * @param yaw - Rotation yaw (input)
 * @param t - Eigen Affine3d (output)
 */
void TranslationAndEulerAngles2EigenAffine(const double& x, const double& y, const double& z, const double& roll, const double& pitch,
                       const double& yaw,
                       Eigen::Transform<double, 3, Eigen::Affine> &t);

/**
 * Retrieve one sample of a normal (Gaussian) distribution, given its mean and standard deviation
 * @param mean - Mean value of the Gaussian
 * @param std_dev - Std deviation of the Gaussian
 * @return - One sample of the Gaussian distribution
 */
double sampleNormal(double mean, double std_dev);

/**
 * Operator << for ostream to add a vector of Feature Ids
 * @param os - Input ostream
 * @param vector_ids - Vector of Feature Ids to add to the ostream
 * @return - ostream with the added vector
 */
std::ostream& operator <<(std::ostream& os,
                          std::vector<Feature::Id> vector_ids);

/**
 * Operator << for ostream to add the Location of a Feature
 * @param os - Input ostream
 * @param location - Location of a Feature to add to the ostream
 * @return - ostream with the added Location
 */
std::ostream& operator <<(std::ostream& os, Feature::Location location);


std::ostream& operator <<(std::ostream& os, Eigen::Twistd twistd);

/**
 * Compute the L2 distance between two Feature Locations. The Locations can represent
 * the same Feature in two time steps or two different Features
 * @param first - First Feature location
 * @param second - Second Feature location
 * @return - L2 distance between first and second Feature Locations
 */
double L2Distance(const Feature::Location& first, const Feature::Location& second);

/**
 * DEPRECATED!
 * This function is dangerous because it uses internally the function log to convert the transformation matrix
 * into an element of the Lie algebra and this function is discontinuous!
 * DO NOT USE THIS FUNCTION!
 * Convert an Eigen Matrix (4x4) of a homogeneous transformation to an Eigen Twist
 * @param transformation_matrix - Eigen matrix of the transformation
 * @return - Twist of the transformation
 */
void TransformMatrix2Twist(const Eigen::Matrix4d& transformation_matrix, Eigen::Twistd& twist);

/**
 * Convert an Eigen Matrix (4x4) of a homogeneous transformation to an Eigen Twist
 * @param transformation_matrix - Eigen matrix of the transformation
 * @return - Twist of the transformation
 */
void TransformMatrix2TwistUnwrapped(const Eigen::Matrix4d& transformation_matrix, Eigen::Twistd& twist, const Eigen::Twistd& twist_previous);

/**
 * Convert an Eigen Twist to an Eigen Matrix (4x4) homogeneous transformation
 * @param transformation_twist - Eigen twist of the transformation
 * @return - Eigen Matrix (4x4) homogeneous transformation
 */
void Twist2TransformMatrix( const Eigen::Twistd& transformation_twist, Eigen::Matrix4d& matrix);

/**
 * Transform the location of a Feature using a rigid body transformation (matrix)
 * @param origin - Original location of the Feature
 * @param transformation - Rigid body transformation (pose)
 * @return - New location of the Feature
 */
void TransformLocation(const Feature::Location& origin, const Eigen::Matrix4d& transformation, Feature::Location& new_location);

/**
 * Transform the location of a Feature using a rigid body transformation (twist)
 * @param origin - Original location of the Feature
 * @param twist - Rigid body transformation (pose)
 * @return - New location of the Feature
 */
void TransformLocation(const Feature::Location& origin, const Eigen::Twistd& twist, Feature::Location &new_location, int feat_id=0);

/**
 * Transform the location of a Feature using a rigid body transformation (twist)
 * @param origin - Original location of the Feature
 * @param twist - Rigid body transformation (pose)
 * @return - New location of the Feature
 */
void TransformLocation(const Feature::Location& origin, const geometry_msgs::Twist& twist,Feature::Location& new_location);

/**
 * Convert the Location of a Feature to a ColumnVector (3x1)
 * @param lof - Location of a Feature
 * @return - Column vector (3x1)
 */
void LocationOfFeature2ColumnVector(const Feature::Location& lof, MatrixWrapper::ColumnVector& col_vec);

/**
 * Convert the Location of a Feature to a ColumnVector in homogeneous coordinates (4x1)
 * @param lof - Location of a Feature
 * @return - Column vector homogeneous (4x1)
 */
void LocationOfFeature2ColumnVectorHomogeneous(const Feature::Location &lof, MatrixWrapper::ColumnVector &col_vec);

void LocationOfFeature2EigenVectorHomogeneous(const Feature::Location& lof, Eigen::Vector4d& eig_vec);

/**
 * Operator - for two Feature Locations
 * @param location1 - First Feature Location
 * @param location2 - Second Feature Location
 * @return - Subtraction of the first Feature Location minus the second Feature Location (it should be a
 * vector but for coherence we return a Feature Location)
 */
Feature::Location operator-(const Feature::Location& location1,
                            const Feature::Location& location2);

Feature::Location operator+(const Feature::Location& location1,
                            const Feature::Location& location2);
/**
 * Convert a ROS geometry message Twist into an Eigen Twist
 * @param gm_twist - ROS geometry message Twist
 * @return - Eigen twist
 */
void GeometryMsgsTwist2EigenTwist(const geometry_msgs::Twist& gm_twist, Eigen::Twistd& eigen_twist);

/**
 * Convert an Eigen Twist into a ROS geometry message Twist
 * @param eigen_twist - Eigen twist
 * @return - ROS geometry message Twist
 */
void EigenTwist2GeometryMsgsTwist(Eigen::Twistd& eigen_twist, geometry_msgs::Twist& gm_twist);

/**
 * Checks if all the elements of the Eigen matrix are finite
 * @param transformation - Eigen matrix to test
 * @return - TRUE if all elements of the matrix are finite
 */
bool isFinite(const Eigen::Matrix4d& transformation);

void Location2PointPCL(const Feature::Location &point_location, pcl::PointXYZ& point_pcl);

void LocationAndId2FeaturePCL(const Feature::Location &feature_location, const Feature::Id &feature_id, pcl::PointXYZL& feature_pcl);

void LocationAndId2FeaturePCLwc(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCLwc& feature_pcl);

void ROSTwist2EigenTwist(const geometry_msgs::Twist& ros_twist, Eigen::Twistd &eigen_twist);

Eigen::Twistd unwrapTwist(Eigen::Twistd& current_twist, Eigen::Displacementd& current_displacement, Eigen::Twistd& previous_twist, bool &changed);

Eigen::Twistd invertTwist(Eigen::Twistd& current_twist, Eigen::Twistd& previous_twist, bool& inverted);

void invert3x3Matrix(const MatrixWrapper::Matrix& to_inv, MatrixWrapper::Matrix& inverse);

void invert3x3MatrixEigen(const Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& to_inv,
                          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& inverse);

void invert3x3MatrixEigen2(const Eigen::Matrix3d& to_inv, Eigen::Matrix3d& inverse);

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

/**
 * @brief computeAdjoint Returns the adjoint matrix of the given pose. Necessary because the LGSM library represents first rotation
 * and then translation while we represent first translation and then rotation
 * @param pose_ec Pose (in exponential coordinates) to obtain the adjoint for
 * @param adjoint Matrix containing the adjoint. 3 first rows are translation and 3 last rows are rotation
 */
void computeAdjoint(const Eigen::Twistd& pose_ec, Eigen::Matrix<double, 6, 6>& adjoint_out);

/**
 * @brief adjointXcovXadjointT Transforms a covariance from one reference frame to another
 * @param pose_ec Pose (in exponential coordinates) where we want to have the covariance expressed
 * @param cov Original covariance
 * @param transformed_cov Transformed covariance
 */
void adjointXcovXadjointT(const Eigen::Twistd& pose_ec, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out);

/**
 * Same as the functions above but the pose is passed as a displacement
 */
void computeAdjoint(const Eigen::Displacementd pose_disp, Eigen::Matrix<double, 6, 6> &adjoint_out);
void adjointXcovXadjointT(const Eigen::Displacementd &pose_disp, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out);

void adjointXinvAdjointXcovXinvAdjointTXadjointT(const Eigen::Displacementd& pose_disp1,
                                                       const Eigen::Displacementd& pose_disp2,
                                                       const Eigen::Matrix<double, 6, 6>& cov,
                                                       Eigen::Matrix<double, 6, 6>& transformed_cov_out);

void findIntersectionOfEllipsoidAndPlane(const Eigen::Matrix3d& ellipsoid,
                                               const Eigen::Vector3d& normal_plane,
                                               double& r1,
                                               double& r2,
                                               Eigen::Matrix3d& full_rotation);

}

#endif /* OMIP_UTILS_H_ */
