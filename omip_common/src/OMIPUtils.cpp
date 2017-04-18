#include <lgsm/Lgsm>

#include "omip_common/OMIPUtils.h"

#include <math.h>

#include <tr1/random>

#include <iostream>
#include <fstream>

#include <Eigen/Eigenvalues>
#include <boost/math/distributions/chi_squared.hpp>

using namespace omip;

void omip::EigenAffine2TranslationAndEulerAngles(const Eigen::Affine3d& t,
                                                 double& x, double& y, double& z,
                                                 double& roll, double& pitch,
                                                 double& yaw)
{
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    roll = atan2(t(2, 1), t(2, 2));
    pitch = asin(-t(2, 0));
    yaw = atan2(t(1, 0), t(0, 0));
}

void omip::TranslationAndEulerAngles2EigenAffine(
        const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw,
        Eigen::Transform<double, 3, Eigen::Affine> &t)
{
    double A = cos(yaw), B = sin(yaw), C = cos(pitch), D = sin(pitch), E = cos(roll), F = sin(roll), DE = D * E, DF = D * F;

    t(0, 0) = A * C;
    t(0, 1) = A * DF - B * E;
    t(0, 2) = B * F + A * DE;
    t(0, 3) = x;
    t(1, 0) = B * C;
    t(1, 1) = A * E + B * DF;
    t(1, 2) = B * DE - A * F;
    t(1, 3) = y;
    t(2, 0) = -D;
    t(2, 1) = C * F;
    t(2, 2) = C * E;
    t(2, 3) = z;
    t(3, 0) = 0;
    t(3, 1) = 0;
    t(3, 2) = 0;
    t(3, 3) = 1;
}

double omip::sampleNormal(double mean, double std_dev)
{
    using namespace boost;
    static std::tr1::mt19937 rng(static_cast<unsigned>(std::time(0)));

    std::tr1::normal_distribution<double> norm_dist(mean, std_dev);

    std::tr1::variate_generator<std::tr1::mt19937&, std::tr1::normal_distribution<double> > normal_sampler(
                rng, norm_dist);

    return (normal_sampler());
}

std::ostream& omip::operator <<(std::ostream& os,
                                std::vector<Feature::Id> vector_ids)
{
    for (size_t idx = 0; idx < vector_ids.size(); idx++)
    {
        os << vector_ids.at(idx) << " ";
    }
    return (os);
}

std::ostream& omip::operator <<(std::ostream& os, Feature::Location location)
{
    os << "(" << boost::tuples::get<0>(location) << ","
       << boost::tuples::get<1>(location) << ","
       << boost::tuples::get<2>(location) << ")";

    return (os);
}

std::ostream& omip::operator <<(std::ostream& os, Eigen::Twistd twistd)
{
    os << "(" << twistd.vx() << ","
       << twistd.vy() << ","
       << twistd.vz() << ","
       << twistd.rx() << ","
       << twistd.ry() << ","
       << twistd.rz() << ")";

    return (os);
}

double omip::L2Distance(const Feature::Location &first, const Feature::Location &second)
{
    double error_value = sqrt( pow(boost::tuples::get<0>(first) - boost::tuples::get<0>(second), 2)
                               + pow(boost::tuples::get<1>(first) - boost::tuples::get<1>(second), 2)
                               + pow(boost::tuples::get<2>(first) - boost::tuples::get<2>(second), 2));
    return error_value;
}

void omip::TransformMatrix2Twist(const Eigen::Matrix4d& transformation_matrix, Eigen::Twistd &twist)
{
    //std::cout << "Error! Do not use TransformMatrix2Twist. It uses the log to convert from lie group to lie algebra and log is dicontinuous. Use TransformMatrix2TwistUnwrapped instead!" << std::endl;
    //getchar();
    Eigen::Displacementd displ_from_transf(transformation_matrix);
    twist = displ_from_transf.log(1e-12);
}

void omip::TransformMatrix2TwistUnwrapped(const Eigen::Matrix4d& transformation_matrix, Eigen::Twistd &twist, const Eigen::Twistd &twist_previous)
{
    Eigen::Displacementd displ_from_transf(transformation_matrix);
    twist = displ_from_transf.log(1e-12);
}

void omip::Twist2TransformMatrix(const Eigen::Twistd& transformation_twist, Eigen::Matrix4d &matrix)
{
    Eigen::Displacementd displ_from_twist = transformation_twist.exp(1e-12);
    matrix = displ_from_twist.toHomogeneousMatrix();
}

void omip::TransformLocation(const Feature::Location& origin, const Eigen::Matrix4d& transformation, Feature::Location& new_location)
{
    Eigen::Affine3d motion_matrix(transformation);
    Eigen::Vector3d origin_vector = Eigen::Vector3d(
                boost::tuples::get<0>(origin), boost::tuples::get<1>(origin),
                boost::tuples::get<2>(origin));
    Eigen::Vector3d destination_vector = motion_matrix * origin_vector;
    boost::tuples::get<0>(new_location) = destination_vector[0];
    boost::tuples::get<1>(new_location)= destination_vector[1];
    boost::tuples::get<2>(new_location) = destination_vector[2];
}

void omip::TransformLocation(const Feature::Location& origin, const Eigen::Twistd& twist, Feature::Location& new_location, int feat_id)
{
    Eigen::Matrix4d eigen_transform;
    omip::Twist2TransformMatrix(twist, eigen_transform);

    omip::TransformLocation(origin, eigen_transform, new_location);

    //  std::ofstream feat_innovation;
    //  std::ostringstream name_file;
    //  name_file << "/home/roberto/Desktop/twists_and_Ts.txt";
    //  feat_innovation.open (name_file.str().c_str(), std::ios::app);

    //  feat_innovation << feat_id  << std::endl;
    //  feat_innovation << boost::tuples::get<0>(origin) << " " << boost::tuples::get<1>(origin) << " " << boost::tuples::get<2>(origin) << " "  << std::endl;
    //  Eigen::Twistd copy = twist;
    //  Eigen::Vector3d angular_part = Eigen::Vector3d(copy.rx(),copy.ry(),copy.rz() );
    //  feat_innovation << (angular_part.norm() - 2*M_PI)  << std::endl;
    //  feat_innovation << boost::tuples::get<0>(new_location) << " " << boost::tuples::get<1>(new_location) << " " << boost::tuples::get<2>(new_location) << " "  << std::endl;
    //  feat_innovation << twist << std::endl;
    //  feat_innovation << eigen_transform << std::endl<< std::endl;
    //  feat_innovation.close();
}

void omip::TransformLocation(const Feature::Location& origin, const geometry_msgs::Twist& twist, Feature::Location& new_location)
{
    Eigen::Twistd eigen_twist;
    GeometryMsgsTwist2EigenTwist(twist, eigen_twist);
    omip::TransformLocation(origin, eigen_twist, new_location);
}

void omip::LocationOfFeature2ColumnVector(const Feature::Location &lof, MatrixWrapper::ColumnVector &col_vec)
{
    col_vec(1) = boost::tuples::get<0>(lof);
    col_vec(2) = boost::tuples::get<1>(lof);
    col_vec(3) = boost::tuples::get<2>(lof);
}

void omip::LocationOfFeature2ColumnVectorHomogeneous(const Feature::Location& lof, MatrixWrapper::ColumnVector& col_vec)
{
    col_vec(1) = boost::tuples::get<0>(lof);
    col_vec(2) = boost::tuples::get<1>(lof);
    col_vec(3) = boost::tuples::get<2>(lof);
    col_vec(4) = 1;
}

void omip::LocationOfFeature2EigenVectorHomogeneous(const Feature::Location& lof, Eigen::Vector4d& eig_vec)
{
    eig_vec(0) = boost::tuples::get<0>(lof);
    eig_vec(1) = boost::tuples::get<1>(lof);
    eig_vec(2) = boost::tuples::get<2>(lof);
    eig_vec(3) = 1;
}

Feature::Location omip::operator-(const Feature::Location& location1,
                                  const Feature::Location& location2)
{
    return Feature::Location(
                boost::tuples::get<0>(location1) - boost::tuples::get<0>(location2),
                boost::tuples::get<1>(location1) - boost::tuples::get<1>(location2),
                boost::tuples::get<2>(location1) - boost::tuples::get<2>(location2));
}

Feature::Location omip::operator+(const Feature::Location& location1,
                                  const Feature::Location& location2)
{
    return Feature::Location(
                boost::tuples::get<0>(location1) + boost::tuples::get<0>(location2),
                boost::tuples::get<1>(location1) + boost::tuples::get<1>(location2),
                boost::tuples::get<2>(location1) + boost::tuples::get<2>(location2));
}

void omip::GeometryMsgsTwist2EigenTwist(const geometry_msgs::Twist& gm_twist, Eigen::Twistd &eigen_twist)
{
    eigen_twist = Eigen::Twistd(gm_twist.angular.x, gm_twist.angular.y, gm_twist.angular.z,
                                gm_twist.linear.x, gm_twist.linear.y, gm_twist.linear.z);
}

void omip::EigenTwist2GeometryMsgsTwist(Eigen::Twistd& eigen_twist, geometry_msgs::Twist &gm_twist)
{
    gm_twist.linear.x = eigen_twist.getLinearVelocity().x();
    gm_twist.linear.y = eigen_twist.getLinearVelocity().y();
    gm_twist.linear.z = eigen_twist.getLinearVelocity().z();
    gm_twist.angular.x = eigen_twist.getAngularVelocity().x();
    gm_twist.angular.y = eigen_twist.getAngularVelocity().y();
    gm_twist.angular.z = eigen_twist.getAngularVelocity().z();
}

bool omip::isFinite(const Eigen::Matrix4d& transformation)
{
    bool ret_val = true;
    for (int col = 0; col < transformation.cols(); ++col)
    {
        for (int row = 0; row < transformation.rows(); ++row)
        {
            if (isnan(transformation(row, col)))
            {
                ret_val = false;
                break;
            }
        }
    }
    return ret_val;
}

void omip::Location2PointPCL(const Feature::Location &point_location,pcl::PointXYZ& point_pcl)
{
    point_pcl.x = boost::tuples::get<0>(point_location);
    point_pcl.y = boost::tuples::get<1>(point_location);
    point_pcl.z = boost::tuples::get<2>(point_location);
}

void omip::LocationAndId2FeaturePCL(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCL &feature_pcl)
{
    feature_pcl.x = boost::tuples::get<0>(feature_location);
    feature_pcl.y = boost::tuples::get<1>(feature_location);
    feature_pcl.z = boost::tuples::get<2>(feature_location);
    feature_pcl.label = feature_id;
}

void omip::LocationAndId2FeaturePCLwc(const Feature::Location &feature_location, const Feature::Id &feature_id, omip::FeaturePCLwc &feature_pcl)
{
    feature_pcl.x = boost::tuples::get<0>(feature_location);
    feature_pcl.y = boost::tuples::get<1>(feature_location);
    feature_pcl.z = boost::tuples::get<2>(feature_location);
    feature_pcl.label = feature_id;
}

void omip::ROSTwist2EigenTwist(const geometry_msgs::Twist& ros_twist, Eigen::Twistd& eigen_twist)
{
    geometry_msgs::Vector3 linear = ros_twist.linear;
    geometry_msgs::Vector3 angular = ros_twist.angular;
    eigen_twist = Eigen::Twistd(angular.x, angular.y, angular.z,
                                linear.x, linear.y, linear.z);
}

//Normalize to [-180,180):
inline double constrainAngle(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle){
    return fmod(constrainAngle(angle),2*M_PI);
}
inline double angleDiff(double a,double b){
    double dif = fmod(b - a + M_PI,2*M_PI);
    if (dif < 0)
        dif += 2*M_PI;
    return dif - M_PI;
}
inline double unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle));
}

Eigen::Twistd omip::unwrapTwist(Eigen::Twistd& current_twist, Eigen::Displacementd& current_displacement, Eigen::Twistd& previous_twist, bool& changed)
{
    Eigen::Vector3d current_angular_component = Eigen::Vector3d(current_twist.rx(), current_twist.ry(), current_twist.rz());
    double current_rotation_angle = current_angular_component.norm();
    Eigen::Vector3d current_angular_direction = current_angular_component / current_rotation_angle;
    Eigen::Vector3d previous_angular_component = Eigen::Vector3d(previous_twist.rx(), previous_twist.ry(), previous_twist.rz());
    double previous_rotation_angle = previous_angular_component.norm();
    Eigen::Vector3d previous_angular_direction = previous_angular_component / previous_rotation_angle;

    // The difference should be around 2PI (a little bit less)
    if(Eigen::Vector3d(current_twist.rx()-previous_twist.rx(), current_twist.ry()-previous_twist.ry(), current_twist.rz()-previous_twist.rz()).norm() > M_PI
            || (current_angular_component.norm() > 0.5 && previous_angular_component.norm() > 0.5 && current_angular_direction.dot(previous_angular_direction) < -0.8))
    {

        Eigen::Vector3d new_angular_component;

        Eigen::Twistd unwrapped_twist;
        // Two cases:
        // 1) Jump from PI to -PI or vice versa
        // 1) Jump from 2*PI to 0 or from -2*PI to zero
        if(previous_rotation_angle + 0.3 < 2*M_PI)
        {
            // If previous and current are looking in opposite directions -> scalar product will be close to -1
            if(current_angular_direction.dot(previous_angular_direction) < 0)
            {
                new_angular_component= M_PI*previous_angular_direction -(M_PI - current_rotation_angle)*current_angular_direction;
            }// If both are looking in the same direction -> scalar product is close to +1
            else{
                new_angular_component= M_PI*previous_angular_direction +(M_PI - current_rotation_angle)*current_angular_direction;
            }
        }else{
            ROS_ERROR_STREAM("Numeric instability in the logarithm of a homogeneous transformation!");
            // If previous and current are looking in opposite directions -> scalar product will be close to -1
            if(current_angular_direction.dot(previous_angular_direction) < 0)
            {
                new_angular_component= std::min(2*M_PI, previous_rotation_angle)*previous_angular_direction +current_rotation_angle*current_angular_direction;
            }// If both are looking in the same direction -> scalar product is close to +1
            else{
                new_angular_component= std::min(2*M_PI, previous_rotation_angle)*previous_angular_direction +current_rotation_angle*current_angular_direction;
            }
        }

        double n2 = new_angular_component.squaredNorm();  // ||wtilde||^2
        double n = sqrt(n2);
        double sn = sin(n);
        double val = (double(2.0) * sn - n * (double(1.0) + cos(n))) / (double(2.0) *n2 * sn);

        Eigen::Vector3d RE3Element = Eigen::Vector3d(current_displacement.getR3Element()(0),current_displacement.getR3Element()(1),current_displacement.getR3Element()(2) );
        Eigen::Vector3d lin = -0.5*new_angular_component.cross(RE3Element);
        lin += (double(1.0) - val * n2 ) * RE3Element;
        lin += val * new_angular_component.dot(RE3Element) * new_angular_component;

        unwrapped_twist = Eigen::Twistd(new_angular_component.x(), new_angular_component.y(), new_angular_component.z(), lin.x(), lin.y(), lin.z());

        if(std::fabs(val) > 10)
        {
            ROS_ERROR_STREAM("Numeric instability in the logarithm of a homogeneous transformation. Val: " << val);
        }

        changed = true;
        return unwrapped_twist;
    }else{
        changed = false;
        return current_twist;
    }
}

Eigen::Twistd omip::invertTwist(Eigen::Twistd& current_twist, Eigen::Twistd& previous_twist, bool& inverted)
{
    Eigen::Vector3d current_angular_component = Eigen::Vector3d(current_twist.rx(), current_twist.ry(), current_twist.rz());
    double current_rotation_angle = current_angular_component.norm();
    Eigen::Vector3d current_angular_direction = current_angular_component / current_rotation_angle;
    Eigen::Vector3d previous_angular_component = Eigen::Vector3d(previous_twist.rx(), previous_twist.ry(), previous_twist.rz());
    double previous_rotation_angle = previous_angular_component.norm();
    Eigen::Vector3d previous_angular_direction = previous_angular_component / previous_rotation_angle;
    // The difference should be around 2PI (a little bit less)
    if(Eigen::Vector3d(current_twist.rx()-previous_twist.rx(), current_twist.ry()-previous_twist.ry(), current_twist.rz()-previous_twist.rz()).norm() > M_PI
            || current_angular_direction.dot(previous_angular_direction) < -0.8)
    {
        inverted = true;
        return -current_twist;
    }else{
        inverted = false;
        return current_twist;
    }
}

void omip::invert3x3Matrix(const MatrixWrapper::Matrix& to_inv, MatrixWrapper::Matrix& inverse)
{
    double determinant =    +to_inv(1,1)*(to_inv(2,2)*to_inv(3,3)-to_inv(3,2)*to_inv(2,3))
            -to_inv(1,2)*(to_inv(2,1)*to_inv(3,3)-to_inv(2,3)*to_inv(3,1))
            +to_inv(1,3)*(to_inv(2,1)*to_inv(3,2)-to_inv(2,2)*to_inv(3,1));
    double invdet = 1.0/determinant;
    inverse(1,1) =  (to_inv(2,2)*to_inv(3,3)-to_inv(3,2)*to_inv(2,3))*invdet;
    inverse(2,1) = -(to_inv(1,2)*to_inv(3,3)-to_inv(1,3)*to_inv(3,2))*invdet;
    inverse(3,1) =  (to_inv(1,2)*to_inv(2,3)-to_inv(1,3)*to_inv(2,2))*invdet;
    inverse(1,2) = -(to_inv(2,1)*to_inv(3,3)-to_inv(2,3)*to_inv(3,1))*invdet;
    inverse(2,2) =  (to_inv(1,1)*to_inv(3,3)-to_inv(1,3)*to_inv(3,1))*invdet;
    inverse(3,2) = -(to_inv(1,1)*to_inv(2,3)-to_inv(2,1)*to_inv(1,3))*invdet;
    inverse(1,3) =  (to_inv(2,1)*to_inv(3,2)-to_inv(3,1)*to_inv(2,2))*invdet;
    inverse(2,3) = -(to_inv(1,1)*to_inv(3,2)-to_inv(3,1)*to_inv(1,2))*invdet;
    inverse(3,3) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
}

void omip::invert3x3MatrixEigen(const Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& to_inv,
                                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >& inverse)
{
    double determinant =    +to_inv(0,0)*(to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))
            -to_inv(0,1)*(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))
            +to_inv(0,2)*(to_inv(1,0)*to_inv(2,1)-to_inv(1,1)*to_inv(2,0));
    double invdet = 1.0/determinant;
    inverse(0,0) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
    inverse(1,0) = -(to_inv(0,1)*to_inv(2,2)-to_inv(0,2)*to_inv(2,1))*invdet;
    inverse(2,0) =  (to_inv(0,1)*to_inv(1,2)-to_inv(0,2)*to_inv(1,1))*invdet;
    inverse(0,1) = -(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))*invdet;
    inverse(1,1) =  (to_inv(0,0)*to_inv(2,2)-to_inv(0,2)*to_inv(2,0))*invdet;
    inverse(2,1) = -(to_inv(0,0)*to_inv(1,2)-to_inv(1,0)*to_inv(0,2))*invdet;
    inverse(0,2) =  (to_inv(1,0)*to_inv(2,1)-to_inv(2,0)*to_inv(1,1))*invdet;
    inverse(1,2) = -(to_inv(0,0)*to_inv(2,1)-to_inv(2,0)*to_inv(0,1))*invdet;
    inverse(2,2) =  (to_inv(0,0)*to_inv(1,1)-to_inv(1,0)*to_inv(0,1))*invdet;
}

void omip::invert3x3MatrixEigen2(const Eigen::Matrix3d& to_inv,
                                 Eigen::Matrix3d& inverse)
{
    double determinant =    +to_inv(0,0)*(to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))
            -to_inv(0,1)*(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))
            +to_inv(0,2)*(to_inv(1,0)*to_inv(2,1)-to_inv(1,1)*to_inv(2,0));
    double invdet = 1.0/determinant;
    inverse(0,0) =  (to_inv(1,1)*to_inv(2,2)-to_inv(2,1)*to_inv(1,2))*invdet;
    inverse(1,0) = -(to_inv(0,1)*to_inv(2,2)-to_inv(0,2)*to_inv(2,1))*invdet;
    inverse(2,0) =  (to_inv(0,1)*to_inv(1,2)-to_inv(0,2)*to_inv(1,1))*invdet;
    inverse(0,1) = -(to_inv(1,0)*to_inv(2,2)-to_inv(1,2)*to_inv(2,0))*invdet;
    inverse(1,1) =  (to_inv(0,0)*to_inv(2,2)-to_inv(0,2)*to_inv(2,0))*invdet;
    inverse(2,1) = -(to_inv(0,0)*to_inv(1,2)-to_inv(1,0)*to_inv(0,2))*invdet;
    inverse(0,2) =  (to_inv(1,0)*to_inv(2,1)-to_inv(2,0)*to_inv(1,1))*invdet;
    inverse(1,2) = -(to_inv(0,0)*to_inv(2,1)-to_inv(2,0)*to_inv(0,1))*invdet;
    inverse(2,2) =  (to_inv(0,0)*to_inv(1,1)-to_inv(1,0)*to_inv(0,1))*invdet;
}

void omip::computeAdjoint(const Eigen::Twistd &pose_ec, Eigen::Matrix<double, 6, 6> &adjoint_out)
{
    omip::computeAdjoint(pose_ec.exp(1e-12), adjoint_out);
}

void omip::adjointXcovXadjointT(const Eigen::Twistd& pose_ec, const Eigen::Matrix<double, 6, 6>& cov, Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    Eigen::Matrix<double, 6, 6> adjoint;
    omip::computeAdjoint(pose_ec, adjoint);

    transformed_cov_out = adjoint*cov*adjoint.transpose();
}

//LGSM adjoint:
// (   R    0 )
// ((t x R) R )
//Desired adjoint:
// ( R (t x R))
// ( 0    R   )
void omip::computeAdjoint(Eigen::Displacementd pose_disp, Eigen::Matrix<double, 6, 6> &adjoint_out)
{
    Eigen::Matrix<double, 6, 6> adjoint_rot_trans = pose_disp.adjoint();

    adjoint_out = adjoint_rot_trans;
    adjoint_out.block<3,3>(0,3) = adjoint_rot_trans.block<3,3>(3,0);
    adjoint_out.block<3,3>(3,0) = adjoint_rot_trans.block<3,3>(0,3);
}

void omip::adjointXcovXadjointT(const Eigen::Displacementd& pose_disp,
                                const Eigen::Matrix<double, 6, 6>& cov,
                                Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    Eigen::Matrix<double, 6, 6> adjoint;
    omip::computeAdjoint(pose_disp, adjoint);

    transformed_cov_out = adjoint*cov*adjoint.transpose();
}

void omip::adjointXinvAdjointXcovXinvAdjointTXadjointT(const Eigen::Displacementd& pose_disp1,
                                                       const Eigen::Displacementd& pose_disp2,
                                                       const Eigen::Matrix<double, 6, 6>& cov,
                                                       Eigen::Matrix<double, 6, 6>& transformed_cov_out)
{
    //T_rrbf_srbf.adjoint()*T_rrbf_srbf_t0.inverse().adjoint()*_srb_initial_pose_cov_in_rrbf*T_rrbf_srbf_t0.inverse().adjoint().transpose()*T_rrbf_srbf.adjoint().transpose()
    Eigen::Matrix<double, 6, 6> adjoint1;
    omip::computeAdjoint(pose_disp1, adjoint1);

    Eigen::Matrix<double, 6, 6> adjoint2;
    omip::computeAdjoint(pose_disp2.inverse(), adjoint2);

    transformed_cov_out = adjoint1*adjoint2*cov*adjoint2.transpose()*adjoint1.transpose();
}

void omip::findIntersectionOfEllipsoidAndPlane(const Eigen::Matrix3d& ellipsoid,
                                               const Eigen::Vector3d& normal_plane,
                                               double& r1,
                                               double& r2,
                                               Eigen::Matrix3d& full_rotation)
{    
    double confidence_value = 0.5;
    boost::math::chi_squared chi_sq_dist(2);
    double critical_value = boost::math::quantile(chi_sq_dist, confidence_value);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(ellipsoid);

    double A1 = 2*eigensolver.eigenvalues()[2]*std::sqrt(critical_value);
    double a1 = 1.0/pow(A1, 2);
    double A2 = 2*eigensolver.eigenvalues()[1]*std::sqrt(critical_value);
    double a2 = 1.0/pow(A2, 2);
    double A3 = 2*eigensolver.eigenvalues()[0]*std::sqrt(critical_value);
    double a3 = 1.0/pow(A3, 2);

    if(A1 == A2 && A2 == A3)
    {
        r1 = A1;
        r2 = A1;
        Eigen::Vector3d major_axis_cart = normal_plane.cross(Eigen::Vector3d::UnitX());
        Eigen::Vector3d minor_axis_cart = major_axis_cart.cross(normal_plane);
        full_rotation << major_axis_cart.x(), minor_axis_cart.x(), normal_plane.x(),
                major_axis_cart.y(), minor_axis_cart.y(), normal_plane.y(),
                major_axis_cart.z(), minor_axis_cart.z(), normal_plane.z();
    }else{

        Eigen::Vector3d x1_axis = eigensolver.eigenvectors().col(2);
        Eigen::Vector3d x2_axis = eigensolver.eigenvectors().col(1);
        Eigen::Vector3d x3_axis = eigensolver.eigenvectors().col(0);

        Eigen::Matrix3d R;
        R << x1_axis[0] , x2_axis[0] , x3_axis[0] ,
                x1_axis[1] , x2_axis[1] , x3_axis[1] ,
                x1_axis[2] , x2_axis[2] , x3_axis[2] ;

        Eigen::Vector3d beta = normal_plane.transpose()*R;

        double coeff_a = (a2*a3*pow(beta[0],2) + a1*a3*pow(beta[1],2)) + a1*a2*pow(beta[2],2);
        double coeff_b =  -((a2+a3)*pow(beta[0],2) + (a1+a3)*pow(beta[1],2) + (a1+a2)*pow(beta[2],2));

        double ra = (-coeff_b + sqrt(pow(coeff_b,2) -4*coeff_a))/(2.0*coeff_a);
        double rb = (-coeff_b - sqrt(pow(coeff_b,2) -4*coeff_a))/(2.0*coeff_a);

        r1 = ra > rb ? ra : rb;
        r1 = sqrt(r1);
        r2 = ra < rb ? ra : rb;
        r2 = sqrt(r2);

        double k11 = beta[0]/(a1*pow(r1,2)-1);
        double k12 = beta[1]/(a2*pow(r1,2)-1);
        double k13 = beta[2]/(a3*pow(r1,2)-1);
        double sumk1 = k11*k11+k12*k12+k13*k13;

        double k21 = beta[0]/(a1*pow(r2,2)-1);
        double k22 = beta[1]/(a2*pow(r2,2)-1);
        double k23 = beta[2]/(a3*pow(r2,2)-1);
        double sumk2 = k21*k21+k22*k22+k23*k23;

        Eigen::Vector3d major_axis = 1/sqrt(sumk1)*Eigen::Vector3d(k11, k12, k13);

        Eigen::Vector3d minor_axis = 1/sqrt(sumk2)*Eigen::Vector3d(k21, k22, k23);

        Eigen::Vector3d major_axis_cart = major_axis.transpose()*(R.transpose());
        Eigen::Vector3d minor_axis_cart = minor_axis.transpose()*(R.transpose());

        //The 3 axis are: z -> _joint_orientation
        //x-> major_axis_cart
        //y-> minor_axis_cart
        //Create a rotation matrix with them and query the quaternion

        //Impose that the xyz axis are correctly aligned
        // x cross y dot z should be 1
        // if it is -1, we flip y
        if(major_axis_cart.cross(minor_axis_cart).dot(normal_plane) < -0.9)
        {
            minor_axis_cart = -minor_axis_cart;
        }

        full_rotation << major_axis_cart.x(), minor_axis_cart.x(), normal_plane.x(),
                major_axis_cart.y(), minor_axis_cart.y(), normal_plane.y(),
                major_axis_cart.z(), minor_axis_cart.z(), normal_plane.z();
    }
}
