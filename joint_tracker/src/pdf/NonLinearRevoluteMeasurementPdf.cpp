#include "joint_tracker/pdf/NonLinearRevoluteMeasurementPdf.h"

#include <Eigen/Geometry>
#include <lgsm/Lgsm>
#include <ros/ros.h>

#include "omip_common/OMIPUtils.h"


using namespace BFL;

#define REV_PDF_DIM 6
#define REV_CONDITIONAL_VAR_DIM 7
#define REV_NUM_CONDITIONAL_ARGS 2

NonLinearRevoluteMeasurementPdf::NonLinearRevoluteMeasurementPdf(const Gaussian& additiveNoise) :
    AnalyticConditionalGaussianAdditiveNoise(additiveNoise,REV_NUM_CONDITIONAL_ARGS),
    dfx(REV_PDF_DIM, REV_CONDITIONAL_VAR_DIM)
{

}

NonLinearRevoluteMeasurementPdf::~NonLinearRevoluteMeasurementPdf()
{

}

MatrixWrapper::ColumnVector NonLinearRevoluteMeasurementPdf::ExpectedValueGet() const
{
    ColumnVector state = ConditionalArgumentGet(0);

    double phi = state(1);
    double theta = state(2);
    double sp = sin(phi);
    double cp = cos(phi);
    double st = sin(theta);
    double ct = cos(theta);

    double px = state(3);
    double py = state(4);
    double pz = state(5);

    Eigen::Vector3d rev_joint_orientation = Eigen::Vector3d(cp * st, sp * st, ct);
    Eigen::Vector3d rev_joint_position = Eigen::Vector3d(px, py, pz);

    double rv = state(6);

    Eigen::Vector3d joint_translation = (-rv * rev_joint_orientation).cross(rev_joint_position);

    Eigen::Twistd joint_pose = Eigen::Twistd(rv * rev_joint_orientation.x(),
                                             rv * rev_joint_orientation.y(),
                                             rv * rev_joint_orientation.z(),
                                             joint_translation.x(),
                                             joint_translation.y(),
                                             joint_translation.z());

    ColumnVector expected_pose(REV_PDF_DIM);
    expected_pose(1) = joint_pose.vx();
    expected_pose(2) = joint_pose.vy();
    expected_pose(3) = joint_pose.vz();
    expected_pose(4) = joint_pose.rx();
    expected_pose(5) = joint_pose.ry();
    expected_pose(6) = joint_pose.rz();

    return expected_pose + this->AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix NonLinearRevoluteMeasurementPdf::dfGet(unsigned int i) const
{
    if (i == 0) //derivative to the first conditional argument (x)
    {
        double phi = ConditionalArgumentGet(0)(1);
        double theta = ConditionalArgumentGet(0)(2);
        double sp = sin(phi);
        double cp = cos(phi);
        double st = sin(theta);
        double ct = cos(theta);

        double px = ConditionalArgumentGet(0)(3);
        double py = ConditionalArgumentGet(0)(4);
        double pz = ConditionalArgumentGet(0)(5);
        double rv = ConditionalArgumentGet(0)(6);

        dfx = 0;

        dfx(1, 1) = -rv * cp * st * pz;
        dfx(2, 1) = -rv * sp * st * pz;
        dfx(3, 1) = rv * st * (cp * px + sp * py);
        dfx(4, 1) = -rv * sp * st;
        dfx(5, 1) = rv * cp * st;

        dfx(1, 2) = -rv * (st * py + sp * ct * pz);
        dfx(2, 2) = rv * (cp * ct * pz + st * px);
        dfx(3, 2) = rv * ct * (sp * px - cp * py);
        dfx(4, 2) = rv * cp * ct;
        dfx(5, 2) = rv * sp * ct;
        dfx(6, 2) = -rv * st;

        dfx(2, 3) = -rv * ct;
        dfx(3, 3) = rv * sp * st;

        dfx(1, 4) = rv * ct;
        dfx(3, 4) = -rv * cp * st;

        dfx(1, 5) = -rv * sp * st;
        dfx(2, 5) = rv * cp * st;

        dfx(1, 6) = ct * py - sp * st * pz;
        dfx(2, 6) = cp * st * pz - ct * px;
        dfx(3, 6) = st * (sp * px - cp * py);
        dfx(4, 6) = cp * st;
        dfx(5, 6) = sp * st;
        dfx(6, 6) = ct;

        dfx(1, 7) = 0;
        dfx(2, 7) = 0;
        dfx(3, 7) = 0;
        dfx(4, 7) = 0;
        dfx(5, 7) = 0;
        dfx(6, 7) = 0;
        return dfx;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED( "NonLinearRevoluteMeasurementPdf::dfGet",
                    "The derivative is not implemented for the " << i << "th conditional argument");
        exit(BFL_ERRMISUSE);
    }
}

