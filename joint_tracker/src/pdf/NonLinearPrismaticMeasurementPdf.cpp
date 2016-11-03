#include "joint_tracker/pdf/NonLinearPrismaticMeasurementPdf.h"

#include <ros/ros.h>

using namespace BFL;

#define PRISM_PDF_DIM 6
#define PRISM_CONDITIONAL_VAR_DIM 4
#define PRISM_NUM_CONDITIONAL_ARGS 2

NonLinearPrismaticMeasurementPdf::NonLinearPrismaticMeasurementPdf(const Gaussian& additiveNoise) :
    AnalyticConditionalGaussianAdditiveNoise(additiveNoise, PRISM_NUM_CONDITIONAL_ARGS),
    dfx(PRISM_PDF_DIM, PRISM_CONDITIONAL_VAR_DIM)
{

}

NonLinearPrismaticMeasurementPdf::~NonLinearPrismaticMeasurementPdf()
{

}

MatrixWrapper::ColumnVector NonLinearPrismaticMeasurementPdf::ExpectedValueGet() const
{
    ColumnVector state = ConditionalArgumentGet(0);

    double phi = state(1);
    double theta = state(2);
    double pv = state(3);

    ColumnVector expected_pose(PRISM_PDF_DIM);
    expected_pose(1) = pv * cos(phi) * sin(theta);
    expected_pose(2) = pv * sin(phi) * sin(theta);
    expected_pose(3) = pv * cos(theta);
    expected_pose(4) = 0.;
    expected_pose(5) = 0.;
    expected_pose(6) = 0.;

    return expected_pose + this->AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix NonLinearPrismaticMeasurementPdf::dfGet(unsigned int i) const
{
    if (i == 0) //derivative to the first conditional argument (x)
    {
        double phi = ConditionalArgumentGet(0)(1);
        double theta = ConditionalArgumentGet(0)(2);
        double pv = ConditionalArgumentGet(0)(3);

        dfx = 0;
        dfx(1, 1) = -pv*sin(theta)*sin(phi);
        dfx(2, 1) = pv*sin(theta)*cos(phi);
        dfx(1, 2) = pv*cos(theta)*cos(phi);
        dfx(2, 2) = pv*cos(theta)*sin(phi);
        dfx(3, 2) = -pv*sin(theta);
        dfx(1, 3) = sin(theta)*cos(phi);
        dfx(2, 3) = sin(theta)*sin(phi);
        dfx(3, 3) = cos(theta);
        return dfx;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("NonLinearPrismaticMeasurementPdf::dfGet",
                               "The derivative is not implemented for the " << i << "th conditional argument");
        exit(BFL_ERRMISUSE);
    }
}

