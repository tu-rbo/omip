#include "joint_tracker/pdf/NonLinearPrismaticMeasurementPdf.h"

#include <ros/ros.h>

using namespace BFL;

#define PRISM_PDF_DIM 6
#define PRISM_CONDITIONAL_VAR_DIM 5
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

    double x = state(1);
    double y = state(2);
    double z = state(3);
    double pv = state(4);

    ColumnVector expected_pose(PRISM_PDF_DIM);
    expected_pose(1) = pv * x;
    expected_pose(2) = pv * y;
    expected_pose(3) = pv * z;
    expected_pose(4) = 0.;
    expected_pose(5) = 0.;
    expected_pose(6) = 0.;

    return expected_pose + this->AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix NonLinearPrismaticMeasurementPdf::dfGet(unsigned int i) const
{
    if (i == 0) //derivative to the first conditional argument (x)
    {
        double x = ConditionalArgumentGet(0)(1);
        double y = ConditionalArgumentGet(0)(2);
        double z = ConditionalArgumentGet(0)(3);
        double pv = ConditionalArgumentGet(0)(4);

        dfx = 0;
        dfx(1, 1) = pv; //dh/d(x)
        dfx(1, 2) = 0; //d/d(y)
        dfx(1, 3) = 0; //d/d(z)
        dfx(1, 4) = x; //d/d(pv)

        dfx(2, 1) = 0; //dh/d(x)
        dfx(2, 2) = pv; //d/d(y)
        dfx(2, 3) = 0; //d/d(z)
        dfx(2, 4) = y; //d/d(pv)

        dfx(3, 1) = 0; //dh/d(x)
        dfx(3, 2) = 0; //d/d(y)
        dfx(3, 3) = pv; //d/d(z)
        dfx(3, 4) = z; //d/d(pv)

        return dfx;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("NonLinearPrismaticMeasurementPdf::dfGet",
                               "The derivative is not implemented for the " << i << "th conditional argument");
        exit(BFL_ERRMISUSE);
    }
}

