#include "joint_tracker/pdf/NonLinearGraspMeasurementPdf.h"

#include <Eigen/Geometry>
#include <lgsm/Lgsm>
#include <ros/ros.h>

using namespace BFL;

#define JACOBIAN_PDF_DIM 6
#define JACOBIAN_CONDITIONAL_VAR_DIM 36
#define JACOBIAN_NUM_CONDITIONAL_ARGS 2

NonLinearGraspMeasurementPdf::NonLinearGraspMeasurementPdf(const Gaussian& additiveNoise) :
    AnalyticConditionalGaussianAdditiveNoise(additiveNoise,JACOBIAN_NUM_CONDITIONAL_ARGS),
    dfx(JACOBIAN_PDF_DIM, JACOBIAN_CONDITIONAL_VAR_DIM)
{
}

NonLinearGraspMeasurementPdf::~NonLinearGraspMeasurementPdf()
{

}

MatrixWrapper::ColumnVector NonLinearGraspMeasurementPdf::ExpectedValueGet() const
{
    // Here we estimate the delta motion of the SRB (wrt cf) given the delta motion of the RRB (wrt cf)

    // This is the Jacobian Matrix
    ColumnVector jacobian = ConditionalArgumentGet(0);
    Eigen::Matrix<double, 6, 6> jacobian_ee_to_interacted_rb_wrt_cf = Eigen::Matrix<double, 6, 6>::Identity();
    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            jacobian_ee_to_interacted_rb_wrt_cf(i,j) = jacobian(6*i + j + 1);
        }
    }

    // This is the motion of the end effector (it can be past motion or future expected motion)
    ColumnVector delta_eef_wrt_cf_cv = ConditionalArgumentGet(1);

    Eigen::Matrix<double, 6, 1> delta_rrb_wrt_cf;
    delta_rrb_wrt_cf << delta_eef_wrt_cf_cv(1), delta_eef_wrt_cf_cv(2), delta_eef_wrt_cf_cv(3),
            delta_eef_wrt_cf_cv(4), delta_eef_wrt_cf_cv(5), delta_eef_wrt_cf_cv(6);

    Eigen::Matrix<double, 6, 1> delta_srb_pose_wrt_cf = jacobian_ee_to_interacted_rb_wrt_cf*delta_rrb_wrt_cf;;

    ColumnVector expected_delta_srb_pose_wrt_cf(JACOBIAN_PDF_DIM);
    expected_delta_srb_pose_wrt_cf(1) = delta_srb_pose_wrt_cf(0);
    expected_delta_srb_pose_wrt_cf(2) = delta_srb_pose_wrt_cf(1);
    expected_delta_srb_pose_wrt_cf(3) = delta_srb_pose_wrt_cf(2);
    expected_delta_srb_pose_wrt_cf(4) = delta_srb_pose_wrt_cf(3);
    expected_delta_srb_pose_wrt_cf(5) = delta_srb_pose_wrt_cf(4);
    expected_delta_srb_pose_wrt_cf(6) = delta_srb_pose_wrt_cf(5);

    return expected_delta_srb_pose_wrt_cf + this->AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix NonLinearGraspMeasurementPdf::dfGet(unsigned int i) const
{
    if (i == 0) //derivative wrt the first conditional argument (x)
    {       
        dfx = 0;

        for(int i=0; i<6; i++)
        {
            for(int j=0; j<6; j++)
            {
                dfx(i+1, i*6 + j + 1) = 1;
            }
        }

        return dfx;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED( "NonLinearGraspMeasurementPdf::dfGet",
                    "The derivative is not implemented for the " << i << "th conditional argument");
        exit(BFL_ERRMISUSE);
    }
}

