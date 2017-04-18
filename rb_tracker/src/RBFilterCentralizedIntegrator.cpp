#include "rb_tracker/RBFilterCentralizedIntegrator.h"

#include "omip_common/OMIPUtils.h"

using namespace omip;

RBFilterCentralizedIntegrator::RBFilterCentralizedIntegrator()
    :RecursiveEstimatorCentralizedBeliefIntegrator(1)
{

}

RBFilterCentralizedIntegrator::RBFilterCentralizedIntegrator(double loop_period_ns,
         const Eigen::Matrix4d initial_pose,
         const Eigen::Twistd& initial_velocity):
    RBFilterCentralizedIntegrator()
{

}

RBFilterCentralizedIntegrator::~RBFilterCentralizedIntegrator()
{

}

void RBFilterCentralizedIntegrator::addRBFilter(const RBFilter::Ptr& filter_to_integrate)
{
    _filters_to_integrate.push_back(filter_to_integrate);
}

void RBFilterCentralizedIntegrator::integrateBeliefs()
{
    // We have right now only the case of merging 2 estimates
    if(_filters_to_integrate.size() == 2)
    {
        // Compute the MAP of two pose estimates (http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1238637 Computing MAP trajectories by representing, propagating
        //and combining PDFs over groups)
        Eigen::Matrix<double, 6, 6> inv_sum_of_covs = (_filters_to_integrate[0]->getPoseCovariance() + _filters_to_integrate[1]->getPoseCovariance()).inverse();
        Eigen::Matrix4d delta_ht = _filters_to_integrate[1]->getPose()*(_filters_to_integrate[0]->getPose().inverse());
        Eigen::Twistd delta_ec;
        TransformMatrix2Twist(delta_ht, delta_ec);
        Eigen::Matrix<double, 6, 1> delta_vec;
        delta_vec << delta_ec.vx(), delta_ec.vy(), delta_ec.vz(),
                delta_ec.rx(), delta_ec.ry(), delta_ec.rz();

        //Equation 13
        Eigen::Matrix<double, 6, 1> epsilon_1_vec = _filters_to_integrate[0]->getPoseCovariance()*inv_sum_of_covs*delta_vec;

        Eigen::Twistd epsilon_1_ec(epsilon_1_vec[3], epsilon_1_vec[4], epsilon_1_vec[5], epsilon_1_vec[0], epsilon_1_vec[1], epsilon_1_vec[2]);
        Eigen::Matrix4d epsilon_1_ht;
        Twist2TransformMatrix(epsilon_1_ec, epsilon_1_ht);

        //Equation 9
        Eigen::Matrix4d integrated_pose = epsilon_1_ht*_filters_to_integrate[0]->getPose();

        //Equation 19
        Eigen::Matrix<double, 6, 6> integrated_pose_cov = _filters_to_integrate[0]->getPoseCovariance()*inv_sum_of_covs*_filters_to_integrate[1]->getPoseCovariance();

        // Then propagate the new MAP back to the distributed filters
        _filters_to_integrate[0]->setPose(integrated_pose);
        _filters_to_integrate[1]->setPose(integrated_pose);

        _filters_to_integrate[0]->setPoseCovariance(integrated_pose_cov);
        _filters_to_integrate[1]->setPoseCovariance(integrated_pose_cov);
    }
}
