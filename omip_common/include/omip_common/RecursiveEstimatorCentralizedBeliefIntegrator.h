#ifndef RecursiveEstimatorCentralizedBeliefIntegrator_H_
#define RecursiveEstimatorCentralizedBeliefIntegrator_H_

#include <vector>
#include <string>
#include <iostream>

#include <std_msgs/Bool.h>
#include <ros/node_handle.h>

#include <Eigen/Core>

//Covariance intersection!!!!!!!!
// Assuming cross-correlation zero

namespace omip
{

class RecursiveEstimatorCentralizedBeliefIntegrator
{
public:

    typedef boost::shared_ptr<RecursiveEstimatorCentralizedBeliefIntegrator> Ptr;
    typedef boost::shared_ptr<const RecursiveEstimatorCentralizedBeliefIntegrator> ConstPtr;

    RecursiveEstimatorCentralizedBeliefIntegrator(int state_dimension)
    {
        _state_dimension = state_dimension;
        _integrated_belief_mu = Eigen::MatrixXd::Zero(_state_dimension,1);
        _integrated_belief_sigma = Eigen::MatrixXd::Identity(_state_dimension, _state_dimension);
        _integrated_belief_sigma *= 1e2;
    }

    virtual ~RecursiveEstimatorCentralizedBeliefIntegrator()
    {

    }

    virtual void init(const Eigen::MatrixXd& initial_value_mu, const Eigen::MatrixXd& initial_value_sigma)
    {
        _integrated_belief_mu = initial_value_mu;
        _integrated_belief_sigma = initial_value_sigma;

        std::cout <<"Initial integrated state: " << initial_value_mu.transpose() << std::endl;
    }

    virtual Eigen::MatrixXd getIntegratedBeliefMu() const
    {
        return this->_integrated_belief_mu;
    }

    virtual Eigen::MatrixXd getIntegratedBeliefSigma() const
    {
        return this->_integrated_belief_sigma;
    }

    virtual void addBelief(const Eigen::MatrixXd& state_mu, const Eigen::MatrixXd& state_sigma, int channel)
    {
        _last_beliefs[channel] = std::pair<Eigen::MatrixXd, Eigen::MatrixXd>(state_mu, state_sigma);
    }

    virtual void integrateBeliefs()
    {
        // integrated belief = IntegratedSigma * SUM( IndividualSigma-1 * IndividualBelief )
        // integrated sigma = ( SUM( IndividualSigma-1 ) )-1
        std::map<int, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> >::iterator beliefs_it = _last_beliefs.begin();
        std::map<int, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> >::iterator beliefs_end = _last_beliefs.end();

        Eigen::MatrixXd sum_inverse_belief_sigmas = Eigen::MatrixXd::Zero(_state_dimension, _state_dimension);
        Eigen::MatrixXd sum_inverse_belief_sigmas_times_belief_mus = Eigen::MatrixXd::Zero(_state_dimension, 1);
        int idx = 0;
        for(;beliefs_it != beliefs_end; ++beliefs_it)
        {
            Eigen::MatrixXd belief_sigma_prime = beliefs_it->second.second/_channel_weights[beliefs_it->first];
            sum_inverse_belief_sigmas += belief_sigma_prime.inverse();
            sum_inverse_belief_sigmas_times_belief_mus +=  belief_sigma_prime.inverse()*beliefs_it->second.first;

            //std::cout << "Channel " << idx << std::endl;
            //std::cout << "Belief " << std::endl << beliefs_it->second.first.transpose() << std::endl;
            //std::cout << "Covariance " << std::endl << belief_sigma_prime << std::endl;
        }

        _integrated_belief_sigma = sum_inverse_belief_sigmas.inverse();
        _integrated_belief_mu = _integrated_belief_sigma*sum_inverse_belief_sigmas_times_belief_mus;

        _last_beliefs.clear();



//        _last_beliefs[channel] = std::pair<Eigen::MatrixXd, Eigen::MatrixXd>(state_mu, state_sigma);

//        Eigen::MatrixXd state_sigma_prime = state_sigma/_channel_weights[channel];

//        _integrated_belief_sigma += 100*Eigen::MatrixXd::Identity(_integrated_belief_sigma.rows(), _integrated_belief_sigma.cols());

//        // This is like a simple kalman filter with state transition and measurement models the identity
//        Eigen::MatrixXd innovation = state_mu - _integrated_belief_mu;
//        Eigen::MatrixXd kalman_gain = _integrated_belief_sigma*((_integrated_belief_sigma + state_sigma_prime).inverse());
//        _integrated_belief_mu += innovation*kalman_gain;
//        _integrated_belief_sigma = (Eigen::MatrixXd::Identity(_integrated_belief_mu.cols(), _integrated_belief_mu.cols()) - kalman_gain)*_integrated_belief_sigma;

        std::cout <<"New integrated state: " << _integrated_belief_mu.transpose() << std::endl;
    }

    virtual void setChannelSwitch(const bool& on, int channel)
    {
        _channel_weights[channel] = _channel_weights[channel]*(double)on;
    }

    virtual void setChannelWeight(const double& weight, int channel)
    {
        _channel_weights[channel] = weight;
    }

protected:

    int _state_dimension;
    Eigen::MatrixXd _integrated_belief_mu;
    Eigen::MatrixXd _integrated_belief_sigma;
    std::map<int, double> _channel_weights;
    std::map<int, std::pair<Eigen::MatrixXd, Eigen::MatrixXd> > _last_beliefs;
};

}

#endif
