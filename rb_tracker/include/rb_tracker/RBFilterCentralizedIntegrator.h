/*
 * RBFilterCentralizedIntegrator.h
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

#ifndef RB_FILTER_CENTRALIZED_INTEGRATOR_H_
#define RB_FILTER_CENTRALIZED_INTEGRATOR_H_

#include "omip_common/OMIPTypeDefs.h"
#include "omip_common/RecursiveEstimatorCentralizedBeliefIntegrator.h"

#include "rb_tracker/RBFilter.h"

namespace omip
{

/**
 * Class RBFilterCentralizedIntegrator
 * Integrates beliefs from different sensor sources about the motion of a rigid body
 */
class RBFilterCentralizedIntegrator : public RecursiveEstimatorCentralizedBeliefIntegrator
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Shared Pointer to a RBFilterCentralizedIntegrator
    typedef boost::shared_ptr<RBFilterCentralizedIntegrator> Ptr;

    /**
   * Default constructor
   */
    RBFilterCentralizedIntegrator();

    /**
   * Constructor
   */
    RBFilterCentralizedIntegrator(double loop_period_ns,
             const Eigen::Matrix4d initial_pose,
             const Eigen::Twistd& initial_velocity);

    /**
   * Destructor
   */
    virtual ~RBFilterCentralizedIntegrator();

    virtual void addRBFilter(const RBFilter::Ptr& filter_to_integrate);

    virtual void integrateBeliefs();

protected:

    std::vector<RBFilter::Ptr> _filters_to_integrate;

};

}

#endif /* RB_FILTER_CENTRALIZED_INTEGRATOR_H_ */

