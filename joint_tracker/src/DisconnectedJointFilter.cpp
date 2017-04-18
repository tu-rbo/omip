#include "joint_tracker/DisconnectedJointFilter.h"

#include <boost/random.hpp>

using namespace omip;

DisconnectedJointFilter::DisconnectedJointFilter() :
    JointFilter(),
    _is_grasp_model(false)
{
    _unnormalized_model_probability = 0.8;

    std::srand(std::time(0));
}

DisconnectedJointFilter::~DisconnectedJointFilter()
{

}

DisconnectedJointFilter::DisconnectedJointFilter(const DisconnectedJointFilter &disconnected_joint) :
    JointFilter(disconnected_joint)
{
}

void DisconnectedJointFilter::initialize()
{

}

double DisconnectedJointFilter::getProbabilityOfJointFilter() const
{
    return (_unnormalized_model_probability / this->_normalizing_term);
}

void DisconnectedJointFilter::estimateUnnormalizedModelProbability()
{    
    // If we receive ft sensor signals we use them to discriminate between grasp failure and any other model
    if(_ft_meas.size() == 6)
    {
        this->_estimateFTMeasurementLikelihood();
    }

    // If it is not a grasp model (articulated object model) or if it is a grasp model and it is not grasped
    if(!_is_grasp_model)
    {
        this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood;
    }else{

        this->_unnormalized_model_probability = _model_prior_probability*_measurements_likelihood*_current_prob_grasp_failure;
    }
}

void DisconnectedJointFilter::predictMeasurement()
{
    // We just give a random prediction


    typedef boost::mt19937 RNGType;
    RNGType rng;
    boost::uniform_real<> minus_plus_hundred( -100, 100 );
    boost::variate_generator< RNGType, boost::uniform_real<> > dice(rng, minus_plus_hundred);

    this->_change_in_relative_pose_predicted_in_rrbf = Eigen::Twistd(dice(),dice(),dice(),
                                                      dice(),dice(),dice());
    Eigen::Displacementd predicted_delta = this->_change_in_relative_pose_predicted_in_rrbf.exp(1e-20);
    Eigen::Displacementd T_rrbf_srbf_t0 = this->_srb_initial_pose_in_rrbf.exp(1.0e-20);
    Eigen::Displacementd T_rrbf_srbf_t_next = predicted_delta * T_rrbf_srbf_t0;

    this->_srb_predicted_pose_in_rrbf = T_rrbf_srbf_t_next.log(1.0e-20);

    for(int i=0; i<6; i++)
    {
        for(int j=0; j<6; j++)
        {
            if(i == j)
            {
                this->_change_in_relative_pose_cov_predicted_in_rrbf(i,j) = 1.0e6;
            }else
            {
                this->_change_in_relative_pose_cov_predicted_in_rrbf(i,j) = 1.0e3;
            }
        }
    }
}

std::vector<visualization_msgs::Marker> DisconnectedJointFilter::getJointMarkersInRRBFrame() const
{
    // Delete other markers
    std::vector<visualization_msgs::Marker> empty_vector;
    visualization_msgs::Marker empty_marker;
    empty_marker.pose.position.x = 0.;
    empty_marker.pose.position.y = 0.;
    empty_marker.pose.position.z = 0.;
    //empty_marker.header.frame_id = "camera_rgb_optical_frame";
    empty_marker.type = visualization_msgs::Marker::SPHERE;
    empty_marker.action = visualization_msgs::Marker::DELETE;
    empty_marker.scale.x = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.scale.y = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.scale.z = 0.0001; //Using start and end points, scale.x is the radius of the array body
    empty_marker.color.a = 0.3;
    empty_marker.color.r = 0.0;
    empty_marker.color.g = 0.0;
    empty_marker.color.b = 1.0;
    empty_marker.ns = "kinematic_structure";
    empty_marker.id = 3 * this->_joint_id;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);
    empty_marker.ns = "kinematic_structure_uncertainty";
    empty_marker.id = 3 * this->_joint_id ;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 1;
    empty_vector.push_back(empty_marker);
    empty_marker.id = 3 * this->_joint_id + 2;
    empty_vector.push_back(empty_marker);

    return empty_vector;
}

JointFilterType DisconnectedJointFilter::getJointFilterType() const
{
    return DISCONNECTED_JOINT;
}

std::string DisconnectedJointFilter::getJointFilterTypeStr() const
{
    return std::string("DisconnectedJointFilter");
}


