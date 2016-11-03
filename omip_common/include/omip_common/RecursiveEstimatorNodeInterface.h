/*
 * RecursiveEstimatorNodeInterface.h
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

#ifndef RECURSIVE_ESTIMATOR_NODE_INTERFACE_H_
#define RECURSIVE_ESTIMATOR_NODE_INTERFACE_H_

#include <vector>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <std_msgs/Empty.h>

namespace omip
{

/**
 * @class RecursiveEstimatorNodeInterface
 * @brief This class defines an INTERFACE that all recursive estimator NODES need to implement
 * It uses ROS
 * TODO: For multimodality I will need different measurement types
 */
template <class MeasurementTypeROS, class StateTypeROS, class RecursiveEstimatorFilterClass>
class RecursiveEstimatorNodeInterface
{
public:

    /**
     * @brief Constructor
     *
     * @param num_external_state_predictors - Number of state predictors that the node will connect to
     */
    RecursiveEstimatorNodeInterface(int num_external_state_predictors) :
        _re_filter(NULL),
        _num_external_state_predictors(num_external_state_predictors),
        _active(true)

    {
        // We create a separate node handle with its own callback queue to process the measurement (only 1 type of measurement?)
        this->_measurements_queue = ros::getGlobalCallbackQueue();
        this->_measurements_node_handle.setCallbackQueue(this->_measurements_queue);

        // We want to process the predictions about the state as soon as they arrive
        // Therefore, we create a separate node handle with separate callback queues for each external state predictor
        this->_state_prediction_node_handles.resize(this->_num_external_state_predictors);
        this->_state_prediction_queues.resize(this->_num_external_state_predictors, NULL);
        for(int num_external_state_predictors_idx = 0; num_external_state_predictors_idx < this->_num_external_state_predictors; num_external_state_predictors_idx++)
        {
            this->_state_prediction_queues.at(num_external_state_predictors_idx) = new ros::CallbackQueue(true);
            this->_state_prediction_node_handles.at(num_external_state_predictors_idx).setCallbackQueue((this->_state_prediction_queues.at(num_external_state_predictors_idx)));
        }

        this->_current_measurement_time.fromNSec(0);
        this->_previous_measurement_time.fromNSec(0);

        // All nodes subscribe to the "shutdown" signal
        this->_node_quit_subscriber = this->_measurements_node_handle.subscribe("/omip/shutdown", 1, &RecursiveEstimatorNodeInterface::quitCallback, this);
    }

    /**
     * @brief Destructor
     *
     */
    virtual ~RecursiveEstimatorNodeInterface()
    {
        if(this->_re_filter)
        {
            delete this->_re_filter;
            this->_re_filter = NULL;
        }
    }

    /**
     * @brief Creates a thread per state predictor that listens to their predictions, and this thread will listen to measurements
     *
     */
    virtual void run()
    {
        // For each predictor of states...
        for(int idx = 0; idx < this->_num_external_state_predictors; idx++)
        {
            // ... we create a thread that spins on the callback queue of the predictor
            this->_state_predictor_listener_threads.push_back(new boost::thread(boost::bind(&RecursiveEstimatorNodeInterface::spinStatePredictorQueue, this, idx)));
        }
        // Spin here the queue of the measurements
        while (ros::ok() && this->_active)
        {
            //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
            this->_measurements_queue->callAvailable(ros::WallDuration(0.005));
        }
    }

    /**
     * @brief Spins one queue of predictors
     *
     * @param state_prediction_queue_idx - The idx of the queue of predictions
     */
    virtual void spinStatePredictorQueue(int state_prediction_queue_idx)
    {
        while (ros::ok() && this->_active)
        {
            this->_state_prediction_queues.at(state_prediction_queue_idx)->callAvailable(ros::WallDuration(0.01));
        }
    }

    /**
     * @brief Callback for the state predictions
     *
     * @param predicted_next_state
     */
    virtual void measurementCallback(const boost::shared_ptr<MeasurementTypeROS const> & measurement) = 0;

    /**
     * @brief Callback for the state predictions
     *
     * @param predicted_next_state
     */
    virtual void statePredictionCallback(const boost::shared_ptr<StateTypeROS const> & predicted_next_state) = 0;

    /**
     * @brief Callback to exit the node
     *
     * @param msg - Empty message that signals that the node should terminate
     */
    virtual void quitCallback(const std_msgs::EmptyConstPtr &msg)
    {
        ROS_INFO_STREAM("RecursiveEstimatorNode stopping!");
        _active = false;
    }

    /**
     * Gets a parameter from ROS
     */
    template<class T>
    bool getROSParameter(std::string param_name, T & param_container)
    {
        if (!(this->_measurements_node_handle.getParam(param_name, param_container)))
        {
            ROS_ERROR("The parameter %s can not be found.", param_name.c_str());
            throw(std::string("The parameter can not be found. Parameter name: ") + param_name);
            return false;
        }
        else
            return true;
    }

    /**
     * Gets a parameter from ROS with a default value if it is not available
     */
    template<class T>
    bool getROSParameter(std::string param_name, T & param_container, const T & default_value)
    {
        if (!(this->_measurements_node_handle.getParam(param_name, param_container)))
        {
            ROS_WARN_STREAM("The parameter " << param_name <<" can not be found. Using the default value " << default_value );
            param_container = default_value;
            return false;
        }
        else
            return true;
    }

protected:

    /**
     * @brief Publish the current state of this RE level
     *
     */
    virtual void _publishState() const = 0;

    /**
     * @brief Publish the prediction about the next measurement by this RE level
     *
     */
    virtual void _publishPredictedMeasurement() const = 0;


    RecursiveEstimatorFilterClass* _re_filter;

    std::string _namespace;

    // We create a separate thread to listen to a separate queue where the state predictions from higher level will arrive
    // Right now the hierarchy that I developed uses only one prediction from higher levels, but nobody knows how it will evolve ;)
    int _num_external_state_predictors;
    std::vector<ros::NodeHandle> _state_prediction_node_handles;
    std::vector<ros::CallbackQueue*> _state_prediction_queues;
    std::vector<boost::thread*> _state_predictor_listener_threads;

    // We have only one subscriber for state predictions (how to create several of them?)
    ros::Subscriber _measurement_subscriber;
    ros::Subscriber _state_prediction_subscriber;
    ros::Subscriber _node_quit_subscriber;
    ros::Publisher _state_publisher;
    ros::Publisher _state_prediction_publisher;
    ros::Publisher _measurement_prediction_publisher;

    // I have to set the measurement queue to be the global queue so that the dynamic reconfigure works
    // That is done in the constructor
    ros::CallbackQueue* _measurements_queue;
    ros::NodeHandle _measurements_node_handle;

    // These 2 times contain the time of the previous and current measurement that are processed
    ros::Time       _current_measurement_time;
    ros::Time       _previous_measurement_time;
    ros::Duration   _loop_period_ns;

    bool            _active;
};
}

#endif /* RECURSIVE_ESTIMATOR_NODE_INTERFACE_H_ */
