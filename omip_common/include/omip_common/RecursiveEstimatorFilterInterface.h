/*
 * RecursiveEstimatorFilterInterface.h
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

#ifndef RECURSIVE_ESTIMATOR_INTERFACE_H_
#define RECURSIVE_ESTIMATOR_INTERFACE_H_

#include <vector>
#include <string>
#include <iostream>

namespace omip
{

/**
 * @class RecursiveEstimatorFilterInterface
 * @brief This class defines an INTERFACE that all recursive estimator filters need to implement
 * It is ROS free
 * It is templated on the type of the state that it estimates and the type of measurements it accepts as input
 * TODO: For multimodality I will need different measurement types
 */
template <class StateType, class MeasurementType>
class RecursiveEstimatorFilterInterface
{
public:

    /**
    * @brief Constructor
    *
    */
    RecursiveEstimatorFilterInterface(double loop_period_ns) :
        _loop_period_ns(loop_period_ns),
        _state(),
        _most_likely_predicted_state(),
        _measurement(),
        _predicted_measurement(),
        _measurement_timestamp_ns(0.0),
        _previous_measurement_timestamp_ns(0.0)
    {

    }

    /**
     * @brief Destructor
     *
     */
    virtual ~RecursiveEstimatorFilterInterface()
    {

    }

    /**
     * @brief First step when updating the filter. The next state is predicted from current state and system model
     *
     */
    virtual void predictState(double time_interval_ns) = 0;

    /**
     * @brief Second step when updating the filter. The next measurement is predicted from the predicted next state
     *
     */
    virtual void predictMeasurement() = 0;

    /**
     * @brief Third and final step when updating the filter. The predicted next state is corrected based on the difference
     * between predicted and acquired measurement
     *
     */
    virtual void correctState() = 0;

    /**
     * @brief Set the latest acquired measurement
     *
     * @param acquired_measurement Latest acquired measurement
     */
    virtual void setMeasurement(const MeasurementType& acquired_measurement, const double& measurement_timestamp)
    {
        this->_measurement = acquired_measurement;
        this->_previous_measurement_timestamp_ns = this->_measurement_timestamp_ns;
        this->_measurement_timestamp_ns = measurement_timestamp;
    }

    /**
     * @brief Add a new prediction about the state generated in the higher level (as prediction about the
     * next measurement)
     *
     * @param predicted_state Predicted next state
     */
    virtual void addPredictedState(const StateType& predicted_state, const double& predicted_state_timestamp_ns) = 0;

    /**
     * @brief Get the predicted next measurement.
     *
     * @return MeasurementType MeasurementType Predicted next measurement based on the predicted next state
     */
    virtual MeasurementType getPredictedMeasurement() const
    {
        return this->_predicted_measurement;
    }

    /**
     * @brief Get the currently belief state
     *
     * @return StateType Currently belief state
     */
    virtual StateType getState() const
    {
        return this->_state;
    }

protected:

    StateType _state;
    MeasurementType _measurement;

    // Period of the recursive loop. It is also the period of the measurements. This time is used to predict the next state (and then the next measurement)
    // at the end of one processing loop, so that the prediction can:
    //  1> be passed to the lower level as alternative predicted next state
    //  2> as predicted measurement for this level, which we compare to the acquired measurement to correct the state
    // If we lose some ticks (computing time is longer than the loop period) we will have to predict again using the real interval between measurements
    double _loop_period_ns;

    // Timestamp of the last measurement in nano seconds
    double _measurement_timestamp_ns;

    // Timestamp of the previous acquired (and processed) measurement in nano seconds
    // We use the difference between timestamps of last and previous measurements to compute the "real" loop period in the last iteration and decide if it is
    // close enough to our expected loop period (valid predictions) or if we have lost some ticks (recompute predictions)
    double _previous_measurement_timestamp_ns;

    std::string _filter_name;

    //We use a vector because we can have more than one predicted state. Usually we will have two: the predicted state using the internal model
    //and the predicted state coming from the higher level (is the predicted next measurement of the next level)
    //The normal procedure is to test both and use the most likely one
    std::vector<StateType> _predicted_states;

    //We find the most likely predicted state and store it here
    StateType _most_likely_predicted_state;

    MeasurementType _predicted_measurement;

};
}

#endif /* RECURSIVE_ESTIMATOR_INTERFACE_H_ */
