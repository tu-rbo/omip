#include "omip_common/Feature.h"

#include <ros/ros.h>

using namespace omip;

Feature::Id Feature::_feature_id_generator = 1;

Feature::Feature() :
    _id(Feature::_feature_id_generator++), _first_location(0.0, 0.0, 0.0), _birthday(
        -1)
{
}

Feature::Feature(int feature_birthday, Location first_fl_in) :
    _id(Feature::_feature_id_generator++), _first_location(first_fl_in), _birthday(
        feature_birthday)
{
  this->_trajectory.push_back(first_fl_in);
}

// To be used when the feature Ids are set externally
Feature::Feature(int feature_birthday, Location first_fl_in, Id f_id) :
    _id(f_id), _first_location(first_fl_in), _birthday(feature_birthday)
{
  this->_trajectory.push_back(first_fl_in);
}

Feature::Feature(const Feature &f)
{
  this->_id = f.getId();
  this->_first_location = f.getFirstLocation();
  this->_birthday = f.getFeatureBirthday();
  this->_trajectory = f.getTrajectory();
}

Feature::~Feature()
{

}

Feature::Ptr Feature::clone() const
{
  return (Ptr(doClone()));
}

void Feature::addLocation(Location fl_in)
{
  if (this->_trajectory.size() == 0)
  {
    this->_first_location = fl_in;
  }
  this->_trajectory.push_back(fl_in);
}

Feature::Id Feature::getId() const
{
  return this->_id;
}

Feature::Location Feature::getFirstLocation() const
{
  return this->_first_location;
}

Feature::Location Feature::getLastLocation() const
{
  if (this->_trajectory.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(
        "Feature.getLastLocation",
        "Feature with Id " << this->_id << " has no last location.");
    return Location();
  }
  else
  {
    return this->_trajectory.back();
  }
}

Feature::Location Feature::getNextToLastLocation() const
{
  if (this->_trajectory.size() < 2)
  {
    ROS_ERROR_STREAM_NAMED(
        "Feature.getNextToLastLocation",
        "Feature with Id " << this->_id << " has no next to last location.");
    return Location();
  }
  else
  {
    return this->_trajectory.at(this->_trajectory.size() - 2);
  }
}

Feature::LocationPair Feature::getTwoLastLocations() const
{
  return Feature::LocationPair(this->getLastLocation(), this->getNextToLastLocation());
}

Feature::Location Feature::getNToLastLocation(int frames_to_last) const
{
  if ((int)this->_trajectory.size() - frames_to_last - 1 < 0)
  {
    ROS_ERROR_STREAM_NAMED(
        "Feature.getNToLastLocation",
        "Feature with Id " << this->_id << " was not born at that time (" << frames_to_last << " frames to last < feature age = " << this->_trajectory.size() <<").");
    return Location();
  }
  else
  {
    return this->_trajectory.at(this->_trajectory.size() - frames_to_last -1);
  }
}

const Feature::Trajectory& Feature::getTrajectory() const
{
  return this->_trajectory;
}

//void Feature::getTrajectory(const Feature::Trajectory& feat_traject) const
//{
//  feat_traject = this->_trajectory;
//}

size_t Feature::getFeatureAge() const
{
  return this->_trajectory.size();
}

int Feature::getFeatureBirthday() const
{
  return this->_birthday;
}

double Feature::getLastX() const
{
  return boost::tuples::get<0>(this->getLastLocation());
}

double Feature::getLastY() const
{
  return boost::tuples::get<1>(this->getLastLocation());
}

double Feature::getLastZ() const
{
  return boost::tuples::get<2>(this->getLastLocation());
}

void Feature::setFeatureBirthday(int feature_birthday)
{
  this->_birthday = feature_birthday;
}

