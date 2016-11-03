#include "omip_common/FeaturesDataBase.h"

#include <boost/foreach.hpp>

#include <ros/ros.h>

using namespace omip;

FeaturesDataBase::FeaturesDataBase() :
    _time(0)
{

}

FeaturesDataBase::~FeaturesDataBase()
{

}

bool FeaturesDataBase::addFeatureLocation(Feature::Id f_id,
                                          Feature::Location f_loc)
{
  MapOfFeatures::iterator feat_it = this->_map_of_features.find(f_id);
  if (feat_it == this->_map_of_features.end())
  {
    // This is a new feature
    Feature::Ptr new_feature = Feature::Ptr(
        new Feature(this->_time, f_loc, f_id));
    this->_map_of_features[f_id] = new_feature;
    this->_alive_feat_ids.push_back(new_feature->getId());
    return true;
  }
  else
  {
    // This feature is already in the data base
    feat_it->second->addLocation(f_loc);
    this->_alive_feat_ids.push_back(f_id);
    return false;
  }
}

Feature::Id FeaturesDataBase::addFeatureLocation(Feature::Location f_loc)
{
  Feature::Ptr new_feature = Feature::Ptr(new Feature(this->_time, f_loc));
  this->_map_of_features[new_feature->getId()] = new_feature;
  this->_alive_feat_ids.push_back(new_feature->getId());
  return new_feature->getId();
}

void FeaturesDataBase::step()
{
  // Delete all features from the data base that has not been updated in the last loop
  for (MapOfFeatures::iterator feats_it = this->_map_of_features.begin(); feats_it != this->_map_of_features.end();)
  {
    //The feature is not alive anylonger! -> delete it from the map
    if (std::find(this->_alive_feat_ids.begin(), this->_alive_feat_ids.end(),feats_it->first) == this->_alive_feat_ids.end())
    {
      feats_it = this->_map_of_features.erase(feats_it);
    }
    else
    {
      ++feats_it;
    }
  }
  this->_time++;
}

void FeaturesDataBase::clearListOfAliveFeatureIds()
{
  this->_alive_feat_ids.clear();
}

std::vector<Feature::Id> FeaturesDataBase::getListOfAliveFeatureIds() const
{
  return this->_alive_feat_ids;
}

Feature::Ptr FeaturesDataBase::getFeatureClone(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureClone",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Ptr(new Feature());
  }
  else
  {
    return this->_map_of_features.at(f_id)->clone();
  }
}

Feature::Ptr FeaturesDataBase::getFeature(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED("FeaturesDataBase.getFeature", "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Ptr(new Feature());
  }
  else
  {
    return this->_map_of_features.at(f_id);
  }
}

Feature::Location FeaturesDataBase::getFeatureFirstLocation(
    Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureFirstLocation",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Location();
  }
  else
  {
    return this->_map_of_features.at(f_id)->getFirstLocation();
  }
}

Feature::Location FeaturesDataBase::getFeatureLastLocation(
    Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureLastLocation",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Location();
  }
  else
  {
    return this->_map_of_features.at(f_id)->getLastLocation();
  }
}

Feature::Location FeaturesDataBase::getFeatureNextToLastLocation(
    Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureNextToLastLocation",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Location();
  }
  else
  {
    return this->_map_of_features.at(f_id)->getNextToLastLocation();
  }
}

Feature::LocationPair FeaturesDataBase::getFeatureTwoLastLocations(
    Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureTwoLastLocations",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::LocationPair(Feature::Location(), Feature::Location());
  }
  else
  {
    return this->_map_of_features.at(f_id)->getTwoLastLocations();
  }
}

Feature::Location FeaturesDataBase::getFeatureNToLastLocation(
    Feature::Id f_id, int frames_to_last) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureNToLastLocation",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Location();
  }
  else
  {
    return this->_map_of_features.at(f_id)->getNToLastLocation(frames_to_last);
  }
}

Feature::Trajectory FeaturesDataBase::getFeatureTrajectory(
    Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureTrajectory",
        "Feature with Id " << f_id << " is not stored in the database.");
    return Feature::Trajectory();
  }
  else
  {
    return this->_map_of_features.at(f_id)->getTrajectory();
  }
}

size_t FeaturesDataBase::getFeatureAge(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureAge",
        "Feature with Id " << f_id << " is not stored in the database.");
    return -1;
  }
  else
  {
    return this->_map_of_features.at(f_id)->getFeatureAge();
  }
}

bool FeaturesDataBase::isFeatureStored(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    return false;
  }
  else
  {
    return true;
  }
}

double FeaturesDataBase::getFeatureLastX(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureX",
        "Feature with Id " << f_id << " is not stored in the database.");
    return -10.0;
  }
  else
  {
    return this->_map_of_features.at(f_id)->getLastX();
  }
}

double FeaturesDataBase::getFeatureLastY(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureLastY",
        "Feature with Id " << f_id << " is not stored in the database.");
    return -10.0;
  }
  else
  {
    return this->_map_of_features.at(f_id)->getLastY();
  }
}

double FeaturesDataBase::getFeatureLastZ(Feature::Id f_id) const
{
  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
  {
    ROS_ERROR_STREAM_NAMED(
        "FeaturesDataBase.getFeatureLastZ",
        "Feature with Id " << f_id << " is not stored in the database.");
    return -10.0;
  }
  else
  {
    return this->_map_of_features.at(f_id)->getLastZ();
  }
}

FeaturesDataBase::MapOfFeatureLocations FeaturesDataBase::getAllFeaturesLastLocation() const
{
  MapOfFeatureLocations ultimate_locations;
  BOOST_FOREACH(MapOfFeatures::value_type feat_id_feat, this->_map_of_features)
  {
    ultimate_locations[feat_id_feat.first] = feat_id_feat.second
        ->getLastLocation();
  }
  return ultimate_locations;
}

FeaturesDataBase::MapOfFeatureLocations FeaturesDataBase::getAllFeaturesNextToLastLocation() const
{
  MapOfFeatureLocations penultimate_locations;
  BOOST_FOREACH(MapOfFeatures::value_type feat_id_feat, this->_map_of_features)
  {
    penultimate_locations[feat_id_feat.first] = feat_id_feat.second
        ->getNextToLastLocation();
  }
  return penultimate_locations;
}

FeaturesDataBase::MapOfFeatureLocationPairs FeaturesDataBase::getAllFeaturesTwoLastLocations() const
{
  MapOfFeatureLocationPairs two_last_locations;
  BOOST_FOREACH(MapOfFeatures::value_type feat_id_feat, this->_map_of_features)
  {
    two_last_locations[feat_id_feat.first] = feat_id_feat.second
        ->getTwoLastLocations();
  }
  return two_last_locations;
}

//std::pair<Location, Location> FeaturesDataBase::getLastAndOtherLocationsOfFeature(Id f_id, int frames_to_last) const
//{
//  if (this->_map_of_features.find(f_id) == this->_map_of_features.end())
//    {
//      ROS_ERROR_STREAM_NAMED("FeaturesDataBase::getLastTwoLocationsOfFeature",
//                             "NewFeature with Id " << f_id << " is not stored in the database.");
//      return std::pair<Location, Location>(Location(), Location());
//    }
//    else
//    {
//      std::pair<Location, Location> ret_pair;
//      ret_pair.first =  this->_map_of_features.at(f_id)->getUltimateLocationOfFeature();
//      ret_pair.second = this->_map_of_features.at(f_id)->getLocationOfFeatureCurrentMinusNFrames(frames_to_last);
//      return ret_pair;
//    }
//}

FeaturesDataBase::MapOfFeatures FeaturesDataBase::getAllFeatures() const
{
  return this->_map_of_features;
}

int FeaturesDataBase::getTime() const
{
  return this->_time;
}
