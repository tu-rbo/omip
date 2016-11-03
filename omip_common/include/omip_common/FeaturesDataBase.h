/*
 * FeatureDataBase.h
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

#ifndef NEWFEATUREDB_H_
#define NEWFEATUREDB_H_

#include "omip_common/Feature.h"
#include <boost/unordered_map.hpp>

namespace omip
{

class FeaturesDataBase
{
public:

  // Shared pointer to the Feature DataBase
  typedef boost::shared_ptr<FeaturesDataBase> Ptr;

  // Map of Feature Ids to Feature shared pointers
  typedef boost::unordered_map<Feature::Id, Feature::Ptr> MapOfFeatures;

  // Map of Feature Ids to Feature locations
  typedef boost::unordered_map<Feature::Id, Feature::Location> MapOfFeatureLocations;

  // Map of Feature Ids to pairs of Feature locations
  typedef boost::unordered_map<Feature::Id, Feature::LocationPair > MapOfFeatureLocationPairs;

  /**
   * Constructor
   */
  FeaturesDataBase();

  /**
   * Destructor
   */
  virtual ~FeaturesDataBase();

  /**
   * Add a new location of a Feature that could be already in the data base or not
   * The Id of the Feature is added to the alive Features list
   * @param f_id - Id of the Feature that gets a new Location
   * @param f_loc - New Location of the Feature
   * @return - TRUE if the Feature is new and FALSE if it was already contained int he data base
   */
  bool addFeatureLocation(Feature::Id f_id, Feature::Location f_loc);

  /**
   * Add the first location of a new Feature, requiring to create first the new Feature
   * The Id of the Feature is added to the alive Features list
   * @param f_loc - First location of the new Feature
   * @return - Feature Id of the new Feature
   */
  Feature::Id addFeatureLocation(Feature::Location f_loc);

  /**
   * Step the Data base:
   *    1: Update the frame counter (used to set the Feature birthdays)
   *    2: Delete from the Data base all Features that have not been updated (get a new Location)
   *    in the last step
   */
  void step();

  /**
   * Clear the list of Ids of alive Features
   * Every time a new Location of a contained Feature or the first Location of a new Feature is added,
   * the Id of that Feature gets into the list of alive Features
   * This Function MUST be called before the next iteration!!!!
   * (before it was done internally in the step function)
   */
  void clearListOfAliveFeatureIds();

  /**
   * Get a vector containing the Ids of the Features that are still alive
   * @return - vector of Ids of alive Features
   */
  std::vector<Feature::Id> getListOfAliveFeatureIds() const;

  /**
   * Get the clone of a Feature
   * @param f_id - Id of the Feature to get the clone from
   * @return - Shared pointer to the cloned Feature
   */
  Feature::Ptr getFeatureClone(Feature::Id f_id) const;

  /**
   * Get a shared pointer to a Feature
   * @param f_id - Id of the Feature to get the shared pointer from
   * @return - Shared pointer to the Feature
   */
  Feature::Ptr getFeature(Feature::Id f_id) const;

  /**
   * Get first Location of a Feature
   * @param f_id - Id of the Feature to get the first Location from
   * @return - First Location of a Feature
   */
  Feature::Location getFeatureFirstLocation(Feature::Id f_id) const;

  /**
   * Get last location of a Feature
   * @param f_id - Id of the Feature to get the last Location from
   * @return - Last Location of a Feature
   */
  Feature::Location getFeatureLastLocation(Feature::Id f_id) const;

  /**
   * Get next to last location of a Feature
   * @param f_id - Id of the Feature to get the next to last Location from
   * @return - Next to last Location of a Feature
   */
  Feature::Location getFeatureNextToLastLocation(Feature::Id f_id) const;

  /**
   * Get the two last Locations (last and next to last) of a Feature
   * @param f_id - Id of the Feature to get the two last Locations from
   * @return - two last Locations of a Feature
   */
  Feature::LocationPair getFeatureTwoLastLocations(Feature::Id f_id) const;

  /**
   * Get the Location of a Feature N frames before the last Location
   * @param f_id - Id of the Feature to get the two last Locations from
   * @param frames_to_last - Number of frames before the last frame where we want the Location from
   * @return - Location of a Feature in the last minus N frame
   */
  Feature::Location getFeatureNToLastLocation(Feature::Id f_id, int frames_to_last) const;

  /**
   * Get the Trajectory of a Feature
   * @param f_id - Id of the Feature to get the Trajectory from
   * @return - Trajectory of a Feature
   */
  Feature::Trajectory getFeatureTrajectory(Feature::Id f_id) const;

  /**
   * Get the age of a Feature
   * @param f_id - Id of the Feature to get the age from
   * @return - Age of a Feature
   */
  size_t getFeatureAge(Feature::Id f_id) const;

  /**
   * Check if a Feature is stored in the Data base
   * @param f_id - Id of the Feature we want to know if is stored or not
   * @return - TRUE if the Feature with the given Id is in the Database
   */
  bool isFeatureStored(Feature::Id f_id) const;

  /**
   * Get the x coordinate of the last Location of a Feature
   * @param f_id - Id of the Feature to get the x coordinate from
   * @return - x coordinate of the last Location of a Feature
   */
  double getFeatureLastX(Feature::Id f_id) const;

  /**
   * Get the y coordinate of the last Location of a Feature
   * @param f_id - Id of the Feature to get the y coordinate from
   * @return - y coordinate of the last Location of a Feature
   */
  double getFeatureLastY(Feature::Id f_id) const;

  /**
   * Get the z coordinate of the last Location of a Feature
   * @param f_id - Id of the Feature to get the z coordinate from
   * @return - z coordinate of the last Location of a Feature
   */
  double getFeatureLastZ(Feature::Id f_id) const;

  /**
   * Get a map of Feature::Ids -> Feature last location
   * @return - Map with the last locations of all contained Features
   */
  MapOfFeatureLocations getAllFeaturesLastLocation() const;

  /**
   * Get a map of Feature::Ids -> Feature next to last location
   * @return - Map with the next to last locations of all contained Features
   */
  MapOfFeatureLocations getAllFeaturesNextToLastLocation() const;

  /**
   * Get a map of Feature::Ids -> Feature last and next to last location
   * @return - Map with last and the next to last locations of all contained Features
   */
  MapOfFeatureLocationPairs getAllFeaturesTwoLastLocations() const;

//  Feature::LocationPair getOneFeatureLastAndOtherLocations(Feature::Id f_id, int frames_to_last) const;

  /**
   * Get a map of Feature::Ids -> Feature::Ptr
   * @return - Map with the shared pointers of all contained Features
   */
  MapOfFeatures getAllFeatures() const;

  /**
   * Get the frame counter of this Database (the number of times that this->step() was called()
   * @return - Frame counter
   */
  int getTime() const;

protected:

  MapOfFeatures _map_of_features;

  std::vector<Feature::Id> _alive_feat_ids;

  int _time;
};

}

#endif /* NEWFEATUREDB_H_ */
