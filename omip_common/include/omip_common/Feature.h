/*
 * Feature.h
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

#ifndef FEATURE_H_
#define FEATURE_H_

#include <utility>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>

namespace omip
{
/**
 *\class    Feature
 *\brief    Minimal class that represents a Point of Interest by an ID and stores its successive locations
 *$Author: Martin
 *$Revision: 0.1 $
 */
class Feature
{
public:

  // Shared pointer to Feature
  typedef boost::shared_ptr<Feature> Ptr;

  // Feature Id
  typedef long Id;

  // Location of a Feature in 3D
  typedef boost::tuple<double, double, double> Location;

  // Trajectory of a Feature in 3D
  typedef std::vector<Location> Trajectory;

  // Pair of Feature Locations
  typedef std::pair<Location, Location> LocationPair;

  /**
   * Feature constructor
   */
  Feature();

  /**
   * Feature constructor
   * @param feature_birthday - Time stamp when the Feature was initially detected
   * @param first_fl_in - First Feature Location
   */
  Feature(int feature_birthday, Location first_fl_in);

  /**
   * Feature constructor
   * This constructor should be used when the Feature Ids are externally provided
   * @param feature_birthday - Time stamp when the Feature was initially detected
   * @param first_fl_in - First Feature Location
   * @param f_id - Id of the Feature
   */
  Feature(int feature_birthday, Location first_fl_in, Id f_id);

  /**
   * Copy constructor
   * @param f - Feature to be copied
   */
  Feature(const Feature &f);

  /**
   * Default destructor
   */
  virtual ~Feature();

  /**
   * Create a new Feature object as a copy of this and pass a pointer
   * @return - Shared pointer to the copy of this Feature
   */
  Ptr clone() const;

  /**
   * Add a new Location of this Feature
   * @param fl_in - New location of this Feature
   */
  void addLocation(Location fl_in);

  /**
   * Get the Id of this Feature
   * @return - Id of this Feature
   */
  Id getId() const;

  /**
   * Get the first Location of this Feature
   * @return - First Location of this Feature
   */
  Location getFirstLocation() const;

  /**
   * Get the last Location of this Feature
   * @return - Last Location of this Feature
   */
  Location getLastLocation() const;

  /**
   * Get the next to last Location of this Feature
   * @return - Next to last Location of this Feature
   */
  Location getNextToLastLocation() const;

  /**
   * Get the two last Locations (last and next to last) of this Feature
   * @return - Two last Location of this Feature
   */
  LocationPair getTwoLastLocations() const;

  /**
   * Get the Location of this Feature N frames before the last Location
   * @param frames_to_last - Number of frames before the last frame where we want the Location from
   * @return - Location of this Feature in the last minus N frame
   */
  Location getNToLastLocation(int frames_to_last) const;

  /**
   * Get the Trajectory of this Feature
   * @return - Trajectory of this Feature
   */
  const Trajectory& getTrajectory() const;

  /**
   * Get the age of this Feature
   * @return - Age of this Feature
   */
  size_t getFeatureAge() const;

  /**
   * Get this Feature's Birthday (frame number)
   * @return - Birthday of this Feature as the frame when it was initially detected
   */
  int getFeatureBirthday() const;

  /**
   * Get the x coordinate of the last Location of this Feature
   * @return - x coordinate of the last Location of this Feature
   */
  double getLastX() const;

  /**
   * Get the y coordinate of the last Location of this Feature
   * @return - y coordinate of the last Location of this Feature
   */
  double getLastY() const;

  /**
   * Get the z coordinate of the last Location of this Feature
   * @return - z coordinate of the last Location of this Feature
   */
  double getLastZ() const;

  /**
   * Set this Feature's Birthday (frame number)
   * @param feature_birthday - Birthday of this Feature as the frame when it was initially detected
   */
  void setFeatureBirthday(int feature_birthday);

protected:

  // Unique counter to generate Feature ids
  static Id _feature_id_generator;

  // Unique identifier for every Feature
  Id _id;

  // Trajectory (sequence of Locations) of this Feature
  Trajectory _trajectory;

  // First location of this Feature
  Location _first_location;

  // Birthday (frame number) when this Feature was initially detected
  int _birthday;

  // Cloning function that uses the copy constructor
  virtual Feature* doClone() const
  {
    return (new Feature(*this));
  }

};

}
;

#endif /* FEATURE_H_ */
