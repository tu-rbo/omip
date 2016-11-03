/*
 * FeatureTracker.h
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

#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <omip_common/RecursiveEstimatorFilterInterface.h>
#include <omip_common/OMIPTypeDefs.h>
#include <omip_common/FeaturesDataBase.h>

//ROS and OpenCV
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/gpumat.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <feature_tracker/FeatureTrackerDynReconfConfig.h>

namespace omip
{

/**
 * @brief Interface to be implemented by feature trackers
 * It derives from the RecursiveEstimator interface
 *
 */

class FeatureTracker : public RecursiveEstimatorFilterInterface<ft_state_t, ft_measurement_t>
{
public:

    /**
    * @brief Default constructor
    *
    */
    FeatureTracker(double loop_period_ns) :
        RecursiveEstimatorFilterInterface(loop_period_ns)
    {
    }

    /**
     * @brief Default destructor (empty)
     *
     */
    virtual ~FeatureTracker()
    {
    }

    /**
     * @brief First step when updating the filter. The next state is predicted from current state and system model (part of the RecursiveEstimatorInterace)
     * In the FeatureTracker the system model is the most simple one: no change in the state. So the predicted state is just the current state.
     *
     */
    virtual void predictState(double time_interval_ns) = 0;

    /**
     * @brief Corrects the predicted state(s). First we track the features using both the prediction with the system model (copy of the previous feature locations)
     * and if there is a prediction from the higher level (RBT) we track features using it too. We then decide which prediction is the "most likely" based on the
     * quality measurement that the feature tracker returns and use the new location to update the belief state
     *
     */
    virtual void correctState() = 0;

    /**
     * Get a point cloud with the 3D locations and the unique Feature Id (L value) of the last
     * tracked features
     * @return - Shared pointer to the 3D XYZL point cloud of tracked Features
     */
    virtual ft_state_t getState() const = 0;

    /**
    * Set the input RGB-D point cloud
    * @param full_rgb_pc - Shared pointer to the input RGB-D point cloud
    */
    virtual void setFullRGBDPC(PointCloudPCL::ConstPtr full_rgb_pc)
    {
    }

    /**
    * Set the input occlusion mask image
    * @param rgb_img - Shared pointer to the input occlusion mask image
    */
    virtual void setOcclusionMaskImg(cv_bridge::CvImagePtr occ_mask_img)
    {
    }

    /**
    * Set the parameters of the camera to project the predicted Feature locations
    * to the image plane
    * @param camera_info - Pointer to the CameraInfo message containing the camera parameters
    */
    virtual void setCameraInfoMsg(const sensor_msgs::CameraInfo* camera_info)
    {
    }

    virtual void setDynamicReconfigureValues(feature_tracker::FeatureTrackerDynReconfConfig &config) =0;

    virtual cv_bridge::CvImagePtr getRGBImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    virtual cv_bridge::CvImagePtr getDepthImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the last tracked Features as marks in the RGB image
    * @return - Shared pointer to the image with the 2D tracked Feature locations marked
    */
    virtual cv_bridge::CvImagePtr getTrackedFeaturesImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get a pointer to a RGB image with markers where the Features have been tracked in the last step
    * and only showing the RGB area where the features are searched
    * @return - Shared pointer to the RGB image with markers and searching areas
    */
    virtual cv_bridge::CvImagePtr getTrackedFeaturesWithPredictionMaskImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the image with the depth edges, used to reject Features that are on edges
    * @return - Shared pointer to the image of depth edges
    */
    virtual cv_bridge::CvImagePtr getDepthEdgesImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the image with the depth edges, used to reject Features that are on edges
    * @return - Shared pointer to the image of depth edges
    */
    virtual cv_bridge::CvImagePtr getPredictingMaskImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the mask used to track features. Combines edges, predictions mask and max depth
    * @return - Shared pointer to the image of tracking mask
    */
    virtual cv_bridge::CvImagePtr getTrackingMaskImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the mask used to detect new features. Combines edges, max depth and mask of current features (to not detect them)
    * @return - Shared pointer to the image of detecting mask
    */
    virtual cv_bridge::CvImagePtr getDetectingMaskImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }

    /**
    * Get the image with the last feature locations and the predicted feature locations
    * @return - Shared pointer to the image of the predicted and last features
    */
    virtual cv_bridge::CvImagePtr getPredictedAndLastFeaturesImg()
    {
        return cv_bridge::CvImagePtr(new cv_bridge::CvImage());
    }
};
}

#endif /* FEATURE_TRACKER_H_ */
