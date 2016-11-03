/*
 * RangeImagePlanar.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014) and the extension to segment, reconstruct and track shapes introduced in our paper
 * "An Integrated Approach to Visual Perception of Articulated Objects" (Martín-Martín, Höfer and Brock, 2016)
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

@inproceedings{martinmartin_hoefer_iros_2016,
Title = {An Integrated Approach to Visual Perception of Articulated Objects},
Author = {Roberto {Mart\'{i}n-Mart\'{i}n} and Sebastian H\"ofer and Oliver Brock},
Booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation},
Pages = {5091 - 5097},
Year = {2016},
Doi = {10.1109/ICRA.2016.7487714},
Location = {Stockholm, Sweden},
Month = {05},
Note = {http://www.redaktion.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martin_hoefer_15_iros_sr_opt.pdf},
Url = {http://www.redaktion.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martin_hoefer_15_iros_sr_opt.pdf},
Url2 = {http://ieeexplore.ieee.org/xpl/articleDetails.jsp?tp=&arnumber=7487714},
Projectname = {Interactive Perception}
}
 * If you have questions or suggestions, contact us:
 * roberto.martinmartin@tu-berlin.de
 *
 * Enjoy!
 */

#ifndef SHAPE_RECONSTRUCTION_RANGE_IMAGE_PLANAR
#define SHAPE_RECONSTRUCTION_RANGE_IMAGE_PLANAR

#include <map>
#include <pcl/range_image/range_image_planar.h>

#include <opencv2/core/core.hpp>


namespace shape_reconstruction {

/**
 * @brief Adapted version of pcl::RangeImagePlanar
 *
 * When creating the RangeImage we additionally track changes
 * to the z-buffer to know which point is projected on which
 * pixel in the range image
 */
class RangeImagePlanar : public pcl::RangeImagePlanar {
public:
    typedef boost::shared_ptr<shape_reconstruction::RangeImagePlanar> Ptr;
    typedef boost::shared_ptr<const shape_reconstruction::RangeImagePlanar> ConstPtr;

    RangeImagePlanar () : pcl::RangeImagePlanar(), img_coord_to_point_idx(NULL) {
        delete img_coord_to_point_idx;
    }

    /** Destructor */
    virtual ~RangeImagePlanar () {
    }

    template <typename PointCloudType> void
    createFromPointCloudWithFixedSizeAndStorePoints (const PointCloudType& point_cloud,
                                       int di_width, int di_height, float di_center_x, float di_center_y,
                                       float di_focal_length_x, float di_focal_length_y,
                                       const Eigen::Affine3f& sensor_pose,
                                       CoordinateFrame coordinate_frame=CAMERA_FRAME, float noise_level=0.0f,
                                       float min_range=0.0f);

    template <typename PointCloudType> void
    doZBufferAndStorePoints (const PointCloudType& point_cloud,
                             float noise_level, float min_range,
                             int& top, int& right, int& bottom, int& left);



    ////////////////////////////////////////

    template <typename PointCloudType> void
    computeZandMatchingPoints (const PointCloudType& point_cloud,
                               //float noise_level,
                               float min_range,
                               const cv::Mat& matching_im, const float matching_im_min_range,
                               pcl::PointIndicesPtr& matching_indices,
                               pcl::PointIndicesPtr& matching_indices_in_matching_im,
                               pcl::PointIndicesPtr& indices_to_remove,
                               int height, int width);

    template <typename PointCloudType> void
    matchPointCloudAndImage (const PointCloudType& point_cloud,
                             int di_width, int di_height,
                             float di_center_x, float di_center_y,
                             float di_focal_length_x, float di_focal_length_y,
                             const Eigen::Affine3f& sensor_pose,
                             CoordinateFrame coordinate_frame,
                             const cv::Mat& matching_im, const float matching_im_min_range,
                             pcl::PointIndicesPtr& matching_indices,
                             pcl::PointIndicesPtr& matching_indices_in_matching_im,
                             pcl::PointIndicesPtr& indices_to_remove,
                             float min_range=0.0f
                             );


    bool getPointIndex(const int& x, const int& y, int& idx) const {
        assert (img_coord_to_point_idx != NULL);
        int arrayPos = y*width + x;
        idx = img_coord_to_point_idx[arrayPos];
        if (idx == 0)
            return false;

        idx--; // see setPointIndex
        return true;
    }

    void setPointIndex(const int& x, const int& y, const int& idx) {
        assert (img_coord_to_point_idx != NULL);
        int arrayPos = y*width + x;
        // small hack: because we use 0 as invalid index,
        // but in a vector 0 actually could be a valid index (of the first point)
        // we add +1, and decrement it again in getPointIndex
        img_coord_to_point_idx[arrayPos] = idx+1;
    }

    void resetPointIndex(const int& width, const int& height) {
        unsigned int size = width*height;
        delete img_coord_to_point_idx;
        img_coord_to_point_idx = new int[size];
        // initial to 0=invalid index
        ERASE_ARRAY (img_coord_to_point_idx, size);
    }


//protected:
//    std::map<std::pair<int,int>, int >  img_coord_to_point_idx;
    int*  img_coord_to_point_idx;

};

}

#endif
