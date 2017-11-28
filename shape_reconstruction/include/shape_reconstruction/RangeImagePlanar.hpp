/*
 * RangeImagePlanar.hpp
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

#ifndef SHAPE_RECONSTRUCTION_RANGE_IMAGE_PLANAR_IMPL
#define SHAPE_RECONSTRUCTION_RANGE_IMAGE_PLANAR_IMPL

#include "shape_reconstruction/RangeImagePlanar.h"

#include <opencv2/core/core.hpp>

namespace shape_reconstruction {

template <typename PointCloudType> void
RangeImagePlanar::doZBufferAndStorePoints (const PointCloudType& point_cloud,
                                           float noise_level, float min_range, int& top, int& right, int& bottom, int& left)
{
    typedef typename PointCloudType::PointType PointType2;
    const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;

    unsigned int size = width*height;
    int* counters = new int[size];
    ERASE_ARRAY (counters, size);

    resetPointIndex(width, height);

    top=height; right=-1; bottom=-1; left=width;

    float x_real, y_real, range_of_current_point;
    int x, y;
    int idx=0;

    for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it, ++idx)
    {
        if (!isFinite (*it))  // Check for NAN etc
            continue;
        pcl::Vector3fMapConst current_point = it->getVector3fMap ();

        this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
        this->real2DToInt2D (x_real, y_real, x, y);

        if (range_of_current_point < min_range|| !isInImage (x, y))
            continue;
        //std::cout << " ("<<current_point[0]<<", "<<current_point[1]<<", "<<current_point[2]<<") falls into pixel "<<x<<","<<y<<".\n";

        // Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
        int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
                ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));

        int neighbor_x[4], neighbor_y[4];
        neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
        neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
        neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
        neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;
        //std::cout << x_real<<","<<y_real<<": ";

        for (int i=0; i<4; ++i)
        {
            int n_x=neighbor_x[i], n_y=neighbor_y[i];
            //std::cout << n_x<<","<<n_y<<" ";
            if (n_x==x && n_y==y)
                continue;
            if (isInImage (n_x, n_y))
            {
                int neighbor_array_pos = n_y*width + n_x;
                if (counters[neighbor_array_pos]==0)
                {
                    float& neighbor_range = points[neighbor_array_pos].range;
                    //neighbor_range = (pcl_isinf (neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
                    top= (std::min) (top, n_y); right= (std::max) (right, n_x); bottom= (std::max) (bottom, n_y); left= (std::min) (left, n_x);

                    if (pcl_isinf (neighbor_range) || range_of_current_point < neighbor_range) {
                        neighbor_range = range_of_current_point;
                        setPointIndex(n_x, n_y, idx);
                    }

                    //img_coord_to_point_idx[std::make_pair(n_x,n_y)] = idx;
//                    std::cout << "Interpolating point "<<n_x<<","<<n_y << std::endl;
//                    std::cout << " --> idx=" << idx << ", val=" << neighbor_range<< std::endl;
                }
            }
        }
        //std::cout <<std::endl;

        // The point itself
        int arrayPos = y*width + x;
        float& range_at_image_point = points[arrayPos].range;
        int& counter = counters[arrayPos];
        bool addCurrentPoint=false, replace_with_current_point=false;

        if (counter==0)
        {
            replace_with_current_point = true;
        }
        else
        {
            if (range_of_current_point < range_at_image_point-noise_level)
            {
                replace_with_current_point = true;
            }
            else if (fabs (range_of_current_point-range_at_image_point)<=noise_level)
            {
                addCurrentPoint = true;
            }
        }

        if (replace_with_current_point)
        {
//            if (counter == 0) {
//                std::cout << "Replacing point "<<x<<","<<y;
//            } else {
//                std::cout << "Add point "<<x<<","<<y << std::endl;
//                std::cout << ", was " << img_coord_to_point_idx[std::make_pair(x,y)];
//            }
//            std::cout << " --> idx=" << idx << ", val=" << range_of_current_point<< std::endl;

            counter = 1;
            range_at_image_point = range_of_current_point;
            top= (std::min) (top, y); right= (std::max) (right, x); bottom= (std::max) (bottom, y); left= (std::min) (left, x);

            setPointIndex(x, y, idx);
        }
        else if (addCurrentPoint)
        {
            ++counter;
            range_at_image_point += (range_of_current_point-range_at_image_point)/counter;
            //std::cout << "Averaging point "<<x<<","<<y << std::endl;
        }

    }
    delete[] counters;
}

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void
RangeImagePlanar::createFromPointCloudWithFixedSizeAndStorePoints(const PointCloudType& point_cloud,
                                  int di_width, int di_height,
                                  float di_center_x, float di_center_y,
                                  float di_focal_length_x, float di_focal_length_y,
                                  const Eigen::Affine3f& sensor_pose,
                                  CoordinateFrame coordinate_frame, float noise_level,
                                  float min_range)
{
  //std::cout << "Starting to create range image from "<<point_cloud.points.size ()<<" points.\n";

  width = di_width;
  height = di_height;
  center_x_ = di_center_x;
  center_y_ = di_center_y;
  focal_length_x_ = di_focal_length_x;
  focal_length_y_ = di_focal_length_y;
  focal_length_x_reciprocal_ = 1 / focal_length_x_;
  focal_length_y_reciprocal_ = 1 / focal_length_y_;

  is_dense = false;

  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;

  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);

  unsigned int size = width*height;
  points.clear ();
  points.resize (size, unobserved_point);

  int top=height, right=-1, bottom=-1, left=width;

  doZBufferAndStorePoints (point_cloud, noise_level, min_range, top, right, bottom, left);

  // Do not crop
  //cropImage (border_size, top, right, bottom, left);
  recalculate3DPointPositions ();
}

/////////////////////////////////////////////////////////////////////////

template <typename PointCloudType> void
RangeImagePlanar::computeZandMatchingPoints (const PointCloudType& point_cloud,
                                           //float noise_level,
                                           float min_range,
                                           const cv::Mat& matching_im, const float matching_im_min_range,
                                           pcl::PointIndicesPtr& matching_indices_in_pc,
                                           pcl::PointIndicesPtr& matching_indices_in_matching_im,
                                           pcl::PointIndicesPtr& indices_to_remove,
                                           int height, int width)
{
    assert (height == matching_im.rows);
    assert (width == matching_im.cols);
    assert (matching_indices_in_pc);
    assert (matching_indices_in_matching_im);
    assert (indices_to_remove);
    assert (matching_im.type() == CV_32FC1);

    // debug
    pcl::PointIndicesPtr occluded_indices(new pcl::PointIndices);

    int top=height;
    int right=-1;
    int bottom=-1;
    int left=width;

    typedef typename PointCloudType::PointType PointType2;
    const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;

    unsigned int size = width*height;
    int* counters = new int[size];
    ERASE_ARRAY (counters, size);

    std::vector<bool> removal_candidates(points2.size(), true);
    std::vector<bool> matching_in_matching_m(size, false);

    top=height; right=-1; bottom=-1; left=width;

    float x_real, y_real, range_of_current_point;
    int x, y;
    int idx=0;
    int matching_im_idx=0;
    for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it=points2.begin (); it!=points2.end (); ++it, ++idx)
    {
        if (!isFinite (*it))  { // Check for NAN etc
            ROS_ERROR_STREAM_NAMED("RangeImagePlanar.computeZandMatchingPoints", "Error! Point " << idx << " is NaN!");
            throw 0;
            //continue;
        }

        pcl::Vector3fMapConst current_point = it->getVector3fMap ();

        this->getImagePoint (current_point, x_real, y_real, range_of_current_point);
        this->real2DToInt2D (x_real, y_real, x, y);

        if (range_of_current_point < min_range|| !isInImage (x, y)) {
            removal_candidates[idx] = false;
            occluded_indices->indices.push_back(idx); // DEBUGGING
            continue;
        }

        // calculate z for current point
        Eigen::Vector3f current_point_z;
        calculate3DPoint(x, y, range_of_current_point, current_point_z);

        // get values from matching image
        float m_im_z = matching_im.at< float >(y,x);
        matching_im_idx = y*width+x;

        // do some minor interpolation by comparing the z value of the
        // projected point cloud to the z value of the to the four closest neighbors
        // in the matching_im.
        int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
                ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));
        int neighbor_x[4], neighbor_y[4];
        neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
        neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
        neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
        neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;

        for (int i=0; i<4; ++i)
        {
            int n_x=neighbor_x[i], n_y=neighbor_y[i];
            if (n_x==x && n_y==y)
                continue;

            //int neighbor_array_pos = n_y*width + n_x;
            //float& neighbor_range = points[neighbor_array_pos].range;

            // if the neighbor location is outside the image we cannot compare
            if (!isInImage (n_x, n_y)) {
                continue;
            }
            float n_m_im_z = matching_im.at< float >(n_y,n_x);

            // if matching_im is nan at this point we cannot compare
            if (std::isnan(n_m_im_z)) {
                continue;
            }


            // if PC's z matches to the matching_im at the neighbor's position, we keep it and are done
            if ( fabs(n_m_im_z - current_point_z[2]) < matching_im_min_range) {
                removal_candidates[idx] = false;

                matching_im_idx = n_y*width+n_x;
                // avoid adding duplicates
                if (!matching_in_matching_m[matching_im_idx]) {
                    matching_indices_in_matching_im->indices.push_back(matching_im_idx);
                    matching_in_matching_m[matching_im_idx] = true;
                }

//                std::stringstream debug;
//                debug << " Neighbor search -> keeping " << idx
//                 << " (x,y)=" << x << "," << y << " "<< " (n_x,n_y)=" << n_x << "," << n_y
//                 << " matching_im.n_z=" << m_im_z << ", "
//                 << " matching_im.z=" << n_m_im_z << " vs. z=" << current_point_z[2] << std::endl;
//                ROS_DEBUG_STREAM_NAMED("RangeImagePlanar.computeZandMatchingPoints", debug.str());

                // do not break in order to get all matching_indices_in_matching_im
                //break;
            }

            // matching_im neighbor's z is either lower (occluding) or higher (occluded) than the PC's z.
            // so leave the decision to comparing at x,y
        }
        if (removal_candidates[idx] == false) {
            // the neighbor computation found a match; we don't need to look further
            matching_indices_in_pc->indices.push_back(idx);
            //matching_indices_in_matching_im->indices.push_back(matching_im_idx);
            continue;
        }

        // can't say anything about nan image point
        if (std::isnan(m_im_z)) {
            removal_candidates[idx] = false;
            occluded_indices->indices.push_back(idx); // DEBUGGING
            continue;
        }


        if ( fabs(m_im_z - current_point_z[2]) < matching_im_min_range) {
            // keep this point
            removal_candidates[idx] = false;
            matching_indices_in_pc->indices.push_back(idx);
            matching_indices_in_matching_im->indices.push_back(matching_im_idx);
            continue;
        }

        // value from image closer than projected point
        // -> current point is occluded, keep!
        if ( m_im_z < current_point_z[2] ) {
            removal_candidates[idx] = false;
            occluded_indices->indices.push_back(idx); // DEBUGGING
            continue;
        }



        /*
        // Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
        int floor_x = pcl_lrint (floor (x_real)), floor_y = pcl_lrint (floor (y_real)),
                ceil_x  = pcl_lrint (ceil (x_real)),  ceil_y  = pcl_lrint (ceil (y_real));

        int neighbor_x[4], neighbor_y[4];
        neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
        neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
        neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
        neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;

        for (int i=0; i<4; ++i)
        {
            int n_x=neighbor_x[i], n_y=neighbor_y[i];
            //std::cout << n_x<<","<<n_y<<" ";
            if (n_x==x && n_y==y)
                continue;
            if (isInImage (n_x, n_y))
            {
                int neighbor_array_pos = n_y*width + n_x;
                if (counters[neighbor_array_pos]==0)
                {
                    float& neighbor_range = points[neighbor_array_pos].range;
                    neighbor_range = (pcl_isinf (neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
                    top= (std::min) (top, n_y); right= (std::max) (right, n_x); bottom= (std::max) (bottom, n_y); left= (std::min) (left, n_x);

                    Eigen::Vector3f n_point;
                    calculate3DPoint(n_x, n_y, neighbor_range, n_point);

                    if ( fabs(m_im_z - n_point[2]) < matching_im_min_range) {
                        // remove this point
                        matching_indices->indices.push_back(idx);
                        removal_candidates[idx] = false;
                        break;
                    }

                }
            }
        }
        */
    }

    // finally collect removal_candidates
    for (int i = 0; i < points2.size(); i++) {
        if (removal_candidates[i]) {
          indices_to_remove->indices.push_back(i);
        }
    }

    // sanity check
//    for (int i = 0; i < points2.size(); i++) {
//        if (!isFinite(points2[i])) {
//            ROS_ERROR_STREAM_NAMED("RangeImagePlanar.computeZandMatchingPoints", "Error! Point " << i << " is NaN!");
//            throw 0;
//        }
//    }

//    {
//    std::stringstream debug;
//    debug << " occluded = " << occluded_indices->indices.size();
//    debug << " matching = " << matching_indices_in_pc->indices.size();
//    debug << " matching in dm_source = " << matching_indices_in_matching_im->indices.size();
//    debug << " removal  = " << indices_to_remove->indices.size() << std::endl;
//    debug << "  SUM = " << (occluded_indices->indices.size()+indices_to_remove->indices.size()+matching_indices_in_pc->indices.size()) << std::endl;
//    debug << "  TOTAL = " << points2.size() << std::endl;
//    }

    delete[] counters;

}


template <typename PointCloudType> void
RangeImagePlanar::matchPointCloudAndImage (const PointCloudType& point_cloud,
                                           int di_width, int di_height,
                                           float di_center_x, float di_center_y,
                                           float di_focal_length_x, float di_focal_length_y,
                                           const Eigen::Affine3f& sensor_pose,
                                           CoordinateFrame coordinate_frame,
                                           const cv::Mat& matching_im, const float matching_im_min_range,
                                           pcl::PointIndicesPtr& matching_indices,
                                           pcl::PointIndicesPtr& matching_indices_in_matching_im,
                                           pcl::PointIndicesPtr& indices_to_remove,
                                           float min_range
                                           )
{
  //std::cout << "Starting to create range image from "<<point_cloud.points.size ()<<" points.\n";

  width = di_width;
  height = di_height;
  center_x_ = di_center_x;
  center_y_ = di_center_y;
  focal_length_x_ = di_focal_length_x;
  focal_length_y_ = di_focal_length_y;
  focal_length_x_reciprocal_ = 1 / focal_length_x_;
  focal_length_y_reciprocal_ = 1 / focal_length_y_;

  is_dense = false;

  getCoordinateFrameTransformation (coordinate_frame, to_world_system_);
  to_world_system_ = sensor_pose * to_world_system_;

  to_range_image_system_ = to_world_system_.inverse (Eigen::Isometry);

  unsigned int size = width*height;
  points.clear ();
  points.resize (size, unobserved_point);

  computeZandMatchingPoints (point_cloud,
                             min_range,
                             matching_im,
                             matching_im_min_range,
                             matching_indices,
                             matching_indices_in_matching_im,
                             indices_to_remove,
                             height, width);

}


}

#endif
