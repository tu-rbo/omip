/*
 * SRUtils.h
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

#ifndef SR_UTILS_H_
#define SR_UTILS_H_

#include "shape_reconstruction/RangeImagePlanar.h"
#include "omip_common/OMIPUtils.h"
#include "omip_common/OMIPTypeDefs.h"

#include <opencv2/core/core.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <boost/circular_buffer.hpp>

namespace omip
{

// Comment or uncomment this line to test the downsampling (WIP -> RangeImagePlanar?)
//#define DOWNSAMPLING

#ifdef DOWNSAMPLING
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#else
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#endif

#define NUM_SCALES 3

typedef pcl::PointXYZRGB SRPoint;
typedef pcl::PointCloud<SRPoint> SRPointCloud;

void RangeImagePlanar2DepthMap(const pcl::RangeImagePlanar::Ptr& rip, cv::Mat& depth_map_mat);

void RangeImagePlanar2PointCloud(const pcl::RangeImagePlanar::Ptr& rip, SRPointCloud::Ptr& pc);

void OrganizedPC2DepthMap(const SRPointCloud::Ptr organized_pc, cv::Mat& depth_map_mat);
void OrganizedPC2DepthMapAlternative(const SRPointCloud::Ptr organized_pc, const cv::Ptr<cv::Mat>& depth_map_mat_ptr);
void OrganizedPC2ColorMap(const SRPointCloud::Ptr organized_pc, cv::Mat& color_map_mat);


void UnorganizedPC2DepthMap(const SRPointCloud::Ptr& organized_pc,
                            const int& width,
                            const int& height,
                            cv::Mat& depth_map,
                            ::shape_reconstruction::RangeImagePlanar::Ptr& rip,
                            const float& noiseLevel=0.f,
                            const float& minRange=0.5f);

void DepthImage2CvImage(const cv::Mat& depth_image_raw, sensor_msgs::ImagePtr& depth_msg);

void Image8u2Indices(const cv::Mat& image_8u, pcl::PointIndices::Ptr indices_ptr);

void Indices2Image8u(const std::vector<int>& indices, cv::Mat& image_8u);

void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,double scale, const cv::Scalar& color);

void drawOpticalFlowModule(const cv::Mat& optical_flow, cv::Mat& optical_flow_module);

void fillNaNsCBF(const sensor_msgs::Image& color_img, const cv::Mat& depth_img, cv::Mat& filled_depth_img, ros::Time query_time);

void removeDuplicateIndices(std::vector<int>& vec);

/*
% In-paints the depth image using a cross-bilateral filter. The operation
% is implemented via several filterings at various scales. The number of
% scales is determined by the number of spacial and range sigmas provided.
% 3 spacial/range sigmas translated into filtering at 3 scales.
%
% Args:
%   imgRgb - the RGB image, a uint8 HxWx3 matrix
%   imgDepthAbs - the absolute depth map, a HxW double matrix whose values
%                 indicate depth in meters.
%   spaceSigmas - (optional) sigmas for the spacial gaussian term.
%   rangeSigmas - (optional) sigmas for the intensity gaussian term.
%
% Returns:
%    imgDepthAbs - the inpainted depth image.
*/
cv::Mat fill_depth_cbf(cv::Mat imgRgb, cv::Mat imgDepthAbs, double* spaceSigmas, double* rangeSigmas);

// Filters the given depth image using a Cross Bilateral Filter.
//
// Args:
//   depth - HxW row-major ordered matrix.
//   intensity - HxW row-major ordered matrix.
//   mask - HxW row-major ordered matrix.
//   result - HxW row-major ordered matrix.
//   sigma_s - the space sigma (in pixels)
//   sigma_r - the range sigma (in intensity values, 0-1)
void cbf(uint8_t* depth, uint8_t* intensity, bool* mask, uint8_t* result, double* sigma_s, double* sigma_r);

}

#endif /*SR_UTILS_H_*/

