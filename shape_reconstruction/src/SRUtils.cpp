#include "shape_reconstruction/SRUtils.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "shape_reconstruction/RangeImagePlanar.hpp"

typedef unsigned char uint8_t;

namespace omip
{
void RangeImagePlanar2DepthMap(const pcl::RangeImagePlanar::Ptr& rip, cv::Mat& depth_map_mat)
{
    for(int i=0; i<IMG_WIDTH; i++)
    {
        for(int j=0; j<IMG_HEIGHT; j++)
        {
            depth_map_mat.at< float >(j,i)  = rip->points.at(j*IMG_WIDTH+i).z;
        }
    }
}

void RangeImagePlanar2PointCloud(const pcl::RangeImagePlanar::Ptr& rip, SRPointCloud::Ptr& pc)
{
    for(int i=0; i<IMG_WIDTH; i++)
    {
        for(int j=0; j<IMG_HEIGHT; j++)
        {
            pc->points.at(j*IMG_WIDTH+i).x  = rip->points.at(j*IMG_WIDTH+i).x;
            pc->points.at(j*IMG_WIDTH+i).y  = rip->points.at(j*IMG_WIDTH+i).y;
            pc->points.at(j*IMG_WIDTH+i).z  = rip->points.at(j*IMG_WIDTH+i).z;
        }
    }
}

void OrganizedPC2DepthMap(const SRPointCloud::Ptr organized_pc, cv::Mat& depth_map_mat)
{
    float* data_ptr = (float*)depth_map_mat.data;
    size_t elem_step = depth_map_mat.step / sizeof(float);

//    std::cout << organized_pc->height << std::endl;
//    std::cout << organized_pc->width << std::endl;
    for(int v=0; v< organized_pc->height; v++)
    {
        for(int u=0; u< organized_pc->width; u++)
        {
            if(pcl::isFinite(organized_pc->points[v*organized_pc->width + u]))
                data_ptr[v*elem_step + u] = organized_pc->points[v*organized_pc->width + u].z;
            else
                data_ptr[v*elem_step + u] = nanf("");
        }
    }
}

void OrganizedPC2DepthMapAlternative(const SRPointCloud::Ptr organized_pc, const cv::Ptr<cv::Mat>& depth_map_mat_ptr)
{
    float* data_ptr = (float*)depth_map_mat_ptr->data;
    size_t elem_step = depth_map_mat_ptr->step / sizeof(float);

    for(int v=0; v< organized_pc->height; v++)
    {
        for(int u=0; u< organized_pc->width; u++)
        {
            if(pcl::isFinite(organized_pc->points[v*organized_pc->width + u]))
                data_ptr[v*elem_step + u] = organized_pc->points[v*organized_pc->width + u].z;
            else
                data_ptr[v*elem_step + u] = nanf("");
        }
    }
}

void OrganizedPC2ColorMap(const SRPointCloud::Ptr organized_pc, cv::Mat& depth_map_mat)
{
    for (int h=0; h<depth_map_mat.rows; h++) {
        for (int w=0; w<depth_map_mat.cols; w++) {
            pcl::PointXYZRGB point = organized_pc->at(w, h);

            Eigen::Vector3i rgb = point.getRGBVector3i();

            depth_map_mat.at<cv::Vec3b>(h,w)[0] = rgb[2];
            depth_map_mat.at<cv::Vec3b>(h,w)[1] = rgb[1];
            depth_map_mat.at<cv::Vec3b>(h,w)[2] = rgb[0];
        }
    }
}

void UnorganizedPC2DepthMap(const SRPointCloud::Ptr& organized_pc,
                            const int& width,
                            const int& height,
                            cv::Mat& depth_map,
                            ::shape_reconstruction::RangeImagePlanar::Ptr& rip,
                            const float& noiseLevel,
                            const float& minRange) {
    Eigen::Affine3f sensor_pose;
    sensor_pose.matrix() = Eigen::Matrix4f::Identity(); // this->_current_HTransform_inv.cast<float>();
    pcl::RangeImagePlanar::CoordinateFrame coordinate_frame = pcl::RangeImagePlanar::CAMERA_FRAME;

    rip->createFromPointCloudWithFixedSizeAndStorePoints(*organized_pc,
                                                         width,
                                                         height,
                                                         width/2 -0.5, //319.5
                                                         height/2 - 0.5, // 239.5,
                                                         525, 525,
                                                         sensor_pose,
                                                         coordinate_frame,
                                                         noiseLevel,
                                                         minRange);

    RangeImagePlanar2DepthMap(rip, depth_map);
}

void DepthImage2CvImage(const cv::Mat& depth_image_raw, sensor_msgs::ImagePtr& depth_msg) {
    cv::Mat depth_image(depth_image_raw.rows, depth_image_raw.cols, CV_8UC1);
    cv::convertScaleAbs(depth_image_raw, depth_image, 100, 0);
    depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_image).toImageMsg();
}

void Image8u2Indices(const cv::Mat& image_8u, pcl::PointIndices::Ptr indices_ptr)
{
    int total_count = 0;
    int indices_count = 0;
    indices_ptr->indices.clear();
    for(int j=0; j<image_8u.rows; j++)
    {
        for(int i=0; i<image_8u.cols; i++)
        {
            if(image_8u.at<uchar>(j,i))
            {
                indices_ptr->indices.push_back(j*image_8u.cols + i);
                indices_count++;
            }
            total_count++;
        }
    }
    //std::cout << indices_count << " points of the " << total_count << " detected as candidates." << std::endl;
}

void Indices2Image8u(const std::vector<int>& indices, cv::Mat& image_8u)
{
    image_8u.setTo(0);
    for(int i=0; i<indices.size() ; i++)
    {
        image_8u.at<uchar>(floor(indices.at(i)/image_8u.cols),indices.at(i)%image_8u.cols) = 255;
    }
    //std::cout << indices_count << " points of the " << total_count << " detected as candidates." << std::endl;
}

void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
                    double scale, const cv::Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, cv::Point(x,y), 2, color, -1);
        }
}

void drawOpticalFlowModule(const cv::Mat& optical_flow, cv::Mat& optical_flow_module)
{
    if((optical_flow_module.cols != optical_flow.cols) || (optical_flow_module.rows != optical_flow.rows))
    {
        ROS_ERROR_STREAM_NAMED("drawOpticalFlowModule", "The sizes of the optical flow and the optical flow module images are different!");
        return;
    }

    for(int j = 0; j < optical_flow_module.rows; j++)
    {
        for(int i = 0; i < optical_flow_module.cols; i++)
        {
            const cv::Point2f& flow_ij = optical_flow.at<cv::Point2f>(j, i);
            optical_flow_module.at<float>(j, i) = std::sqrt(flow_ij.x*flow_ij.x + flow_ij.y*flow_ij.y);
        }
    }
}

cv::Mat filled_img_full;
ros::Time previous_query_time;

void fillNaNsCBF(const sensor_msgs::Image& color_img, const cv::Mat& depth_img, cv::Mat& filled_depth_img, ros::Time query_time)
{
    if(query_time == previous_query_time)
    {
        filled_depth_img = filled_img_full;
    }else{
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(color_img, color_img.encoding);

        cv::Mat cv_gray;
        cv::cvtColor(cv_ptr->image, cv_gray, CV_RGB2GRAY);

        // NOTE: CBF not only fills the NaNs but also modifies the not NaN values
        // We do not want this: we want to keep the measured depth for the not NaN values!
        filled_img_full = fill_depth_cbf(cv_gray, depth_img, 0, 0);

        depth_img.copyTo(filled_depth_img);
        filled_img_full.copyTo(filled_depth_img, depth_img!=depth_img);

        previous_query_time = query_time;
    }
}

void removeDuplicateIndices(std::vector<int>& vec) {
    std::set<int> s;
    unsigned size = vec.size();
    for( unsigned i = 0; i < size; ++i ) s.insert( vec[i] );
    vec.assign( s.begin(), s.end() );
}

// Uncomment this define for intermediate filtering results.
// #define DEBUG

#define PI 3.14159

#define MAX_UCHAR 255

// Make sure these are consistent!!
#define FILTER_RAD 5
#define FILTER_LEN 11
#define FILTER_SIZE 121

void toc(const char* message, clock_t start) {
#ifdef DEBUG
    double d = clock() - start;
    d = 1000 * d / CLOCKS_PER_SEC;
    printf("[%s] %10.0f\n", message, d);
#endif
}

// Args:
//   filter_size - the number of pixels in the filter.
void create_offset_array(int* offsets_h, int img_height) {
    int kk = 0;
    for (int yy = -FILTER_RAD; yy <= FILTER_RAD; ++yy) {
        for (int xx = -FILTER_RAD; xx <= FILTER_RAD; ++xx, ++kk) {
            offsets_h[kk] = yy + img_height * xx;
        }
    }
}

void calc_pyr_sizes(int* heights, int* widths, int* pyr_offsets) {
    int offset = 0;
    for (int scale = 0; scale < NUM_SCALES; ++scale) {
        pyr_offsets[scale] = offset;

        // Calculate the size of the downsampled images.
        heights[scale] = static_cast<int>(IMG_HEIGHT / pow((float)2, scale));
        widths[scale] = static_cast<int>(IMG_WIDTH / pow((float)2, scale));
        offset += heights[scale] * widths[scale];
    }

#ifdef DEBUG
    for (int ii = 0; ii < NUM_SCALES; ++ii) {
        printf("Scale %d: [%d x %d], offset=%d\n", ii, heights[ii], widths[ii], pyr_offsets[ii]);
    }
#endif
}

int get_pyr_size(int* heights, int* widths) {
    int total_pixels = 0;
    for (int ii = 0; ii < NUM_SCALES; ++ii) {
        total_pixels += heights[ii] * widths[ii];
    }
    return total_pixels;
}

// We're upsampling from the result matrix (which is small) to the depth matrix,
// which is larger.
//
// For example, dst could be 480x640 and src may be 240x320.
//
// Args:
//   depth_dst - H1xW1 matrix where H1 and W1 are equal to height_dst and
//               width_dst.
void upsample_cpu(float* depth_dst,
                  bool* mask_dst,
                  bool* valid_dst,
                  float* depth_src,
                  float* result_src,
                  bool* mask_src,
                  bool* valid_src,
                  int height_src,
                  int width_src,
                  int height_dst,
                  int width_dst,
                  int dst_img_ind) {

    int num_threads = height_dst * width_dst;

    // Dont bother if the upsampled one isnt missing.
    if (!mask_dst[dst_img_ind]) {
        return;
    }

    int x_dst = static_cast<int>(floorf((float) dst_img_ind / height_dst));
    int y_dst = static_cast<int>(fmodf(dst_img_ind, height_dst));

    int y_src = static_cast<int>((float) y_dst * height_src / height_dst);
    int x_src = static_cast<int>((float) x_dst * width_src / width_dst);

    // Finally, convert to absolute coords.
    int src_img_ind = y_src + height_src * x_src;

    if (!mask_src[src_img_ind]) {
        depth_dst[dst_img_ind] = depth_src[src_img_ind];
    } else {
        depth_dst[dst_img_ind] = result_src[src_img_ind];
    }

    valid_dst[dst_img_ind] = valid_src[src_img_ind];
}

// Args:
//   depth - the depth image, a HxW vector
//   intensity - the intensity image, a HxW vector.
//   is_missing - a binary mask specifying whether each pixel is missing
//                (and needs to be filled in) or not.
//   valid_in - a mask specifying which of the input values are allowed
//              to be used for filtering.
//   valid_out - a mask specifying which of the output values are allowed
//               to be used for future filtering.
//   result - the result of the filtering operation, a HxW matrix.
//   abs_inds - the absolute indices (into depth, intensity, etc) which
//              need filtering.
//   offsets - vector of offsets from the current abs_ind to be used for
//             filtering.
//   guassian - the values (weights) of the gaussian filter corresponding
//              to the offset matrix.
void cbf_cpu(const float* depth, const float* intensity, bool* is_missing,
             bool* valid_in, bool* valid_out, float* result,
             const int* abs_inds,
             const int* offsets,
             const float* gaussian_space,
             int height,
             int width,
             float sigma_s,
             float sigma_r,
             int numThreads,
             int idx) {

    int abs_ind = abs_inds[idx];

    int src_Y = abs_ind % height;
    int src_X = abs_ind / height;

    float weight_sum = 0;
    float value_sum = 0;

    float weight_intensity_sum = 0;

    float gaussian_range[FILTER_SIZE];
    float gaussian_range_sum = 0;

    for (int ii = 0; ii < FILTER_SIZE; ++ii) {
        // Unfortunately we need to double check that the radii are correct
        // unless we add better processing of borders.

        int abs_offset = abs_ind + offsets[ii]; // THESE ARE CALC TWICE.

        int dst_Y = abs_offset % height;
        int dst_X = abs_offset / height;

        if (abs_offset < 0 || abs_offset >= (height * width)
                || abs(src_Y-dst_Y) > FILTER_RAD || abs(src_X-dst_X) > FILTER_RAD) {
            continue;

            // The offsets are into ANY part of the image. So they MAY be accessing
            // a pixel that was originally missing. However, if that pixel has been
            // filled in, then we can still use it.
        } else if (is_missing[abs_offset] && !valid_in[abs_offset]) {
            continue;
        }

        float vv = intensity[abs_offset] - intensity[abs_ind];

        gaussian_range[ii] = exp(-(vv * vv) / (2*sigma_r * sigma_r));
        gaussian_range_sum += gaussian_range[ii];
    }

    int count = 0;

    for (int ii = 0; ii < FILTER_SIZE; ++ii) {
        // Get the Absolute offset into the image (1..N where N=H*W)
        int abs_offset = abs_ind + offsets[ii];
        int dst_Y = abs_offset % height;
        int dst_X = abs_offset / height;
        if (abs_offset < 0 || abs_offset >= (height * width)
                || abs(src_Y-dst_Y) > FILTER_RAD || abs(src_X-dst_X) > FILTER_RAD) {
            continue;
        } else if (is_missing[abs_offset] && !valid_in[abs_offset]) {
            continue;
        }

        ++count;

        weight_sum += gaussian_space[ii] * gaussian_range[ii];
        value_sum += depth[abs_offset] * gaussian_space[ii] * gaussian_range[ii];
    }

    if (weight_sum == 0) {
        return;
    }

    if (std::isnan(weight_sum)) {
        printf("*******************\n");
        printf(" Weight sum is NaN\n");
        printf("*******************\n");
    }

    value_sum /= weight_sum;
    result[abs_ind] = value_sum;
    valid_out[abs_ind] = 1;
}

// Args:
//   filter_size - the number of pixels in the filter.
void create_spatial_gaussian(float sigma_s, float* gaussian_h) {
    float sum = 0;
    int kk = 0;
    for (int yy = -FILTER_RAD; yy <= FILTER_RAD; ++yy) {
        for (int xx = -FILTER_RAD; xx <= FILTER_RAD; ++xx, ++kk) {
            gaussian_h[kk] = exp(-(xx*xx + yy*yy) / (2*sigma_s * sigma_s));
            sum += gaussian_h[kk];
        }
    }

    for (int ff = 0; ff < FILTER_SIZE; ++ff) {
        gaussian_h[ff] /= sum;
    }
}

// Counts the number of missing pixels in the given mask. Note that the mask
// MUST already be in the appropriate offset location.
//
// Args:
//   height - the heigh of the image at the current scale.
//   width - the width of the image at the current scale.
//   mask - pointer into the mask_ms_d matrix. The offset has already been
//          calculated.
//   abs_inds_h - pre-allocated GPU memory location.
int get_missing_pixel_coords(int height, int width, bool* mask, int* abs_inds_to_filter_h) {
    int num_pixels = height * width;

    int num_missing_pixels = 0;
    for (int nn = 0; nn < num_pixels; ++nn) {
        if (mask[nn]) {
            abs_inds_to_filter_h[num_missing_pixels] = nn;
            ++num_missing_pixels;
        }
    }

    return num_missing_pixels;
}

static void savePGM(bool* imf, const char *name, int height, int width) {
    uint8_t im[IMG_HEIGHT * IMG_WIDTH];

    for (int nn = 0; nn < IMG_HEIGHT * IMG_HEIGHT; ++nn) {
        // First convert to X,Y
        int y = nn % height;
        int x = static_cast<int>(floor(nn / static_cast<double>(height)));

        // Then back to Abs Inds
        int mm = y * width + x;
        im[mm] = uint8_t(255*imf[nn]);
    }

    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "P5\n" << width << " " << height << "\n" << MAX_UCHAR << "\n";
    file.write((char *)&im, width * height * sizeof(uint8_t));
}

static void savePGM(float* imf, const char *name, int height, int width) {
    uint8_t im[IMG_HEIGHT * IMG_WIDTH];

    for (int nn = 0; nn < IMG_HEIGHT * IMG_WIDTH; ++nn) {
        // First convert to X,Y
        int y = nn % height;
        int x = static_cast<int>(floor(nn / static_cast<double>(height)));

        // Then back to Abs Inds
        int mm = y * width + x;

        im[mm] = uint8_t(255*imf[nn]);
    }

    std::ofstream file(name, std::ios::out | std::ios::binary);

    file << "P5\n" << width << " " << height << "\n" << MAX_UCHAR << "\n";
    file.write((char *)&im, width * height * sizeof(uint8_t));
}

void filter_at_scale(float* depth_h,
                     float* intensity_h,
                     bool* mask_h,
                     bool* valid_h,
                     float* result_h,
                     int* abs_inds_to_filter_h,
                     int height,
                     int width,
                     float sigma_s,
                     float sigma_r) {

    // Create the offset array.
    int* offsets_h = (int*) malloc(FILTER_SIZE * sizeof(int));
    create_offset_array(offsets_h, height);

    // Create the gaussian.
    float* gaussian_h = (float*) malloc(FILTER_SIZE * sizeof(float));
    create_spatial_gaussian(sigma_s, gaussian_h);

    // ************************************************
    // We need to be smart about how we do this, so rather
    // than execute the filter for EVERY point in the image,
    // we will only do it for the points missing depth information.
    // ************************************************

    int num_missing_pixels = get_missing_pixel_coords(height, width, mask_h, abs_inds_to_filter_h);
    printf("[Depth filling with CBF. INFO] Num NaN depth values at this resolution scale: %d\n", num_missing_pixels);

    clock_t start_filter = clock();

    // We should not be writing into the same value for 'valid' that we're passing in.
    bool* valid_in = (bool*) malloc(height * width * sizeof(bool));
    for (int i = 0; i < height * width; ++i) {
        valid_in[i] = valid_h[i];
    }

    for (int i = 0; i < num_missing_pixels; ++i) {
        cbf_cpu(depth_h,
                intensity_h,
                mask_h,
                valid_in,
                valid_h,
                result_h,
                abs_inds_to_filter_h,
                offsets_h,
                gaussian_h,
                height,
                width,
                sigma_s,
                sigma_r,
                num_missing_pixels,
                i);
    }

    toc("FILTER OP", start_filter);

    free(valid_in);
    free(offsets_h);
    free(gaussian_h);
}

void cbf(uint8_t* depth, uint8_t* intensity, bool* mask_h, uint8_t* result, double* sigma_s, double* sigma_r)
{

    clock_t start_func = clock();

    int pyr_heights[NUM_SCALES];
    int pyr_widths[NUM_SCALES];
    int pyr_offsets[NUM_SCALES];
    calc_pyr_sizes(&pyr_heights[0], &pyr_widths[0], &pyr_offsets[0]);

    // Allocate the memory needed for the absolute missing pixel indices. We'll
    // allocate the number of bytes required for the largest image, since the
    // smaller ones obviously fit inside of it.
    int N = IMG_HEIGHT * IMG_WIDTH;
    int* abs_inds_to_filter_h = (int*) malloc(N * sizeof(int));

    int pyr_size = get_pyr_size(&pyr_heights[0], &pyr_widths[0]);

    // ************************
    //   CREATING THE PYRAMID
    // ************************
    clock_t	start_pyr = clock();

    // NEG TIME.
    float* depth_ms_h = (float*) malloc(pyr_size * sizeof(float));
    float* intensity_ms_h = (float*) malloc(pyr_size * sizeof(float));
    bool* mask_ms_h = (bool*) malloc(pyr_size * sizeof(bool));
    float* result_ms_h = (float*) malloc(pyr_size * sizeof(float));
    bool* valid_ms_h = (bool*) malloc(pyr_size * sizeof(bool));

    for (int nn = 0; nn < N; ++nn)
    {
        depth_ms_h[nn] = depth[nn] / 255.0f;
        intensity_ms_h[nn] = intensity[nn] / 255.0f;
        mask_ms_h[nn] = mask_h[nn];
        valid_ms_h[nn] = !mask_h[nn];
        result_ms_h[nn] = 0;
    }

    float* depth_ms_h_p = depth_ms_h + pyr_offsets[1];
    float* intensity_ms_h_p	= intensity_ms_h + pyr_offsets[1];
    bool* mask_ms_h_p	= mask_ms_h + pyr_offsets[1];
    bool* valid_ms_h_p = valid_ms_h + pyr_offsets[1];
    float* result_ms_h_p = result_ms_h + pyr_offsets[1];

    for (unsigned int scale = 1; scale < NUM_SCALES; ++scale) {
        for (int xx = 0; xx < pyr_widths[scale]; ++xx) {
            for (int yy = 0; yy < pyr_heights[scale]; ++yy, ++depth_ms_h_p, ++intensity_ms_h_p, ++mask_ms_h_p, ++result_ms_h_p, ++valid_ms_h_p) {
                int abs_yy = static_cast<int>(((float)yy / pyr_heights[scale]) * IMG_HEIGHT);
                int abs_xx = static_cast<int>(((float)xx / pyr_widths[scale]) * IMG_WIDTH);
                int img_offset = abs_yy + IMG_HEIGHT * abs_xx;
                *depth_ms_h_p = depth_ms_h[img_offset];
                *intensity_ms_h_p = intensity_ms_h[img_offset];
                *mask_ms_h_p = mask_h[img_offset];
                *valid_ms_h_p = !mask_h[img_offset];
                *result_ms_h_p = 0;
            }
        }
    }

    // *********************************
    //   RUN THE ACTUAL FILTERING CODE
    // *********************************

    for (int scale = NUM_SCALES - 1; scale >= 0; --scale) {
        //printf("Filtering at scale %d, [%dx%d]\n", scale, pyr_heights[scale], pyr_widths[scale]);

#ifdef DEBUG
        char filename1[50];
        sprintf(filename1, "missing_pixels_before_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(mask_ms_h + pyr_offsets[scale], filename1, pyr_heights[scale], pyr_widths[scale]);

        char filename2[50];
        sprintf(filename2, "valid_pixels_before_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(valid_ms_h + pyr_offsets[scale], filename2, pyr_heights[scale], pyr_widths[scale]);

        sprintf(filename2, "valid_intensity_before_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(intensity_ms_h + pyr_offsets[scale], filename2, pyr_heights[scale], pyr_widths[scale]);

        sprintf(filename2, "depth_before_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(depth_ms_h + pyr_offsets[scale], filename2, pyr_heights[scale], pyr_widths[scale]);
#endif

        filter_at_scale(depth_ms_h + pyr_offsets[scale],
                        intensity_ms_h + pyr_offsets[scale],
                        mask_ms_h + pyr_offsets[scale],
                        valid_ms_h + pyr_offsets[scale],
                        result_ms_h + pyr_offsets[scale],
                        abs_inds_to_filter_h,
                        pyr_heights[scale],
                        pyr_widths[scale],
                        sigma_s[scale],
                        sigma_r[scale]);


#ifdef DEBUG
        sprintf(filename2, "valid_pixels_after_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(valid_ms_h + pyr_offsets[scale], filename2, pyr_heights[scale], pyr_widths[scale]);
#endif

#ifdef DEBUG
        char filename[50];
        sprintf(filename, "depth_after_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(result_ms_h + pyr_offsets[scale], filename, pyr_heights[scale], pyr_widths[scale]);
#endif

        if (scale == 0) {
            continue;
        }

        // Now, we need to upsample the resulting depth and store it in the next
        // highest location.
        int num_missing_pixels = pyr_heights[scale-1] * pyr_widths[scale-1];

        //printf("Upsampling %d\n", num_missing_pixels);
        for (int i = 0; i < num_missing_pixels; ++i) {
            upsample_cpu(depth_ms_h + pyr_offsets[scale-1],
                    mask_ms_h + pyr_offsets[scale-1],
                    valid_ms_h + pyr_offsets[scale-1],
                    depth_ms_h + pyr_offsets[scale],
                    result_ms_h + pyr_offsets[scale],
                    mask_ms_h + pyr_offsets[scale],
                    valid_ms_h + pyr_offsets[scale],
                    pyr_heights[scale],
                    pyr_widths[scale],
                    pyr_heights[scale-1],
                    pyr_widths[scale-1],
                    i);
        }


#ifdef DEBUG
        sprintf(filename, "up_depth_after_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(depth_ms_h + pyr_offsets[scale-1], filename, pyr_heights[scale-1], pyr_widths[scale-1]);

        sprintf(filename, "up_valid_after_filtering_scale%d.pgm", scale);
        // Now that we've performed the filtering, save the intermediate image.
        savePGM(valid_ms_h + pyr_offsets[scale-1], filename, pyr_heights[scale-1], pyr_widths[scale-1]);
#endif
    }

    // Copy the final result from the device.
    for (int nn = 0; nn < N; ++nn) {
        if (mask_ms_h[nn]) {
            result[nn] = static_cast<uint8_t>(result_ms_h[nn] * 255);
        } else {
            result[nn] = depth[nn];
        }
    }

    free(depth_ms_h);
    free(intensity_ms_h);
    free(mask_ms_h);
    free(result_ms_h);
    free(valid_ms_h);
    free(abs_inds_to_filter_h);

    toc("Entire Function", start_func);
}

cv::Mat fill_depth_cbf(cv::Mat imgRgb, cv::Mat imgDepthAbs, double* spaceSigmas, double* rangeSigmas)
{
    double spaceSigmas_default[3];
    spaceSigmas_default[0] = 12;
    spaceSigmas_default[1] = 5;
    spaceSigmas_default[2] = 8;

    double rangeSigmas_default[3];
    rangeSigmas_default[0] = 0.2;
    rangeSigmas_default[1] = 0.08;
    rangeSigmas_default[2] = 0.02;

    // Create the 'noise' image and get the maximum observed depth.
    cv::Mat imgIsNoise = imgDepthAbs != imgDepthAbs;
    cv::Mat imgIsNotNoise = imgDepthAbs == imgDepthAbs;

    double min, max;
    cv::minMaxLoc(imgDepthAbs, &min, &max, 0, 0, imgIsNotNoise);

    // Convert the depth image to uint8.
    cv::Mat imgDepth = (255/max) * imgDepthAbs;
    imgDepth.setTo(255, imgDepth >255);

    cv::Mat img_depth_8u;
    imgDepth.convertTo(img_depth_8u, CV_8U);

    cv::Mat result_8u;
    img_depth_8u.copyTo(result_8u);

    bool* isnoise = (bool*) malloc(IMG_WIDTH*IMG_HEIGHT * sizeof(bool));

    for(int k=0; k<imgIsNoise.cols; k++)
    {
        for(int j=0; j<imgIsNoise.rows; j++)
        {
            if(imgIsNoise.at<uchar>(j,k) == 0)
            {
                isnoise[j*imgIsNoise.cols + k] = false;
            }else{
                isnoise[j*imgIsNoise.cols + k] = true;
            }
        }
    }

    // Run the cross-bilateral filter.
    cbf(img_depth_8u.data, imgRgb.data, isnoise, result_8u.data, spaceSigmas_default, rangeSigmas_default);

    cv::Mat result_double;
    result_8u.convertTo(result_double, CV_32FC1);
    result_double = result_double*(max/255.0);

    return result_double;
}

}
