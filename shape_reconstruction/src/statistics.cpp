#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <shape_reconstruction/SRUtils.h>

#include <opencv2/highgui/highgui.hpp>

#include <regex>

using namespace std;
using namespace boost::filesystem;

namespace omip {


void compareAlignedDensePointClouds(SRPointCloud::Ptr& gt_pc, SRPointCloud::Ptr& seg_pc, int& tp, int& tn, int& fp, int& fn, int& error) {
    tp=0, tn=0, fp=0, fn=0, error=0;

    // TODO compare to ground truth
    if (gt_pc->size() != seg_pc->size()) {
        ROS_ERROR(" Cannot compare; sizes differ!");
        return;
    } else {
        for (int i = 0; i < gt_pc->size(); i++) {
            SRPoint& p_gt =  gt_pc->points[i];
            SRPoint& p_seg =  seg_pc->points[i];

            bool gt_nan = !(!isnan(p_gt.x) && !isnan(p_gt.y) && !isnan(p_gt.z));
            bool seg_nan = !(!isnan(p_seg.x) && !isnan(p_seg.y) && !isnan(p_seg.z));

            if (gt_nan && seg_nan)
                tn++;
            else if (gt_nan && !seg_nan)
                fp++;
            else if (!gt_nan && seg_nan)
                fn++;
            else if ( p_gt.x == p_seg.x && p_gt.y == p_seg.y && p_gt.z == p_seg.z )
                tp++;
            else
                error++;
        }
    }

    if (error>0){
        ROS_ERROR_STREAM(endl << "          ERROR non matching points: " << error << " - are the point clouds really aligned?");
    }
}

std::string getTimeStampString(const ros::Time& t) {
    double sec = t.toSec();

    char c[255];
    sprintf(c, "%.3f\n", sec);

    return string(c);
}

void writeCSV( std::ofstream& out, std::vector<std::string>& labels, std::vector<std::vector<double> >& values ) {
    assert (out.is_open());

    out.precision(25);

    out << "#";
    for (vector<string >::iterator it = labels.begin(); it != labels.end(); it++) {
        out << *it << " ";
    }
    out << endl;

    for (vector<vector<double> >::iterator it = values.begin(); it != values.end(); it++) {
        for (vector<double>::iterator it2 = it->begin(); it2 != it->end(); it2++) {
            out << *it2 << " ";
        }
        out << endl;
    }
}

void get_ochs_result_file_list(const string& ochs_result_dir, vector<string>& dense_recons) {
    cout << "Listing Ochs results:  " << ochs_result_dir << endl;
    path ochs_dir(ochs_result_dir);
    directory_iterator end_iter;    

    //vector<path> dense_recons;
    dense_recons.clear();

    if ( !exists(ochs_dir) || !is_directory(ochs_dir)) {
        cout << "ERROR: directory does not exist" << endl;
        return;
    }
    for( directory_iterator dir_iter(ochs_dir) ; dir_iter != end_iter ; ++dir_iter) {
        if (!is_regular_file(dir_iter->status()) ) {
            continue;
        }
        //const directory_entry& entry = *dir_iter;
        const path& p = dir_iter->path();
        if (p.leaf().string().find("dense") != string::npos) {
            dense_recons.push_back(p.string());
        }
    }

    std::sort(dense_recons.begin(), dense_recons.end());

    for (int i = 0; i < dense_recons.size(); i++) {
        cout << "   " << dense_recons[i] << endl;
    }

}


void getOchsSegments(const cv::Mat img, SRPointCloud::ConstPtr orig_pc,  vector<SRPointCloud::Ptr>& segments) {
    // collect colors
    segments.clear();

    map<string,SRPointCloud::Ptr> colors;
    int idx=0;
    for (int h=0; h<img.rows; h++) {
        for (int w=0; w<img.cols; w++) {
            cv::Vec3b c = img.at<cv::Vec3b>(h,w);
            ostringstream hash;
            hash << static_cast<int>(c[0]) << static_cast<int>(c[1]) << static_cast<int>(c[2]);
            map<string,SRPointCloud::Ptr>::iterator it = colors.find(hash.str());
            if (it == colors.end()) {
                cout << "create new point cloud #" << colors.size() << " for hash " << hash.str() << endl;
                SRPointCloud::Ptr pc(new SRPointCloud);
                pc->width = img.cols;
                pc->height = img.rows;
                for (int h=0; h<img.rows; h++) {
                    for (int w=0; w<img.cols; w++) {
                        SRPoint po; po.data[0] = po.data[1] = po.data[2] = std::numeric_limits<float>::quiet_NaN();
                        pc->points.push_back(po);
                    }
                }

//                cout << pcl::isFinite(pc->points[0]) << endl;
//                cout << "orig_pc: " << orig_pc->width << ", " << orig_pc->height << " (" << orig_pc->points.size() << ")" << endl;
//                cout << orig_pc->points.at(0) << endl;
//                cout << "pc: " << pc->width << ", " << pc->height << " (" << pc->points.size() << ")" << endl;

                colors[hash.str()] = pc;
            }
            SRPointCloud::Ptr pc = colors[hash.str()];
            SRPoint &p = pc->points.at(idx);
            p = orig_pc->points.at(idx);

            idx++;
        }
    }

    for (map<string,SRPointCloud::Ptr>::iterator it = colors.begin(); it != colors.end(); it++) {
        segments.push_back(it->second);
    }

}

void process(rosbag::Bag& results_bag, rosbag::Bag& gt_bag, const path& output_dir, const string& ochs_result_dir="") {
    static const int rb_max = 5;

    vector<string> labels;
    labels.push_back("time");
    labels.push_back("tp");
    labels.push_back("tn");
    labels.push_back("fp");
    labels.push_back("fn");

    map<string, vector<vector<double> > > results;

    // find topics
    std::vector<std::string> topics;

    topics.push_back("/camera/depth_registered/points");
    topics.push_back("/real_selected_points");

    std::vector<std::string> segment_topics;
    segment_topics.push_back("/shape_recons/segment_rb");
    segment_topics.push_back("/shape_recons/segment_depth_rb");
    segment_topics.push_back("/shape_recons/segment_color_rb");
    segment_topics.push_back("/shape_recons/segment_ext_d_and_c_rb");
    segment_topics.push_back("/shape_recons/segment_ext_d_rb");
    segment_topics.push_back("/shape_recons/segment_ext_c_rb");

    segment_topics.push_back("/shape_recons/segment_naive_rb");
    segment_topics.push_back("/shape_recons/segment_naive_depth_rb");
    segment_topics.push_back("/shape_recons/segment_naive_color_rb");

    for (vector<string>::iterator it = segment_topics.begin(); it != segment_topics.end(); it++) {
        for (int i = 2; i < rb_max; i++) {
            stringstream ss;
            ss << *it << i;
            topics.push_back(ss.str());

            cout << "       adding topic " << ss.str() << endl;
        }
    }
    cout << endl;

    rosbag::View view_results(results_bag, rosbag::TopicQuery(topics));
    rosbag::View view_gt(gt_bag, rosbag::TopicQuery(topics));


    vector<rosbag::View::const_iterator> gt_pc_list;
    vector<rosbag::View::const_iterator> res_full_pc_list;
    map< string, vector<rosbag::View::const_iterator> > res_seg_pc_map;

    // collect
    cout << "COLLECTING ground truth " << endl;
    rosbag::View::const_iterator git = view_gt.begin();
    for (; git != view_gt.end(); git++) {
        // get ground truth point cloud
        rosbag::MessageInstance const& m = *git;
        assert (m.getTopic() == "/real_selected_points");

        sensor_msgs::PointCloud2ConstPtr gt_msg = m.instantiate<sensor_msgs::PointCloud2>();
        cout << "    found gt at time: " << gt_msg->header.stamp << endl;
        gt_pc_list.push_back(git);
    }
    cout << endl;

    cout << "COLLECTING segmentation results" << endl;
    rosbag::View::const_iterator rit = view_results.begin();
    for (; rit != view_results.end(); rit++) {
        // get ground truth point cloud
        rosbag::MessageInstance const& m = *rit;
        if (m.getTopic() == "/camera/depth_registered/points") {
            sensor_msgs::PointCloud2ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
            res_full_pc_list.push_back(rit);
            cout << " Found PC at time      " << pc_msg->header.stamp << endl;
        } else {
            sensor_msgs::PointCloud2ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
            res_seg_pc_map[getTimeStampString(pc_msg->header.stamp)].push_back(rit);
            cout << " Found segment " << m.getTopic() << " at time " << pc_msg->header.stamp << endl;

            SRPointCloud::Ptr seg_pc(new SRPointCloud);
            rosbag::MessageInstance const& m = *rit;
            sensor_msgs::PointCloud2ConstPtr seg_msg = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*seg_msg, *seg_pc);

//            SRPointCloud::Ptr seg_pc_no_nan(new SRPointCloud);
//            std::vector<int> not_nan_indices;
//            pcl::removeNaNFromPointCloud<SRPoint>(*seg_pc,*seg_pc_no_nan, not_nan_indices);
//            cout << "   Segment contains " << seg_pc_no_nan->size() << " not-nan points" << endl;

        }
    }

    cout << "SUMMARY" << endl;
    cout << "  ground truth:   " << gt_pc_list.size() << " full point clouds " << endl;
    cout << "      segments:   " << res_full_pc_list.size() << " full point clouds " << endl;
    cout << "                  " << res_seg_pc_map.size() << " entries " << endl;

    cout << "---------------------------" << endl << endl;

    if (gt_pc_list.size() != res_full_pc_list.size()) {
        ROS_WARN_STREAM("Ground truth and result bag have different sizes!");
        if (gt_pc_list.size() > res_full_pc_list.size())
            ROS_WARN_STREAM("Assuming results contain a subset, but start at the same time step");
        else {
            ROS_ERROR_STREAM("Results bag has more than ground truth! aborting");
            return;
        }
    }


    // collect ochs results
    vector<string> ochs_results;
    if (ochs_result_dir != "") {
        get_ochs_result_file_list(ochs_result_dir, ochs_results);
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // now iterate over the two
    vector<rosbag::View::const_iterator>::iterator gvit = gt_pc_list.begin();
    vector<rosbag::View::const_iterator>::iterator rvit = res_full_pc_list.begin();
    int idx=0;

    for (; gvit != gt_pc_list.end() && rvit != res_full_pc_list.end(); gvit++, rvit++, idx++) {
        // get ground truth PC
        rosbag::MessageInstance const& m_gt = **gvit;
        SRPointCloud::Ptr gt_pc_with_nan(new SRPointCloud);
        sensor_msgs::PointCloud2ConstPtr gt_msg = m_gt.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*gt_msg, *gt_pc_with_nan);

//        SRPointCloud::Ptr gt_pc;
//        gt_pc.reset(new SRPointCloud);
//        std::vector<int> not_nan_indices;
//        pcl::removeNaNFromPointCloud<SRPoint>(*gt_pc_with_nan,*gt_pc, not_nan_indices);
//        cout << "Ground truth contains " << gt_pc->size() << " not-nan points" << endl;

        // get full scene PC
        rosbag::MessageInstance const& m_full = **rvit;
        sensor_msgs::PointCloud2ConstPtr full_msg = m_full.instantiate<sensor_msgs::PointCloud2>();

        cout << "TIME: " << full_msg->header.stamp << endl;
        cout << "    time ground truth: " << gt_msg->header.stamp << endl;

        // get ochs segments and take best
        if (!ochs_results.empty()) {
            cout << "   " <<  "Looking at Ochs result: " << idx << endl;

            SRPointCloud::Ptr full_pc(new SRPointCloud);
            pcl::fromROSMsg(*full_msg, *full_pc);

            cv::Mat ochs_result_current = cv::imread(ochs_results[idx]);
            vector<SRPointCloud::Ptr> ochs_segments;
            getOchsSegments(ochs_result_current, full_pc,  ochs_segments);

            // select best segment
            int best_ochs_segment = -1;
            double best_f_score = 0.;
            vector<double> res;
            for (int i= 0; i < ochs_segments.size(); i++) {
                int tp, tn, fp, fn, error;
                compareAlignedDensePointClouds(gt_pc_with_nan, ochs_segments[i], tp, tn, fp, fn, error);
                double precision=0.0, recall=0.0, fscore=0.0;

                precision = ((double)tp) / (tp+fp);
                recall = ((double)tp) / (tp+fn);
                fscore = 2* ((double) precision*recall)/(precision+recall);

                cout << "   " << "  ochs segment " << i << " p=" << precision << ", r=" << recall << ", f=" << fscore << endl;
                if (fscore>best_f_score) {
                    best_ochs_segment = i;
                    best_f_score = fscore;
                    cout << "     " << i << " is best so far" << endl;
                    res.clear();
                    res.push_back(full_msg->header.stamp.toSec());
                    res.push_back(tp); res.push_back(tn); res.push_back(fp); res.push_back(fn);
                }
            }
            results[ "ochs" ].push_back(res);

            // <debug>
            cv::imshow("ochs segmentation", ochs_result_current);
            for (int i= 0; i < ochs_segments.size(); i++) {
                stringstream nm;
                nm << "segment " << i;
                SRPointCloud::Ptr sgm_pc = ochs_segments[i];
                cv::Mat sgm_dm(480,640,CV_32FC1);
                OrganizedPC2DepthMap(sgm_pc, sgm_dm);
                cv::imshow(nm.str(), sgm_dm);
            }
            //cv::waitKey(-1);
            //cv::waitKey(50);
            // </debug>
        }


        // compare also SR segments to ground truth
        vector<rosbag::View::const_iterator>& segment_list = res_seg_pc_map[getTimeStampString(full_msg->header.stamp)];
        cout << " Segment list contains " << segment_list.size() << " points" << endl;
        if (segment_list.empty()) {
            ROS_WARN_STREAM("No segments at time " << full_msg->header.stamp);
            continue;
        }

        vector<rosbag::View::const_iterator>::iterator svit = segment_list.begin();
        for (; svit != segment_list.end(); svit++) {

            SRPointCloud::Ptr seg_pc(new SRPointCloud);
            rosbag::MessageInstance const& m_res = **svit;
            sensor_msgs::PointCloud2ConstPtr seg_msg = m_res.instantiate<sensor_msgs::PointCloud2>();
            pcl::fromROSMsg(*seg_msg, *seg_pc);

//            SRPointCloud::Ptr seg_pc_no_nan(new SRPointCloud);
//            std::vector<int> not_nan_indices;
//            pcl::removeNaNFromPointCloud<SRPoint>(*seg_pc,*seg_pc_no_nan, not_nan_indices);
//            cout << "   Segment contains " << seg_pc_no_nan->size() << " not-nan points" << endl;

            int tp,fp,tn,fn,error;
            compareAlignedDensePointClouds(gt_pc_with_nan, seg_pc, tp, tn, fp, fn, error);

            /*
            cv::Mat gt_mat(480, 640, CV_8UC3), seg_mat(480, 640, CV_8UC3);
            omip::OrganizedPC2ColorMap(gt_pc_with_nan, gt_mat);
            omip::OrganizedPC2ColorMap(seg_pc, seg_mat);

            cv::Mat gt_depth_mat(480, 640, CV_32FC1), seg_depth_mat(480, 640, CV_32FC1), seg_depth_mat_rev(480, 640, CV_32FC1);
            omip::OrganizedPC2DepthMap(gt_pc_with_nan, gt_depth_mat);
            omip::OrganizedPC2DepthMap(seg_pc, seg_depth_mat);
            omip::OrganizedPC2DepthMap(seg_pc, seg_depth_mat_rev);

            cv::imshow("Ground truth", gt_mat);
            cv::imshow(m_res.getTopic(), seg_mat);

            cv::imshow("Ground truth depth", gt_depth_mat);
            cv::imshow(m_res.getTopic()  + " depth", seg_depth_mat);
            cv::imshow(m_res.getTopic()  + " depth rev", seg_depth_mat_rev);
            cv::waitKey(-1);
            */

            cout << m_res.getTopic() << endl;
            cout << "  tp= " << tp;
            cout << "  tn= " << tn;
            cout << "  fp= " << fp;
            cout << "  fn= " << fn << endl;

            vector<double> res;
            res.push_back(seg_msg->header.stamp.toSec());
            res.push_back(tp); res.push_back(tn); res.push_back(fp); res.push_back(fn);
            results[ m_res.getTopic() ].push_back(res);
        }
    }


    for (map<string, vector<vector<double> > >::iterator it = results.begin(); it != results.end(); it++) {
        cout << "Writing out results for " << it->first << endl;
        cout << "  has " << it->second.size() << " entries" << endl;
        // prepare output file
        vector<string> strs;
        boost::split(strs, it->first,boost::is_any_of("/"));

        path output_path = output_dir;;
        output_path /=  strs[strs.size()-1] + ".txt";

        cout <<"  Output path: " << output_path.string() << endl;

        ofstream output;
        output.open(output_path.string().c_str());
        writeCSV(output, labels, it->second);
        output.close();
    }
}

}

// Main program
int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Usage: rosrun statistics <full_pc_and_segmentation> <ground_truth> [ochs_results_dir]" << endl;
        return 0;
    }

    path results_bag_path(argv[1]);
    if (!exists(results_bag_path.string())) {
        cerr << " results bag does not exist:  " << results_bag_path.string() << endl;
        return -1;
    }
    path gt_bag_path(argv[2]);
    if (!exists(gt_bag_path.string())) {
        cerr << " ground truth bag does not exist:  " << gt_bag_path.string() << endl;
        return -1;
    }

    string ochs_result_dir("");
    if (argc > 3) {
        ochs_result_dir = argv[3];
    }

    // prepare out path
    path output_dir = results_bag_path.parent_path();
    output_dir /= results_bag_path.stem().string();

    if (!boost::filesystem::exists(output_dir)) {
        if (!boost::filesystem::create_directories(output_dir)) {
            ROS_ERROR_NAMED("statistics", "Output directory for statistics does not exist and cannot be created!");
            return -1;
        }
    }

    // open rosbags
    rosbag::Bag results_bag;
    results_bag.open(results_bag_path.string(), rosbag::bagmode::Read);

    rosbag::Bag gt_bag;
    gt_bag.open(gt_bag_path.string(), rosbag::bagmode::Read);

    omip::process(results_bag, gt_bag, output_dir, ochs_result_dir);
    results_bag.close();
    gt_bag.close();

    return (0);
}
