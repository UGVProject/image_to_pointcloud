// flea3Driver.h
// Created by Zhang Handuo on 08/04/16.
//

#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cmath>
//#include <vector>
#include <libsgm.h>
#include <string>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/common/common.h>
#include <pcl_ros/point_cloud.h>


class stereosgm {
public:
    stereosgm();
    virtual ~stereosgm();
    // bool init(const uint64_t cameraUid );

    void StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat Q, const double &timestamp);
    void visualizer(cv::Mat &cloud);
    bool update;
    // pcl::PointXYZRGB basic_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr;
private:

    double shutter_speed;
    int mHeight;
    int mWidth;
    cv::Mat left, right, left_color;
    int disp_size;

    std::string type2str(int type);

    bool compare_mean(const cv::Point3i &a, const cv::Point3i &b);

    bool compare_row(const cv::Point3i &a, const cv::Point3i &b);

    bool compare_col(const cv::Point3i &a, const cv::Point3i &b);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

};
