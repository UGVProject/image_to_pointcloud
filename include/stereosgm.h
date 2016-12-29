// flea3Driver.h
// Created by Zhang Handuo on 08/04/16.
//

#include <cmath>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
//#include <vector>
#include <boost/thread/thread.hpp>
#include <libsgm.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <string>

class stereosgm {
public:
  stereosgm();
  virtual ~stereosgm();
  // bool init(const uint64_t cameraUid );

  void StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat Q,
                      const double &timestamp);
  void visualizer(cv::Mat &cloud);
  bool update;
  // pcl::PointXYZRGB basic_point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr;
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
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

  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

  void reprojectTo3D(cv::Mat &, cv::Mat &, bool);
};
