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
#include "../include/timer.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <image_to_pointcloud/MapInfo.h>

class stereosgm {

public:
  stereosgm();
  virtual ~stereosgm();
  // bool init(const uint64_t cameraUid );

  void StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat Q,
                      const double &timestamp);
//  void visualizer(cv::Mat &cloud);

  void reprojectTo3D(cv::Mat &, cv::Mat &, bool, const image_to_pointcloud::MapInfoConstPtr& info);

//  void MapGen( const image_to_pointcloud::MapInfoConstPtr& info);

  cv::Mat output;

  Util::CPPTimer timer_cloud;

  typedef pcl::PointXYZI GrayPoint;

  typedef pcl::PointXYZRGB ColorPoint;

  typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;

  ColorCloud::Ptr cloud;

  ColorCloud::Ptr final_map;

private:
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

};
