#include <algorithm>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <errno.h>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include <string.h>
#include <unistd.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_to_pointcloud/MapInfo.h>
#include "../include/timer.h"
#include "../include/stereosgm.h"


using namespace std;
using namespace cv;
using namespace message_filters;

typedef pcl::PointXYZI GrayPoint;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
pcl::visualization::PCLVisualizer viewer ("Map of Non-Iterative SLAM");
float resolution = std::numeric_limits<float>::quiet_NaN();
ColorCloud::Ptr final_map(new ColorCloud());
pcl::VoxelGrid<ColorPoint> voxel;

class ImageGrabber {
public:
  ImageGrabber(stereosgm *pMATCHING) : mpMATCHING(pMATCHING) {}

  void GrabImage(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2, const image_to_pointcloud::MapInfoConstPtr& info);

  void MapGen(const Mat & color, const Mat & depth, const image_to_pointcloud::MapInfoConstPtr& info);

  stereosgm *mpMATCHING;
//  std_msgs::Header head;
  float scale;
  bool do_rectify = true;
  int height = 1024, width = 1280;
//  int height_up_cut = 140, height_down_cut = 704;
  cv::Mat M1l, M2l, M1r, M2r, Q;
  int disp_size;
  Util::CPPTimer timer_cloud;
  GrayPoint viewpoint;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_to_pcl_node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  stereosgm SGM_Matching;

  ImageGrabber igb(&SGM_Matching);
  bool rectify = true;
  igb.scale = 0.5;
  igb.disp_size = 128;

  string calibration = ros::package::getPath("image_to_pointcloud") + "/wide_stereo.yaml";

  if (n.getParam("scale", igb.scale))
    ROS_INFO("Get scale: %d", igb.scale);
  else
    ROS_WARN("Using default scale: %d", igb.scale);

    if(n.getParam("rectify", rectify))
        ROS_INFO("Get rectify flag: %s",rectify? "true":"false");
    else
        ROS_WARN("Using default rectify flag: true!");

    if(n.getParam("calibration", calibration))
        ROS_INFO("Get calibration parameters: %s", calibration.c_str());
    else
        ROS_WARN("Use default calibration file position: %s", calibration.c_str());
  if (n.getParam("disparity_size", igb.disp_size))
    ROS_INFO("Get disparity size: %d", igb.disp_size);
  else
    ROS_WARN("Using default size: %d", igb.disp_size);

    if(n.getParam("/map_visualization/resolution", resolution))
    {
        ROS_WARN("Set fixed visualization resolution parameter: %f", resolution);
        viewer.addPointCloud(final_map, "ugv_slam_map");
    }
    else
        ROS_WARN("Using dynamic visualization resolution!");
  // ros::start();

  if (rectify) {
    igb.do_rectify = true;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(calibration,
                                       cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      cerr << "ERROR: Wrong path to settings of settings" << endl;
      return -1;
    }

      cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, R, T;
      cv::Rect roi1, roi2;
      fsSettings["LEFT.K"] >> K_l;
      K_l *= igb.scale;
      fsSettings["RIGHT.K"] >> K_r;
      K_r *= igb.scale;

      fsSettings["R"] >> R;
      fsSettings["T"] >> T;

      fsSettings["LEFT.P"] >> P_l;
      fsSettings["RIGHT.P"] >> P_r;

      fsSettings["LEFT.R"] >> R_l;
      fsSettings["RIGHT.R"] >> R_r;

      fsSettings["LEFT.D"] >> D_l;
      fsSettings["RIGHT.D"] >> D_r;

      int rows_l = fsSettings["LEFT.height"];
      int cols_l = fsSettings["LEFT.width"];
      int rows_r = fsSettings["RIGHT.height"];
      int cols_r = fsSettings["RIGHT.width"];

      if (K_l.empty() || K_r.empty() || R.empty() || T.empty() || D_l.empty() ||
          D_r.empty() || rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
          cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
               << endl;
          return -1;
    }
      cv::Size Img_size(cols_l, rows_l);
      cv::stereoRectify( K_l, D_l, K_r, D_r, Img_size, R, T, R_l, R_r, P_l, P_r, igb.Q, cv::CALIB_ZERO_DISPARITY, -1, Img_size, &roi1, &roi2 );
      cv::initUndistortRectifyMap(
              K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
              Img_size, CV_32F, igb.M1l, igb.M2l);
      cv::initUndistortRectifyMap(
              K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
              Img_size, CV_32F, igb.M1r, igb.M2r);

  }

    message_filters::Subscriber<sensor_msgs::Image> img_sub1(nh, "/wide/left/image_raw", 10 );

    message_filters::Subscriber<sensor_msgs::Image> img_sub2(nh, "/wide/right/image_raw", 10 );

    message_filters::Subscriber<image_to_pointcloud::MapInfo> info_sub(n, "/pose_body", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, image_to_pointcloud::MapInfo> SyncPolicy;

    Synchronizer<SyncPolicy> sync(SyncPolicy(50), img_sub1, img_sub2, info_sub );

    sync.setMaxIntervalDuration(ros::Duration(0.03));
//    ros::Publisher point_pub =
//      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/points", 1);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage, &igb, _1, _2, _3 ));

    viewer.addCoordinateSystem(0.2, "reference", 0);

    viewer.setBackgroundColor(1, 1, 1, 0);

    voxel.setLeafSize(resolution, resolution, resolution);
  //  ros::Rate loop_rate(30);
  while (nh.ok()) {
//    if (SGM_Matching.basic_cloud_ptr != NULL) {
//      SGM_Matching.basic_cloud_ptr->header.stamp =
//      ros::Time::now().toNSec() / 1000ull;
//      point_pub.publish(SGM_Matching.basic_cloud_ptr);
//    }
    ros::spinOnce();
    viewer.spinOnce();
//    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2, const image_to_pointcloud::MapInfoConstPtr& info) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
    try {
        cv_ptr1 = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    try {
        cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat imLeft_scale, imRight_scale, imLeft1, imRight1;
    imLeft1 = cv_ptr1->image;
    imRight1 = cv_ptr2->image;
    width = imLeft1.cols;
    height = imLeft1.rows;
    cv::Size rzSize(int(width * scale), int(height * scale) );
    cv::resize(imLeft1, imLeft_scale, rzSize);
    cv::resize(imRight1, imRight_scale, rzSize);

    if (do_rectify) {
        Mat imLeftRec, imRightRec;
        remap(imLeft_scale(Range(int(140 * scale), int(704 * scale)), Range::all()),
              imLeftRec, M1l, M2l, cv::INTER_LINEAR);
        remap(imRight_scale(Range(int(140 * scale), int(704 * scale)), Range::all()),
              imRightRec, M1r, M2r, cv::INTER_LINEAR);
        mpMATCHING->StereoMatching(imLeftRec, imRightRec, Q,
                                   cv_ptr1->header.stamp.toSec());
        MapGen(imLeftRec, mpMATCHING->output, info);

    } else {
        mpMATCHING->StereoMatching(imLeft_scale, imRight_scale, Q,
                                   cv_ptr1->header.stamp.toSec());
        MapGen(imLeft_scale, mpMATCHING->output, info);
    }



}

void ImageGrabber::MapGen(const Mat & color, const Mat & depth, const image_to_pointcloud::MapInfoConstPtr& info) {
    timer_cloud.tic();
    Eigen::Affine3d transform;
    GrayPoint newpoint;
    newpoint.x = info->origin.position.x;
    newpoint.y = info->origin.position.y;
    newpoint.z = info->origin.position.z;
    viewer.addLine(viewpoint, newpoint, 0, 0, 1, to_string(info->header.seq));
    viewpoint = newpoint;

    tf::poseMsgToEigen (info->origin, transform);
    ColorCloud::Ptr cloud(new ColorCloud());
    int height_center = info->height/2;
    int width_center = info->width/2;

    for(int i = 0; i < info->height; i++)
        for(int j = 0; j < info->width; j++)
        {
            ColorPoint point;
            point.z = depth.at<float>(i, j);
            if(point.z < 0.2)
                continue;
            point.y = (i - height_center) * info->resolution;
            point.x = (j - width_center) * info->resolution;
            point.r = color.at<float>(i, j) * 255;
            point.g = color.at<float>(i, j) * 255;
            point.b = color.at<float>(i, j) * 255;
            cloud->push_back(point);
        }
    ColorCloud::Ptr transformed_cloud(new ColorCloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    if (!isnan(resolution))
    {
        *final_map += *transformed_cloud;
        ColorCloud::Ptr tmp_cloud(new ColorCloud());
        voxel.setInputCloud( final_map );
        voxel.filter( *tmp_cloud );
        final_map = tmp_cloud;
        viewer.updatePointCloud(final_map, "ugv_map");
        cout<<"ddd"<<endl;
    }
    else
        viewer.addPointCloud(transformed_cloud, to_string(info->header.seq));

    ROS_INFO("Added refined cloud ID: %d; Using time %.4fs", info->header.seq, timer_cloud.end());

}