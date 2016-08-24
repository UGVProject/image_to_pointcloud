#include <algorithm>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
// #include <sensor_msgs
#include "../include/stereosgm.h"

using namespace std;
using namespace cv;

class ImageGrabber {
public:
  ImageGrabber(stereosgm *pMATCHING) : mpMATCHING(pMATCHING) {}

  void GrabImage(const sensor_msgs::ImageConstPtr &msg);

  stereosgm *mpMATCHING;
  std_msgs::Header head;
  bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r, Q;
  int disp_size, scale;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_to_pcl_node");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  stereosgm SGM_Matching;

  ImageGrabber igb(&SGM_Matching);
  igb.scale = 2;
  igb.disp_size = 64;
  // cv::Size fsize(640, 300);
  cv::Size fsize(640, 208);
  std::string intrinsic_filename, extrinsic_filename;
  intrinsic_filename =
      "/home/zh/catkin_ws/src/image_to_pointcloud/intrinsics.yml";
  extrinsic_filename =
      "/home/zh/catkin_ws/src/image_to_pointcloud/extrinsics.yml";

  if (n.getParam("scale", igb.scale))
    ROS_INFO("Get scale: %d", igb.scale);
  else
    ROS_WARN("Using default scale: %d", igb.scale);

  if (n.getParam("intrinsic_filename", intrinsic_filename))
    ROS_INFO("Get intrinsic_filename");
  else
    ROS_WARN("Using default scale");

  if (n.getParam("extrinsic_filename", extrinsic_filename))
    ROS_INFO("Get extrinsic_filename");
  else
    ROS_WARN("Using default scale");

  if (n.getParam("disparity_size", igb.disp_size))
    ROS_INFO("Get disparity size: %d", igb.disp_size);
  else
    ROS_WARN("Using default size: %d", igb.disp_size);

  // ros::start();

  if (1) {
    igb.do_rectify = true;

    // Load settings related to stereo calibration
    cv::FileStorage intrinsic_settings(intrinsic_filename,
                                       cv::FileStorage::READ);
    if (!intrinsic_settings.isOpened()) {
      cerr << "ERROR: Wrong path to settings of intrinsic_settings" << endl;
      return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, R, T;
    intrinsic_settings["M1"] >> K_l;
    K_l /= igb.scale;
    intrinsic_settings["M2"] >> K_r;
    K_r /= igb.scale;

    intrinsic_settings["D1"] >> D_l;
    intrinsic_settings["D2"] >> D_r;

    cv::FileStorage extrinsic_settings(extrinsic_filename,
                                       cv::FileStorage::READ);
    if (!extrinsic_settings.isOpened()) {
      cerr << "ERROR: Wrong path to settings of extrinsic_settings" << endl;
      return -1;
    }
    extrinsic_settings["R"] >> R;
    extrinsic_settings["T"] >> T;

    if (K_l.empty() || K_r.empty() || R.empty() || T.empty() || D_l.empty() ||
        D_r.empty()) {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
           << endl;
      return -1;
    }
    cv::Rect roi1, roi2;

    cv::stereoRectify(K_l, D_l, K_r, D_r, fsize, R, T, R_l, R_r, P_l, P_r,
                      igb.Q, cv::CALIB_ZERO_DISPARITY, -1, fsize, &roi1, &roi2);
    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l, fsize, CV_16SC2, igb.M1l,
                                igb.M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r, fsize, CV_16SC2, igb.M1r,
                                igb.M2r);
  }

  // ros::NodeHandle nodeHandler;
  ros::Subscriber sub =
      nh.subscribe("/wide/image_raw", 1, &ImageGrabber::GrabImage, &igb);

  // ros::init(argc, argv, "publish_point_cloud");
  ros::Publisher point_pub =
      nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/points", 1);

  // ros::spin();
  ros::Rate loop_rate(30);
  // ros::Rate loop_rate(100);
  while (nh.ok()) {
    if (SGM_Matching.basic_cloud_ptr != NULL) {
      SGM_Matching.basic_cloud_ptr->header.stamp =
          ros::Time::now().toNSec() / 1000ull;
      // SGM_Matching.basic_cloud_ptr->header.stamp =
      // igb.head.stamp.toNSec()/1000ull;
      // SGM_Matching.basic_cloud_ptr->header.frame_id = "map";
      // pcl_conversions::toPCL(igb.head, SGM_Matching.basic_cloud_ptr->header);
      // pcl_conversions::toPCL(ros::Time::now(),
      // SGM_Matching.basic_cloud_ptr->header.stamp);
      point_pub.publish(SGM_Matching.basic_cloud_ptr);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImagePtr cv_ptr;
  head = msg->header;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image_full, image_scale, imLeft1, imRight1;
  image_full = cv_ptr->image;
  cv::Size rzSize(2560 / scale, 1024 / scale);
  cv::resize(image_full, image_scale, rzSize);
  // imLeft1 = image_scale(Range(75, 375), Range(0 / scale, 1280 / scale));
  // imRight1 = image_scale(Range(75, 375), Range(1280 / scale, 2560 / scale));
  imLeft1 = image_scale(Range(152, 360), Range(0 / scale, 1280 / scale));
  imRight1 = image_scale(Range(152, 360), Range(1280 / scale, 2560 / scale));
  if (1) {
    Mat imLeftRec, imRightRec;
    remap(imLeft1, imLeftRec, M1l, M2l, cv::INTER_LINEAR);
    remap(imRight1, imRightRec, M1r, M2r, cv::INTER_LINEAR);
    mpMATCHING->StereoMatching(imLeftRec, imRightRec, Q,
                               cv_ptr->header.stamp.toSec());
  } else {
    mpMATCHING->StereoMatching(imLeft1, imRight1, Q,
                               cv_ptr->header.stamp.toSec());
  }
}
