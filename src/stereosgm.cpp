// stereosgm.cpp
// Created by Zhang Handuo on 08/04/16.
// Modified by Zhang Handuo on 22/04/16.

#include "../include/stereosgm.h"
stereosgm::stereosgm( ):
        mWidth(640),
        mHeight(300),
        disp_size(128)
{}

stereosgm::~stereosgm() {}

void stereosgm::StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight, cv::Mat Q, const double &timestamp)
{
  int bits = 8;
  switch (imLeft.type()) {
    case CV_8UC1: bits = 8; left = imLeft; right = imRight; break;
    case CV_16UC1: bits = 16; left = imLeft; right = imRight; break;
    default:
    std::cerr << "invalid input image color format" << imLeft.type() << " So now converting to gray picture: " << std::endl;
    cvtColor(imLeft, left, CV_RGB2GRAY);
    cvtColor(imRight, right, CV_RGB2GRAY);
  }
  struct timeval start, end;
  long mtime, seconds, useconds;
  sgm::StereoSGM ssgm(imLeft.cols, imLeft.rows, disp_size, bits, 8, sgm::EXECUTE_INOUT_HOST2HOST);
	cv::Mat output(cv::Size(imLeft.cols, imLeft.rows), CV_8UC1);

	// gettimeofday(&start, NULL);
	ssgm.execute(left.data, right.data, (void**)&output.data);
	// gettimeofday(&end, NULL);

	// seconds  = end.tv_sec  - start.tv_sec;
  // useconds = end.tv_usec - start.tv_usec;
  // mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
  // std::cout << "Elapsed time: " << mtime << "milliseconds!"  << std::endl;

  // cvtColor(left, left_color, CV_GRAY2RGB);
  //3D point cloud caluculation
  cv::Mat cloud(output.size(),CV_32FC3);
  reprojectImageTo3D(output, cloud, Q, true);
// std::cout << "image type: " << type2str(left_color.type() ).c_str() << std::endl;

  // std::cout << "Genarating example point clouds.\n\n";
  // cv::imshow("image", output * 256 / disp_size);
  // cv::waitKey(1);

  // return cloud;
  visualizer(cloud);

}

void stereosgm::visualizer(cv::Mat &cloud){
  // pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  uint8_t r;
  // uint8_t r, g, b;
  for (int i=0; i<cloud.rows; i++){
    uchar* rgb_ptr = left.ptr<uchar>(i);
    for (int j=0; j<cloud.cols; j++){
      r = rgb_ptr[j];

      pcl::PointXYZRGB basic_point;
      cv::Vec3f cloudpoint = cloud.at<cv::Vec3f>(i, j);
      if (std::isfinite(cloudpoint[0]) && std::isfinite(cloudpoint[1]) && std::isfinite(cloudpoint[2]) ){
        basic_point.x = cloudpoint[0]/1000;
        basic_point.y = cloudpoint[1]/1000;
        basic_point.z = cloudpoint[2]/1000;
      //   uint32_t rgb = (static_cast<uint8_t>(r) << 16|
      // static_cast<uint8_t>(g) << 8 | static_cast<uint8_t>(b));
        uint32_t rgb = (static_cast<uint32_t>(r) << 16|
      static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
        basic_point.rgb = *reinterpret_cast<float*>(&rgb);
        basic_cloud_ptr->points.push_back(basic_point);
        //std::cout<< " ;"<<cloudpoint;//<<std::endl;
      }
    }
  }

  //std::cout<<points.size ()<<std::endl;
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  basic_cloud_ptr->header.frame_id = "map";
  // basic_cloud_ptr->

  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  // viewer = simpleVis(basic_cloud_ptr);
  // // viewer->spinOnce (100);
  // // cv::imshow("image", output);
  // // cv::waitKey(1);
  // //
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  // //
  //   boost::this_thread::sleep (boost::posix_time::microseconds (100));
  // //
  // //   // boost::mutex::scoped_lock updateLock(updateModelMutex);
  // //   // if(update)
  // //   // {
  // //   //     if(!viewer->updatePointCloud(cloud, "sample cloud"))
  // //   //       viewer->addPointCloud(cloud, colorHandler, "sample cloud");
  // //   //     update = false;
  // //   // }
  // //   // updateLock.unlock();
  // }

}

std::string stereosgm::type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

bool stereosgm::compare_mean(const cv::Point3i &a, const cv::Point3i &b) {
  return a.x > b.x;
}

bool stereosgm::compare_row(const cv::Point3i &a, const cv::Point3i &b) {
  return a.y > b.y;
}

bool stereosgm::compare_col(const cv::Point3i &a, const cv::Point3i &b) {
  return a.z > b.z;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> stereosgm::simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::Camera camera_;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
  // viewer->updateCamera();
  return (viewer);
}
