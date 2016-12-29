// stereosgm.cpp
// Created by Zhang Handuo on 08/04/16.
// Modified by Zhang Handuo on 22/04/16.

#include "../include/stereosgm.h"
stereosgm::stereosgm() : mWidth(640), mHeight(282), disp_size(128) {
    basic_cloud_ptr = ColorCloud::Ptr( new ColorCloud );
}

stereosgm::~stereosgm() {}

void stereosgm::StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight,
                               cv::Mat Q, const double &timestamp) {
    int bits = 8;
    switch (imLeft.type()) {
        case CV_8UC1:
            bits = 8;
            left = imLeft;
            right = imRight;
            break;
        case CV_16UC1:
            bits = 16;
            left = imLeft;
            right = imRight;
            break;
        default:
            std::cerr << "invalid input image color format" << imLeft.type()
                      << " So now converting to gray picture: " << std::endl;
            cvtColor(imLeft, left, CV_RGB2GRAY);
            cvtColor(imRight, right, CV_RGB2GRAY);
    }
    // struct timeval start, end;
    // long mtime, seconds, useconds;
    sgm::StereoSGM ssgm(imLeft.cols, imLeft.rows, disp_size, bits, 8,
                        sgm::EXECUTE_INOUT_HOST2HOST);
    cv::Mat output(cv::Size(imLeft.cols, imLeft.rows), CV_8UC1);

    // gettimeofday(&start, NULL);
    ssgm.execute(left.data, right.data, (void **)&output.data);
    // gettimeofday(&end, NULL);
    for (int r = 10; r < output.rows - 10; r++) {
        uchar* rgb_data = output.ptr<uchar>(r);
        for (int c = 10; c < output.cols - 10; c++) {
            if (rgb_data[c] == 0) {
                if(rgb_data[c+1] > rgb_data[c-1]){
                    rgb_data[c] = rgb_data[c+1];
                }
                else{
                    rgb_data[c] = rgb_data[c-1];
                }
            }
        }
    }
    reprojectTo3D(output, Q, true);
    // seconds  = end.tv_sec  - start.tv_sec;
    // useconds = end.tv_usec - start.tv_usec;
    // mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    // std::cout << "Elapsed time: " << mtime << "milliseconds!"  << std::endl;

    // 3D point cloud caluculation
    // cv::Mat cloud(output.size(), CV_32FC3);
    // reprojectImageTo3D(output, cloud, Q, true);


    // std::cout << "image type: " << type2str(left_color.type() ).c_str() <<
    // std::endl;

    // std::cout << "Genarating example point clouds.\n\n";
    // cv::imshow("image", output * 256 / disp_size);
    // cv::waitKey(1);

//   visualizer(cloud);
}

void stereosgm::visualizer(cv::Mat &cloud) {
    basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>);
    uint8_t r;
    for (int i = 0; i < cloud.rows; i++) {
        uchar *rgb_ptr = left.ptr<uchar>(i);
        for (int j = 0; j < cloud.cols; j++) {
            r = rgb_ptr[j];

            pcl::PointXYZRGB basic_point;
            cv::Vec3f cloudpoint = cloud.at<cv::Vec3f>(i, j);
            // if (std::isfinite(cloudpoint[0]) && std::isfinite(cloudpoint[1]) &&
            //     std::isfinite(cloudpoint[2])) {
            basic_point.z = cloudpoint[2] * 0.001f;
            basic_point.x = cloudpoint[0] * 0.001f;
            basic_point.y = cloudpoint[1] * 0.001f;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
            basic_point.rgb = *reinterpret_cast<float *>(&rgb);
            basic_cloud_ptr->points.push_back(basic_point);
            // if ((i < 50) && (i > 20) && (j < 550) && (j > 450)) {
            //   std::cout << "x: " << basic_point.x << " and y: " << basic_point.y
            //             << " and z: " << basic_point.z << std::endl;
            // }
            // }
        }
    }

    // std::cout<<points.size ()<<std::endl;
    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
    basic_cloud_ptr->header.frame_id = "map";
    basic_cloud_ptr->is_dense = true;
    // basic_cloud_ptr->

//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//   viewer = simpleVis(basic_cloud_ptr);
//    viewer->spinOnce (100);
//    cv::imshow("image", output);
//    cv::waitKey(1);
//   //
//   while (!viewer->wasStopped ())
//   {
//     viewer->spinOnce (100);
//   //
//     boost::this_thread::sleep (boost::posix_time::microseconds (100));
//   //
////       boost::mutex::scoped_lock updateLock(updateModelMutex);
////       if(update)
////       {
////           if(!viewer->updatePointCloud(cloud, "sample cloud"))
////             viewer->addPointCloud(cloud, colorHandler, "sample cloud");
////           update = false;
////       }
////       updateLock.unlock();
//   }
}


boost::shared_ptr<pcl::visualization::PCLVisualizer>
stereosgm::simpleVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    // pcl::visualization::Camera camera_;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
            cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    // viewer->updateCamera();
    return (viewer);
}
void stereosgm::reprojectTo3D(cv::Mat &disparity, cv::Mat &Q,
                              bool handleMissingValues) {
    int stype = disparity.type();

    ColorCloud::Ptr cloud(new ColorCloud());

    CV_Assert(stype == CV_8UC1 || stype == CV_16SC1 || stype == CV_32SC1 ||
              stype == CV_32FC1);
    CV_Assert(Q.size() == cv::Size(4, 4));

    int dtype = CV_32FC3;

    const double bigZ = 10000.;
    double q[4][4];
    cv::Mat _Q(4, 4, CV_64F, q);
    Q.convertTo(_Q, CV_64F);

    int x, cols = disparity.cols;
    CV_Assert(cols >= 0);

    std::vector<float> _sbuf(cols + 1), _dbuf(cols * 3 + 1);
    float *sbuf = &_sbuf[0], *dbuf = &_dbuf[0];
    double minDisparity = FLT_MAX;

    // NOTE: here we quietly assume that at least one pixel in the disparity map
    // is not defined.
    // and we set the corresponding Z's to some fixed big value.
    if (handleMissingValues)
        cv::minMaxIdx(disparity, &minDisparity, 0, 0, 0);

    for (int y = 0; y < disparity.rows; y++) {
        float *sptr = sbuf, *dptr = dbuf;
        uchar *rgb_ptr = left.ptr<uchar>(y);
        double qx = q[0][1] * y + q[0][3], qy = q[1][1] * y + q[1][3];
        double qz = q[2][1] * y + q[2][3], qw = q[3][1] * y + q[3][3];

        if (stype == CV_8UC1) {
            const uchar *sptr0 = disparity.ptr<uchar>(y);
            for (x = 0; x < cols; x++)
                sptr[x] = (float)sptr0[x];
        } else
            sptr = (float *)disparity.ptr<float>(y);

        for (x = 0; x < cols;
             x++, qx += q[0][0], qy += q[1][0], qz += q[2][0], qw += q[3][0]) {
            uint8_t r = rgb_ptr[x];
            double d = sptr[x];
            double iW = 1. / (qw + q[3][2] * d);
            double X = (qx + q[0][2] * d) * iW;
            double Y = (qy + q[1][2] * d) * iW;
            double Z = (qz + q[2][2] * d) * iW;
            if (fabs(d - minDisparity) <= FLT_EPSILON)
                Z = bigZ;

            pcl::PointXYZRGB basic_point;

            basic_point.z = (float)Z * 0.001f;
            basic_point.x = (float)X * 0.001f;
            basic_point.y = (float)Y * 0.001f;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
            basic_point.rgb = *reinterpret_cast<float *>(&rgb);
            cloud->points.push_back(basic_point);
        }
    }
    *basic_cloud_ptr = *cloud;
//    basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
//    basic_cloud_ptr->height = 1;
    basic_cloud_ptr->header.frame_id = "map";
    basic_cloud_ptr->is_dense = false;
}
