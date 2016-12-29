// stereosgm.cpp
// Created by Zhang Handuo on 08/04/16.
// Modified by Zhang Handuo on 22/04/16.

#include "../include/stereosgm.h"
#include <image_to_pointcloud/MapInfo.h>
#include "ros/ros.h"
#include <eigen_conversions/eigen_msg.h>


stereosgm::stereosgm() : mWidth(640), mHeight(282), disp_size(128) {
    final_map = ColorCloud::Ptr (new ColorCloud());
//    cv::Mat output(cv::Size(imLeft.cols, imLeft.rows), CV_8UC1);
}

stereosgm::~stereosgm() {}

void stereosgm::StereoMatching(const cv::Mat &imLeft, const cv::Mat &imRight,
                               cv::Mat Q, const double &timestamp,
                               const image_to_pointcloud::MapInfoConstPtr& info) {
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
    for (int r = 4; r < output.rows - 4; r++) {
        uchar* rgb_data = output.ptr<uchar>(r);
        for (int c = 4; c < output.cols - 4; c++) {
            uint8_t r1 = rgb_data[c];
            if (0 == r1) {
                if(rgb_data[c+1] > rgb_data[c-1]){
                    rgb_data[c] = rgb_data[c+1];
                }
                else{
                    rgb_data[c] = rgb_data[c-1];
                }
            }
        }
    }
    reprojectTo3D(output, Q, true, info);

}

void stereosgm::reprojectTo3D(cv::Mat &disparity, cv::Mat &Q,
                              bool handleMissingValues,
                              const image_to_pointcloud::MapInfoConstPtr& info) {
    timer_cloud.tic();
    int stype = disparity.type();

    ColorCloud::Ptr cloud(new ColorCloud());

    CV_Assert(stype == CV_8UC1 || stype == CV_16SC1 || stype == CV_32SC1 ||
              stype == CV_32FC1);
    CV_Assert(Q.size() == cv::Size(4, 4));

    int dtype = CV_32FC3;

    const double bigZ = 1000000.;
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
            if( (basic_point.z > 40) ||(basic_point.z < 0.5) )
                continue;
            basic_point.x = (float)X * 0.001f;
            basic_point.y = (float)Y * 0.001f;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
            basic_point.rgb = *reinterpret_cast<float *>(&rgb);
            cloud->points.push_back(basic_point);
        }
    }

    Eigen::Affine3d transform;
//    float res = info->resolution;
    GrayPoint newpoint;
    newpoint.x = info->origin.position.x;
    newpoint.y = info->origin.position.y;
    newpoint.z = info->origin.position.z;
//    viewer.addLine(viewpoint, newpoint, 0, 0, 1, to_string(info->header.seq));
//    viewpoint = newpoint;

    tf::poseMsgToEigen (info->origin, transform);

    ColorCloud::Ptr transformed_cloud(new ColorCloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    *final_map += *transformed_cloud;
//    *final_map = *cloud;

    ROS_INFO("Added refined cloud ID: %d; Using time %.4fs", info->header.seq, timer_cloud.end());


}

