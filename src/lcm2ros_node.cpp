#include <fstream>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <map>
#include "../include/pose_t.hpp"
#include <image_to_pointcloud/MapInfo.h>

using namespace std;

class LCM2ROS{
public:
    LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_);
    ~LCM2ROS() {}

private:
    boost::shared_ptr<lcm::LCM> lcm_;
    ros::NodeHandle nh_;

    void poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const exlcm::pose_t* msg);
    image_to_pointcloud::MapInfo map_info;
    ros::Publisher pose_body_pub_;
    ros::NodeHandle* rosnode;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_): lcm_(lcm_),nh_(nh_) {
    lcm_->subscribe("POSE_CAM",&LCM2ROS::poseBodyHandler, this);
    pose_body_pub_ = nh_.advertise<image_to_pointcloud::MapInfo>("/pose_body",10);
    rosnode = new ros::NodeHandle();
}

void LCM2ROS::poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const exlcm::pose_t* msg) {
    //ROS_ERROR("LCM2ROS got pose_t");
    geometry_msgs::Pose msgout;
//    map_info.header.stamp= ros::Time().fromSec(msg->utime*1E-6);
    map_info.header.stamp= ros::Time().fromSec(msg->utime);
    map_info.resolution = msg->resolution;
    map_info.width = msg->width;
    map_info.height = msg->height;
    map_info.status = msg->status;
    map_info.origin.position.x = msg->position[0];
    map_info.origin.position.y = msg->position[1];
    map_info.origin.position.z = msg->position[2];
    map_info.origin.orientation.x = msg->orientation[0];
    map_info.origin.orientation.y = msg->orientation[1];
    map_info.origin.orientation.z = msg->orientation[2];
    map_info.origin.orientation.w = msg->orientation[3];
//    cout << "position.x = " << map_info.origin.position.x << endl
//         << "position.y = " << map_info.origin.position.y << endl
//         << "position.z = " << map_info.origin.position.z << endl
//         << "rotation.x = " << map_info.origin.orientation.x << endl
//         << "rotation.y = " << map_info.origin.orientation.y << endl;
    pose_body_pub_.publish(map_info);
}


int main(int argc, char **argv) {
    ros::init(argc,argv,"lcm2ros_node",ros::init_options::NoSigintHandler);
    boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
    if(!lcm->good()){
        std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
    ros::NodeHandle nh;

    LCM2ROS handlerObject(lcm, nh);
    cout << "\nlcm2ros translator ready!\n";

    while(0 == lcm->handle());
    return 0;
}

