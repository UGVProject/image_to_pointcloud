cmake_minimum_required(VERSION 2.8.3)
project(image_to_pointcloud)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/image_pcl_node.cpp src/stereosgm.cpp include/timer.h)

# rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

find_package(CUDA REQUIRED)

set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.")
# SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH}")

find_package(OpenCV REQUIRED core)
if(OpenCV_VERSION VERSION_LESS 3.0)
	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio calib3d objdetect) # 3.x
endif()
#find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        pcl_ros
		cv_bridge
		image_transport
        geometry_msgs
		roscpp
		rospy
		std_msgs
        message_filters
		genmsg
        message_generation
        eigen_conversions
        )
## Generate messages in the 'msg' folder
add_message_files(
        FILES
        MapInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)
catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME} ${catkin_INCLUDE_DIRS}
        CATKIN_DEPENDS roscpp std_msgs message_runtime
#        image_transport
#        DEPENDS opencv
)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

CUDA_ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
# ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
add_library(image_to_pointcloud ${SRC_LIST} )

TARGET_LINK_LIBRARIES(image_to_pcl_node
        ${PROJECT_SOURCE_DIR}/lib/libsgm.a
        ${CUDA_LIBRARIES}
#        ${Eigen3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CUDA_nppi_LIBRARY}
        ${OpenCV_LIBS}
        )

ADD_EXECUTABLE(lcm2ros_node src/lcm2ros_node.cpp include/pose_t.hpp)
TARGET_LINK_LIBRARIES(lcm2ros_node
        ${catkin_LIBRARIES}
        -llcm
        )

# generate_messages(DEPENDENCIES std_msgs)
