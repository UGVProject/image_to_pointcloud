cmake_minimum_required(VERSION 2.8.3)
project(image_to_pointcloud)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/image_pcl_node.cpp src/stereosgm.cpp)

# rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

find_package(CUDA REQUIRED)

set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.")
# SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH}")
find_package(PCL 1.2 REQUIRED)

find_package(OpenCV REQUIRED core) # dammy
if(OpenCV_VERSION VERSION_LESS 3.0)
	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio calib3d objdetect) # 3.x
endif()

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})

include_directories(include)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME} ${catkin_INCLUDE_DIRS}
        CATKIN_DEPENDS roscpp
        image_transport
        DEPENDS opencv)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        std_msgs
        genmsg
        )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

CUDA_ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
# ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
add_library(image_to_pointcloud ${SRC_LIST} )

TARGET_LINK_LIBRARIES(image_to_pcl_node ${PROJECT_SOURCE_DIR}/lib/libsgm.a ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})

# generate_messages(DEPENDENCIES std_msgs)
