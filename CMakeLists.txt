cmake_minimum_required(VERSION 2.8.3)
project(image_to_pointcloud)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/image_pcl_node.cpp src/stereosgm.cpp include/stereo_matching.h)

# rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCOMPILEDWITHC11)
	message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	add_definitions(-DCOMPILEDWITHC0X)
	message(STATUS "Using flag -std=c++0x.")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(CUDA REQUIRED)
FIND_PACKAGE(VisionWorks REQUIRED)
FIND_PACKAGE(VisionWorks-NVXIO REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.")
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH}")

#find_package(PCL 1.2 REQUIRED)

find_package(OpenCV REQUIRED core) # dammy
if(OpenCV_VERSION VERSION_LESS 3.0)
	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio calib3d objdetect) # 3.x
endif()

#include_directories(${PCL_INCLUDE_DIRS})

include_directories(include)

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
		)
###
catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES ${PROJECT_NAME} ${catkin_INCLUDE_DIRS}
#        CATKIN_DEPENDS roscpp std_msgs
#        image_transport
#        DEPENDS opencv
)

include_directories(
		${OpenCV_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
		${VisionWorks_INCLUDE_DIRS}
		${VisionWorks-NVXIO_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS} )

#add_definitions(${PCL_DEFINITIONS})
#CUDA_ADD_LIBRARY(det ${SRC_LIST} STATIC)
add_library(stereo_camera_gpu src/stereo_matching.cpp )

CUDA_ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
# ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})
add_library(image_to_pointcloud ${SRC_LIST} )

#TARGET_LINK_LIBRARIES(image_to_pcl_node ${PROJECT_SOURCE_DIR}/lib/libsgm.a ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${CUDA_nppi_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(image_to_pcl_node ${PROJECT_SOURCE_DIR}/lib/libsgm.a stereo_camera_gpu
		${CUDA_LIBRARIES}
		${catkin_LIBRARIES}
		${CUDA_nppi_LIBRARY}
		${OpenCV_LIBS}
		${OpenMP_LIBS}
		${VisionWorks_LIBRARIES}
		${VisionWorks-NVXIO_LIBRARIES})

# generate_messages(DEPENDENCIES std_msgs)
