cmake_minimum_required(VERSION 2.8.3)
project(Stereo_VO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC_LIST src/stereo_vo_node.cpp )

# rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find_package(CUDA REQUIRED)
#
# set(CUDA_ARCH "-arch=sm_50" CACHE STRING "Value of the NVCC -arch option.")

# find_package(PCL 1.2 REQUIRED)

# find_package(OpenCV REQUIRED HINTS /home/zh/softwares/opencv-3.1.0/install)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(OpenCV REQUIRED core) # dammy
if(OpenCV_VERSION VERSION_LESS 3.0)
	find_package(OpenCV REQUIRED core highgui imgproc)  # 2.x
else()
	find_package(OpenCV REQUIRED core highgui imgproc imgcodecs videoio calib3d objdetect) # 3.x
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/lib/libStereo_VO.so
)

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# include_directories(${PCL_INCLUDE_DIRS})

# link_directories(${PCL_LIBRARY_DIRS})

# add_definitions(${PCL_DEFINITIONS})

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

# CUDA_ADD_EXECUTABLE(image_to_pcl_node ${SRC_LIST} ${CUDA_SRC})

ADD_EXECUTABLE(SLAM ${SRC_LIST} )

add_library(Stereo_VO ${SRC_LIST} )

TARGET_LINK_LIBRARIES(SLAM ${catkin_LIBRARIES} ${LIBS} )

# generate_messages(DEPENDENCIES std_msgs)
