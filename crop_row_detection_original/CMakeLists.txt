cmake_minimum_required (VERSION 2.8)
project(crop_row_detection_original)

cmake_policy(SET CMP0015 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_BUILD_TYPE "Release")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
   set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
        tf
        cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(INCLUDE_DIRS include)
include_directories(
    # src
    include
    ../crop_row_detection_cpp/include
    # ../crop_row_detection_cpp/src
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    # ${Eigen_INCLUDE_DIRS}
)

# add_definitions(${EIGEN_DEFINITIONS})
file(GLOB_RECURSE my_project_SRC "src/*.cpp")


add_executable(crop_row_detection_node 
    ${my_project_SRC}
    )

target_link_libraries(crop_row_detection_node
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        crop_row_detection_cpp_detector
)
