project(crop_row_detection)

cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV 2  REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cv_bridge
)
find_package(Eigen3 REQUIRED)
catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(../crop_row_detection_cpp/include)

file(GLOB_RECURSE my_project_SRC "src/*.cpp")

add_executable(crop_row_detection_node ${my_project_SRC})

target_link_libraries(crop_row_detection_node
        crop_row_detection_cpp_detector
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})