cmake_minimum_required (VERSION 2.8)
project(crop_row_detection_cpp)

cmake_policy(SET CMP0015 NEW)
MESSAGE(STATUS "CPP 77777777777777777777====================DDDDDDDDDDDDDDDDDDDD")

set(CMAKE_BUILD_TYPE "Release")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
   set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} \
    -Wconversion \
    -std=c++14 -Wnon-virtual-dtor -pedantic -Wall \
    -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 \
    -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith \
    -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code \
    -Wlogical-op -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls \
    -Wno-ignored-qualifiers \
    -g \
    -ggdb3 \
    -O0 \
    -pipe \
    -march=native \
    -fopenmp")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
    -std=c++14 \
    -Ofast \
    -O2 \
    -pipe \
    -march=native")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
        tf
        cv_bridge
        )
catkin_package(INCLUDE_DIRS include)
include_directories(
        src
        include/
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        )

add_library(crop_row_detection_cpp_detector src/crop_row_detection_main.cpp src/CropRowDetector.cpp src/ImagePreprocessor.cpp)
target_link_libraries(crop_row_detection_cpp_detector ${OpenCV_LIBS} ${catkin_LIBRARIES} )
