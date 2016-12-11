project(NEW_crop_row_detection)

cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE Debug)
# if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
#   set(CMAKE_BUILD_TYPE "Release")
# endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wnon-virtual-dtor -pedantic -Wall \
    -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 \
    -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith \
    -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code \
    -Wlogical-op -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls \
    -ggdb3 \
    -O0 \
    -fno-omit-frame-pointer -ffloat-store -fno-common -fstrict-aliasing \
    -lm")

# -Werror \
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lm -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic -Wconversion")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=nocona -O3")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

find_package(OpenCV 2 REQUIRED)
INCLUDE_DIRECTORIES("/usr/local/")

# find_package(catkin REQUIRED COMPONENTS
    #   roscpp
    # rospy
    # std_msgs
    # geometry_msgs
    # sensor_msgs
    # tf
    # cv_bridge
    # )
# find_package(Eigen3 REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
# )

# set(CMAKE_C_COMPILER "/usr/share/clang/scan-build/ccc-analyzer")
# set(CMAKE_CXX_COMPILER "/path/to/your/cpp/compiler/executable")


# include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
file(GLOB_RECURSE my_project_SRC "src/crop_row_detection_main.cpp")

add_executable(crop_row_detection ${my_project_SRC}
        src/ImagePreprocessor.cpp
        src/ImagePreprocessor.h
        src/CropRowDetector.cpp
        src/CropRowDetector.h
        # src/hello_world.cpp
        )
# target_link_libraries(crd_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(crop_row_detection ${OpenCV_LIBS})
