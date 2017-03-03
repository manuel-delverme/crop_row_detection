cmake_minimum_required (VERSION 2.8)
project(crop_row_detection_cpp)

cmake_policy(SET CMP0015 NEW)
MESSAGE(STATUS "CPP 77777777777777777777====================DDDDDDDDDDDDDDDDDDDD")

set(CMAKE_BUILD_TYPE "Release")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
   set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -std=c++11")

# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}\
#     -Wconversion \
#     -std=c++14 -Wnon-virtual-dtor -pedantic -Wall \
#     -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 \
#     -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith \
#     -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code \
#     -Wlogical-op -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls \
#     -Wno-ignored-qualifiers \
#     -g \
#     -ggdb3 \
#     -O0 \
#     -pipe \
#     -march=native \
#     -fopenmp")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -Ofast -march=native")

# -falign-functions -falign-jumps -falign-labels \
# -falign-loops -fbranch-probabilities -fcaller-saves \
# -fcprop-registers -fcse-follow-jumps -fcse-skip-blocks \
# -fdata-sections -fdelayed-branch -fdelete-null-pointer-checks \
# -fexpensive-optimizations -ffast-math -ffloat-store -fforce-addr \
# -ffunction-sections -fgcse -fgcse-lm -fgcse-sm \
# -floop-optimize -fcrossjumping -fif-conversion -fif-conversion2 \
# -finline-functions -fkeep-inline-functions \
# -fkeep-static-consts -fmerge-constants -fmerge-all-constants \
# -fno-branch-count-reg \
# -fno-guess-branch-probability -fno-inline -fno-math-errno \
# -fno-peephole -fno-peephole2 -funsafe-math-optimizations -ffinite-math-only \
# -fno-trapping-math -fno-zero-initialized-in-bss \
# -fomit-frame-pointer -foptimize-register-move \
# -fprefetch-loop-arrays -fregmove \
# -frename-registers -freorder-blocks -freorder-functions -frerun-cse-after-loop \
# -frerun-loop-opt -fschedule-insns -fschedule-insns2 \
# -fno-sched-interblock -fno-sched-spec \
# -fsched-spec-load -fsched-spec-load-dangerous -fsignaling-nans \
# -fstrength-reduce -fstrict-aliasing -ftracer -fthread-jumps \
# -funroll-all-loops -funroll-loops \
# -fopenmp"
# -Wconversion \
# -Wnon-virtual-dtor -pedantic -Wall \
# -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 \
# -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith \
# -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code \
# -Wlogical-op -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls \

find_package(OpenCV REQUIRED)
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
        include/
        ${CERES_INCLUDE_DIRS}
        # ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
        )

add_library(crop_row_detection_cpp_detector
        src/crop_row_detection_main.cpp
        src/CropRowDetector.cpp
        src/ImagePreprocessor.cpp
        )

target_link_libraries(crop_row_detection_cpp_detector
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        )
