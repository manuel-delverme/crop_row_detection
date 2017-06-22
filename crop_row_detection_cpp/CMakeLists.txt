cmake_minimum_required (VERSION 2.8)
project(crop_row_detection_cpp)
cmake_policy(SET CMP0015 NEW)

SET(CMAKE_C_COMPILER /usr/bin/clang)
SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

message("BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -std=c++11 -g -ggdb3 -O0")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

#  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wconversion -std=c++11 -Wnon-virtual-dtor -pedantic -Wall
#      -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith
#      -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls
#      -Wno-ignored-qualifiers -g -ggdb3 -O0 -pipe -march=native -fopenmp")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -Ofast -march=native")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
        include/
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIR}
)

aux_source_directory( src CROP_ROW_DETECTION_SRC )

add_library(crop_row_detection STATIC ${CROP_ROW_DETECTION_SRC})
set_target_properties(crop_row_detection PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries( crop_row_detection
                       ${OpenCV_LIBS}
                       ${CERES_LIBRARIES})

add_executable(test_video apps/test_video.cpp)
target_link_libraries( test_video
                       crop_row_detection
                       ${OpenCV_LIBS}
                       ${CERES_LIBRARIES})
set_target_properties( test_video PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(test_image apps/test_image.cpp)
target_link_libraries( test_image
                       crop_row_detection
                       ${OpenCV_LIBS}
                       ${CERES_LIBRARIES})
set_target_properties( test_image PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
