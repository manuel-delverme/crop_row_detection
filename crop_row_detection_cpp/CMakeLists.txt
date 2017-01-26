project(crop_row_detection_cpp)

cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

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
    -g \
    -O0 \
    -pipe \
    -march=native \
    -fopenmp")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} \
    -Wconversion \
    -std=c++14 -Wnon-virtual-dtor -pedantic -Wall \
    -Wno-missing-braces -Wextra -Wno-missing-field-initializers -Wformat=2 \
    -Wswitch-default -Wswitch-enum -Wcast-align -Wpointer-arith \
    -Wstrict-overflow=5 -Winline -Wundef -Wcast-qual -Wshadow -Wunreachable-code \
    -Wlogical-op -Wfloat-equal -Wstrict-aliasing=2 -Wredundant-decls \
    -Ofast \
    -O3 \
    -pipe \
    -march=native \
    -fopenmp")

find_package(OpenCV 2 REQUIRED)

include_directories(
        include
)
add_library(crop_row_detection_cpp_main src/crop_row_detection_main.cpp)
add_library(crop_row_detection_cpp_preprocessor src/ImagePreprocessor.cpp)
add_library(crop_row_detection_cpp_detector src/CropRowDetector.cpp)

# add_executable(crop_row_detection
#         src/crop_row_detection_main.cpp
#         src/ImagePreprocessor.cpp
#         src/CropRowDetector.cpp
#        )
# target_link_libraries(${OpenCV_LIBS})
