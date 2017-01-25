project(NEW_crop_row_detection)

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


# -Og \
# -pthread \
# -Ofast \
# -Werror \
# -ggdb3 \

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES("/usr/local/")

include_directories(
        include
)
add_executable(crop_row_detection
        src/crop_row_detection_main.cpp
        src/ImagePreprocessor.cpp
        src/CropRowDetector.cpp
        src/get_real_phase.h)
target_link_libraries(crop_row_detection ${OpenCV_LIBS})
