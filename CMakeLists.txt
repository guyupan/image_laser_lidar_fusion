cmake_minimum_required(VERSION 2.5)
project(datafusion)
add_definitions(-std=c++11)
set(SRC_LIST src/main.cpp)


include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Eigen3 3.1.2 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC src/readfile.cc)

add_executable(main ${SRC_LIST})
target_link_libraries(main ${CERES_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBRARIES})
target_link_libraries(main ${PROJECT_NAME})