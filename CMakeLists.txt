cmake_minimum_required(VERSION 2.5)

project(datafusion)
add_definitions(-std=c++11)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/local/dinclude/eigen3")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED 
src/readfile.cc
src/cluster.cc
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(main main.cpp)
target_link_libraries(main ${CERES_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBRARIES})
target_link_libraries(main ${PROJECT_NAME})