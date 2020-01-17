cmake_minimum_required(VERSION 2.5)
project(main)
add_definitions(-std=c++11)

add_executable(main main.cpp)
#add_executable(image_process image_process.cpp)
#add_executable(ceres_test ceres_test.cpp)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(main ${CERES_LIBRARIES})
#target_link_libraries(main ${CERES_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(image_process ${OpenCV_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBRARIES})
