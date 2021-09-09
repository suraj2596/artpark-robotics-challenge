cmake_minimum_required(VERSION 3.0.2)
project(entry_detector)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs
    std_msgs cv_bridge message_filters)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${OpenCV_LIBS})