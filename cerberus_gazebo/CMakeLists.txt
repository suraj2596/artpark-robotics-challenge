cmake_minimum_required(VERSION 3.0.2)
project(cerberus_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  gazebo_plugins
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For random_spawn_plugin plugin
add_library(random_spawn_plugin src/random_spawn_plugin.cpp)
target_link_libraries(random_spawn_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    gazebo_plugins
)
