cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find ROS
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build Stewart plugin
add_library(stewart_plugin SHARED stewart_plugin.cpp)
target_link_libraries(stewart_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

# Build Ball1 plugin
add_library(ball1_plugin SHARED ball1_plugin.cpp)
target_link_libraries(ball1_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})


# Build Ball2 plugin
add_library(ball2_plugin SHARED ball2_plugin.cpp)
target_link_libraries(ball2_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
