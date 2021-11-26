cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Build Stewart plugin
add_library(stewart_plugin SHARED stewart_plugin.cpp)
target_link_libraries(stewart_plugin ${GAZEBO_LIBRARIES})


# Build Stewart Velocity API
add_executable(velocityAPI velocityAPI.cpp)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(velocityAPI ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(velocityAPI ${GAZEBO_LIBRARIES})
endif()
