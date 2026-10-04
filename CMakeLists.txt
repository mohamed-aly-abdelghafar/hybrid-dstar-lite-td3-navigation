cmake_minimum_required(VERSION 3.8)
project(hybrid_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

# Gazebo plugin that moves the obstacles of the dynamic test environments
add_library(patrol_obstacle SHARED plugins/patrol_obstacle.cc)
target_include_directories(patrol_obstacle PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(patrol_obstacle PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_compile_options(patrol_obstacle PRIVATE ${GAZEBO_CXX_FLAGS})
target_link_libraries(patrol_obstacle ${GAZEBO_LIBRARIES})

install(TARGETS patrol_obstacle
  LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch maps worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
