cmake_minimum_required(VERSION 2.8.3)
project(manipulator_grasping)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  open_manipulator_msgs
  geometry_msgs
  tf
  message_generation
  std_msgs
  mavros_msgs
  nav_msgs
  cmake_modules
  cv_bridge
  image_transport
  uam_msgs
  #OpenCV
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs open_manipulator_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(
  vs_grasping_node
  src/vs_grasping_node.cpp
  src/vs_grasping_node_callback.cpp
  )
target_link_libraries(vs_grasping_node ${catkin_LIBRARIES})




