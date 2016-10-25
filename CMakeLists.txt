cmake_minimum_required(VERSION 2.8.3)
project(viodom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  std_msgs
  image_transport
  image_geometry
  cv_bridge
  tf
  cmake_modules
)
find_package(OpenCV)
find_package(Eigen)

generate_dynamic_reconfigure_options(
  cfg/stereodom.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(viodom_node src/viodom_node.cpp)
target_link_libraries(viodom_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Eigen_LIBRARIES}
)
