cmake_minimum_required(VERSION 2.8.3)
project(viodom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  std_msgs
  image_transport
  image_geometry
  cv_bridge
  tf
  cmake_modules
)

find_package(Eigen3)
find_package(OpenCV 3 REQUIRED COMPONENTS core highgui imgproc xfeatures2d)
#find_package(OpenCV 3 REQUIRED PATHS "/opt/opencv/" COMPONENTS core highgui imgproc xfeatures2d)

generate_dynamic_reconfigure_options(
  cfg/stereodom.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

#link_directories(${OpenCV_LIB_DIR})

add_executable(viodom_node src/viodom_node.cpp)

target_link_libraries(viodom_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Eigen3_LIBRARIES}
)

