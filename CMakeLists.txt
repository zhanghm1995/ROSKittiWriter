cmake_minimum_required(VERSION 2.8.3)
project(ros_kitti_writer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread system program_options)

add_definitions(${PCL_DEFINITIONS})
###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_handle_node
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

 
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########
add_executable(ros_kitti_writer_node src/ros_kitti_writer_node.cpp src/kitti_writer.cpp src/messages_sync.cpp)
target_link_libraries(ros_kitti_writer_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

