cmake_minimum_required(VERSION 3.0.2)
project(tumrosbag)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  cv_bridge
  image_transport
  

  sensor_msgs
  geometry_msgs
  tf2_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_image_converter
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros rosbag cv_bridge image_transport 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(generate_rosbag src/generate_rosbag.cpp)
target_link_libraries(generate_rosbag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(generate_new src/generate_new.cpp)
target_link_libraries(generate_new ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})