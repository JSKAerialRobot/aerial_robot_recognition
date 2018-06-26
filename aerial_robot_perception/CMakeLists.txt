cmake_minimum_required(VERSION 2.8.3)
project(aerial_robot_perception)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  jsk_topic_tools
  nav_msgs
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
  nodelet
  image_view
  jsk_perception
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge geometry_msgs jsk_topic_tools nav_msgs roscpp rospy tf2_ros nodelet jsk_perception
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

# download and install sample data
add_custom_target(install_sample_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_sample_data.py)


macro(arp_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

arp_add_nodelet(src/find_object_with_size_filter.cpp "aerial_robot_perception/FindObjectWithSizeFilter" "find_object_with_size_filter")

add_library(${PROJECT_NAME} SHARED ${aerial_robot_perception_nodelet_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()
