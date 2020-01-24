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
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  nodelet
  image_view
  jsk_perception
  visualization_msgs
  dynamic_reconfigure
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

generate_dynamic_reconfigure_options(
  cfg/RectangleDetection.cfg
  cfg/DepthDistanceFilter.cfg)

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
add_custom_target(install_sample_data ALL COMMAND ${PROJECT_SOURCE_DIR}/scripts/install_test_data.py)


macro(arp_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

arp_add_nodelet(src/ground_object_detection.cpp "aerial_robot_perception/groundObjectDetection" "ground_object_detection")
arp_add_nodelet(src/cluttered_brick_detection.cpp "aerial_robot_perception/ClutteredBrickDetection" "cluttered_brick_detection")
arp_add_nodelet(src/laser_line_extraction_ros.cpp "aerial_robot_perception/LaserLineExtraction" "laser_line_extraction")
arp_add_nodelet(src/rectangle_detection.cpp "aerial_robot_perception/RectangleDetection" "rectangle_detection")
arp_add_nodelet(src/depth_distance_filter.cpp "aerial_robot_perception/DepthDistanceFilter" "depth_distance_filter")
arp_add_nodelet(src/horizontal_brick_detection.cpp "aerial_robot_perception/HorizontalBrickDetection" "horizontal_brick_detection")


add_library(${PROJECT_NAME} SHARED ${aerial_robot_perception_nodelet_sources} src/line_extraction.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

# test
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  if(roslaunch_VERSION VERSION_LESS "1.11.1")
    message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
  else()
    file(GLOB LAUNCH_FILES launch/*.launch)
    foreach(LAUNCH_FILE ${LAUNCH_FILES})
      roslaunch_add_file_check(${LAUNCH_FILE})
    endforeach()
  endif()
  add_subdirectory(test)
endif()
