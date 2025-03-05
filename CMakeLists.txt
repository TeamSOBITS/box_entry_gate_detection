cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(box_entry_gate_detection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS visualization cloud_viewer)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}

)
link_directories(${PCL_LIBRARY_DIRS})



add_executable(box_detect src/box_detect_cluster.cpp)
ament_target_dependencies(box_detect
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  visualization_msgs

)
target_link_libraries(box_detect
  ${PCL_LIBRARIES}
)

add_executable(topic_saver src/topic_saver.cpp)
ament_target_dependencies(topic_saver
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
)

add_executable(pcdfile_viewer src/pcdfile_viewer.cpp)
ament_target_dependencies(pcdfile_viewer
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
)
target_link_libraries(pcdfile_viewer
  ${PCL_LIBRARIES}
)

add_executable(scene_model src/scene_model.cpp)
ament_target_dependencies(scene_model
  rclcpp
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  pcl_conversions
)
target_link_libraries(scene_model
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  box_detect
  topic_saver
  pcdfile_viewer
  scene_model
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
