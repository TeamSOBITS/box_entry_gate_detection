cmake_minimum_required(VERSION 3.10.2)
project(box_entry_gate_detection)

# Use ament_cmake for building
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sobits_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS visualization)

# Add executable targets
add_executable(box_detect src/box_detect_cluster.cpp)
ament_target_dependencies(box_detect
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  visualization_msgs
  PCL
  tf2_eigen
)

add_executable(topic_saver src/topic_saver.cpp)
ament_target_dependencies(topic_saver
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  std_msgs
)

add_executable(pcdfile_viewer src/pcdfile_viewer.cpp)
ament_target_dependencies(pcdfile_viewer
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  std_msgs
)

add_executable(scene_model src/scene_model.cpp)
ament_target_dependencies(scene_model
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_interfaces
  std_msgs
)

# Link PCL libraries
target_link_libraries(box_detect ${PCL_LIBRARIES})
target_link_libraries(pcdfile_viewer ${PCL_LIBRARIES})
target_link_libraries(scene_model ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  box_detect
  topic_saver
  pcdfile_viewer
  scene_model
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
