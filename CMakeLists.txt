cmake_minimum_required(VERSION 3.10.2)
project(box_entry_gate_detection)
  add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  sobits_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  sensor_msgs 
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)


add_executable(box_detect src/box_detect_cluster.cpp)
target_link_libraries(box_detect ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(topic_saver src/topic_saver.cpp)
target_link_libraries(topic_saver ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pcdfile_saver_binary src/pcdfile_saver_binary.cpp)
target_link_libraries(pcdfile_saver_binary ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(scene_model src/scene_model.cpp)
target_link_libraries(scene_model ${PCL_LIBRARIES} ${catkin_LIBRARIES})
