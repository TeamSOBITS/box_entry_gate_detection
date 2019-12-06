#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>

int main(){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::stringstream load_file;
  std::stringstream save_file;
  std::string pcdfile;
  int save_count = 0;

  ros::param::get("pcdfile", pcdfile);

  //std::cout << "pcdファイルを入力してください" << std::endl;
  //std::cin >> pcdfile;
  load_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/ascii/" << pcdfile;
  pcl::io::loadPCDFile(load_file.str(), cloud);
  std::cout << load_file.str() << " load." << std::endl;
  if ((cloud.height * cloud.width) == 0){return 0;}

  save_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/binary/" << pcdfile;
  pcl::io::savePCDFile( save_file.str(), cloud, false );
  std::cout << save_file.str() << " saved." << std::endl;
  save_count++;
  usleep( 300000 );

  return 0;
}
