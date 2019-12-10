/* ROSの基本ヘッダ */
#include <ros/ros.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pcdfile_translation");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  std::stringstream load_file;
  std::stringstream save_file;
  std::string pcdfile;


  nh.getParam("pcdfile", pcdfile);
  //std::cout << "pcdファイルを入力してください" << std::endl;
  //std::cin >> pcdfile;
  load_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/ascii/" << pcdfile;
  pcl::io::loadPCDFile(load_file.str(), cloud);

  if ((cloud.height * cloud.width) == 0){return 0;}

  save_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/binary/" << pcdfile;
  pcl::io::savePCDFile( save_file.str(), cloud, false );
  std::cout << "------------------------------\n------------------------------\n"
            << load_file.str() << " load." << std::endl;
  std::cout << save_file.str() << " saved."
            << "\n------------------------------\n------------------------------" << std::endl;
  

  return 0;
}
