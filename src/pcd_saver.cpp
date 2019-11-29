#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>

int main(){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/bin.pcd", cloud);

  int save_count = 0;

  //pcl_ros::transformPointCloud("/base_link", ros::Time(0), cloud, camera_frame_name, cloud, listerner);

  if ((cloud.height * cloud.width) == 0)  return 0;
  //std::stringstream filename1;
  //filename1 << "/home/rg-25/catkin_ws/src/pcl_matcher_real/pcd/" << save_count << ".png";
  //cv::imwrite( filename1.str(), color_img );
  //std::cout << filename1.str() << " saved." << std::endl;
  std::stringstream filename2;
  filename2 << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/" << "bin_ASCII" << ".pcd";
  //pcl::io::savePCDFileBinary( filename2.str(), input_cloud );
  pcl::io::savePCDFile( filename2.str(), cloud, false );
  std::cout << filename2.str() << " saved." << std::endl;
  save_count++;
  usleep( 300000 );

  return 0;
}
