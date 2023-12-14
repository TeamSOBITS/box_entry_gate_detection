/* Reading point cloud data from a PCD file, converting it into the sensor_msgs::PointCloud2 type, and publishing */

/* ROS Basic Header */
#include <ros/ros.h>
/* Input/Output Related Header */
#include <iostream>
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
/* sensor_msgs */
#include <sensor_msgs/PointCloud2.h>
#include "unistd.h"



typedef pcl::PointXYZ PointT; // Convert pcl::PointXYZ to PointT (point cloud data structure)
typedef pcl::PointCloud<PointT> PointCloud; // Smart pointer pcl::PointCloud<pcl::PointXYZ> → PointCloud

class PointcloudPublisherNode
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    /* Publisher */
    ros::Publisher pub_cloud_sensor_;       // Point cloud publisher
    ros::Publisher pub_cloud_;              // Point cloud publisher
    /* param */
    std::string data_path_;                 // Path to the PCD file


  // Rotation of the PointCloud (if needed)
  void rotation(){
    PointCloud::Ptr cloud (new PointCloud());                 // Point Cloud variable (pointer) for storing point cloud data
    PointCloud::Ptr cloud_transformed (new PointCloud());     // Point Cloud variable (pointer) for storing point cloud data

    /* Rotation about the x-axis by theta */
    Eigen::Matrix4f rotation_matrix_x;
    float cos_theta = 0.0;
    float sin_theta = 1.0;
    // Create a 4x4 matrix
    rotation_matrix_x << \
      1,         0,           0, 0, \
      0, cos_theta, - sin_theta, 0, \
      0, sin_theta,   cos_theta, 0, \
      0,          0,           0, 1;
    //Rotation
    pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_x );
    cloud = cloud_transformed;


    /* Rotation about the y-axis by theta */
    Eigen::Matrix4f rotation_matrix_y;
    cos_theta = 0.0;
    sin_theta = -1.0;
    // Create a 4x4 matrix
      rotation_matrix_y << \
        cos_theta,  0,    sin_theta, 0, \
                0,  1,            0, 0, \
      -sin_theta,  0,    cos_theta, 0, \
                0,  0,            0, 1;
    //Rotation
    // pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_y );
    // cloud = cloud_transformed;

    /* Rotation about the z-axis by theta */
    Eigen::Matrix4f rotation_matrix_z;
    cos_theta = 0.0;
    sin_theta = -1.0;
    // Create a 4x4 matrix
    rotation_matrix_z << \
      cos_theta,  - sin_theta,   0, 0, \
      sin_theta,    cos_theta,   0, 0, \
              0,            0,   1, 0, \
              0,            0,   0, 1;

    //Rotation
    // pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_z );
    // cloud = cloud_transformed;
    return;
  }


  /* Main Function */
  int main(void){
    PointCloud::Ptr cloud (new PointCloud());                 // Point Cloud variable (pointer) for storing point cloud data
    PointCloud::Ptr cloud_transformed (new PointCloud());     // Point Cloud variable (pointer) for storing point cloud data

    /* Load the created PointCloud */
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->data_path_, *cloud) == -1){
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    /* Output the contents of the loaded point cloud data */
    std::cout << "===========================================================\n" << std::endl;
    std::cout << "Loaded\n"
              << cloud->width * cloud->height
              << "data points from test_pcd.pcd with the following fields:\n"
              << std::endl;
    /*
    std::cout << "\tx : [m]\t\ty : [m]\t\tz : [m]" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i){
      std::cout << i+1  << "\tx : "   << cloud->points[i].x
                        << "\ty : "   << cloud->points[i].y
                        << "\tz : "   << cloud->points[i].z << std::endl;
    }
    */
    std::cout << "===========================================================\n" << std::endl;

    /* sensor_msgs */
    sensor_msgs::PointCloud2 sensor_cloud;  // Variable of type sensor_msgs::PointCloud2 for storing point cloud data

    // Convert the PointCloud variable to a sensor_msgs::PointCloud2 variable
    pcl::toROSMsg(*cloud, sensor_cloud);

    cloud->header.frame_id = "camera_rgb_optical_frame";
    sensor_cloud.header.frame_id = "camera_rgb_optical_frame";

    /* Publish the sensor_cloud */
    ros::Rate loop_rate(3);
    while (ros::ok()){
      this->pub_cloud_.publish(cloud);
      this->pub_cloud_sensor_.publish(sensor_cloud);
      loop_rate.sleep();
    }
    return 0;
  }


  public:
    PointcloudPublisherNode()
      : nh_()
      , pnh_("~"){
      ros::param::get("data_path", this->data_path_);
      std::cout << "====================\nLoad Data" << std::endl;
      std::cout << "data_path = "  << this->data_path_ << std::endl;
      std::cout << "====================\n" << std::endl;

      // Definition of the Point Cloud Data Publisher
      this->pub_cloud_sensor_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor_data", 1);
      this->pub_cloud_ = nh_.advertise<PointCloud>("/PointCloud", 1);
      main();
    }
};

int main(int argc, char *argv[]){
  // Initialization of the node
  ros::init(argc, argv, "pointcloud_publisher_node");
  // Creating an instance of PointcloudPublisherNode
  PointcloudPublisherNode pointcloud_publisher;
  ros::spin();
}
