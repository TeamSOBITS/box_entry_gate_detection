#include <time.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>

//#include <typeinfo.h>


class pcl_topic_saver
{

public:
    pcl_topic_saver(){
        this->proc_count = 0;//init
        this->sub_point = nh.subscribe("/camera/depth/points", 1, &pcl_topic_saver::point_cb, this);
    }//pcl_topic_saver

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& input){
		//ROS_INFO_STREAM("point_cloud get.");
        if( input->header.stamp + ros::Duration(1.0) < ros::Time::now() ){ ROS_INFO_STREAM("ponit_cloud skip"); return; }//古いトピックは使わない
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_input);
        pcl::io::savePCDFileASCII ("saved_PointCloud_" + std::to_string(this->proc_count) + ".pcd", *cloud_input);
		    std::cout << "OK, saved pcd file. : " << "saved_PointCloud_" + std::to_string(this->proc_count) + ".pcd" << std::endl;
        this->proc_count++;
    		std::cout << "Press Enter Key!" << std::endl;
    		std::string temp_str;
    		std::getline(std::cin, temp_str);
        std::cout << temp_str << std::endl;
        return;
    }//pub_cloud_func

private:

    ros::NodeHandle nh;
    ros::Subscriber sub_point;
    int proc_count;
    tf::TransformListener listerner;
    std::string sub_point_topic_name;
};


int main(int argc, char** argv){

    ros::init(argc, argv, "topic_saver");
    ROS_INFO("Start topic_saver.");
    pcl_topic_saver psc;
    ros::spin();

    return 0;
}//main
