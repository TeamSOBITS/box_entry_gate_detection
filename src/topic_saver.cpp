#include <time.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>
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

class pcl_topic_saver
{
public:
    pcl_topic_saver()
        : nh()
        , proc_count(0)
        , listener(buffer)
        , sub_point(nh.subscribe("/camera/depth/points", 1, &pcl_topic_saver::point_cb, this))
    {
    }

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        if (input->header.stamp + ros::Duration(1.0) < ros::Time::now())
        {
            ROS_INFO_STREAM("Point cloud skip");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_input);

        pcl::io::savePCDFileASCII("saved_PointCloud_" + std::to_string(proc_count) + ".pcd", *cloud_input);
        ROS_INFO_STREAM("Saved pcd file: saved_PointCloud_" + std::to_string(proc_count) + ".pcd");
        proc_count++;

        ROS_INFO("Press Enter Key!");
        std::string temp_str;
        std::getline(std::cin, temp_str);
        ROS_INFO_STREAM(temp_str);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_point;
    int proc_count;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "topic_saver");
    ROS_INFO("Start topic_saver.");
    pcl_topic_saver psc;
    ros::spin();
    return 0;
}
