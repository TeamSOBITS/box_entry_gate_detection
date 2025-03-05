#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
// #include <tf2_ros/transform_listener.hpp>
#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <string>

class PclTopicSaver : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_;
    int proc_count;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
public:
    PclTopicSaver()
        : Node("topic_saver"), proc_count(0), buffer_(this->get_clock()), listener_(buffer_)
    {
        sub_point_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10, std::bind(&PclTopicSaver::point_cb, this, std::placeholders::_1));
    }

    void point_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        if ((input->header.stamp.sec + std::chrono::seconds(1).count()) < this->now().seconds())
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud skip");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_input);

        pcl::io::savePCDFileASCII("saved_PointCloud_" + std::to_string(proc_count) + ".pcd", *cloud_input);
        // RCLCPP_INFO(this->get_logger(), "Saved pcd file: saved_PointCloud_" + std::to_string(proc_count) + ".pcd");
        RCLCPP_INFO(this->get_logger(), "Saved pcd file: saved_PointCloud_%d.pcd", proc_count);
        proc_count++;

        RCLCPP_INFO(this->get_logger(), "Press Enter Key!");
        std::string temp_str;
        std::getline(std::cin, temp_str);
        RCLCPP_INFO(this->get_logger(), temp_str.c_str());
    }


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start topic_saver.");
    auto pcl_topic_saver = std::make_shared<PclTopicSaver>();
    rclcpp::spin(pcl_topic_saver);
    rclcpp::shutdown();
    return 0;
}
