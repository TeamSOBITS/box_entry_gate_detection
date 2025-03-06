#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>

class PclTopicSaver : public rclcpp::Node {
public:
    PclTopicSaver()
        : Node("pcl_topic_saver"), proc_count_(0), buffer_(this->get_clock()), listener_(buffer_) {
        sub_point_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10, std::bind(&PclTopicSaver::pointCallback, this, std::placeholders::_1));
    }

private:
    void pointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
        auto current_time = this->get_clock()->now();
        if ((current_time - rclcpp::Time(input->header.stamp)) > rclcpp::Duration(1, 0)) {
            RCLCPP_INFO(this->get_logger(), "Point cloud skip");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_input);

        std::string filename = "saved_PointCloud_" + std::to_string(proc_count_) + ".pcd";
        pcl::io::savePCDFileASCII(filename, *cloud_input);
        RCLCPP_INFO(this->get_logger(), "Saved pcd file: %s", filename.c_str());
        proc_count_++;

        RCLCPP_INFO(this->get_logger(), "Press Enter Key!");
        std::string temp_str;
        std::getline(std::cin, temp_str);
        RCLCPP_INFO(this->get_logger(), "%s", temp_str.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_;
    int proc_count_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto pcl_topic_saver = std::make_shared<PclTopicSaver>();
    RCLCPP_INFO(pcl_topic_saver->get_logger(), "Start topic_saver.");
    rclcpp::spin(pcl_topic_saver);
    rclcpp::shutdown();
    return 0;
}