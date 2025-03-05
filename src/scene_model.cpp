#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT; // PointT for point cloud
typedef pcl::PointCloud<PointT> PointCloud;

class PointcloudPublisherNode : public rclcpp::Node
{
public:
    PointcloudPublisherNode()
        : Node("pointcloud_publisher_node"), proc_count(0)
    {
        // Parameter for PCD file path
        this->declare_parameter<std::string>("data_path", "");
        this->get_parameter("data_path", data_path_);

        RCLCPP_INFO(this->get_logger(), "====================");
        RCLCPP_INFO(this->get_logger(), "Load Data from: %s", data_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "====================");

        // Publisher for PointCloud and PointCloud2
        pub_cloud_sensor_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_data", 1);

        // Load PointCloud from PCD file
        loadPointCloud();

        // Spin to keep the node alive and publish data
        rclcpp::WallRate loop_rate(3); // 3Hz
        while (rclcpp::ok()) {
            pub_cloud_sensor_->publish(sensor_cloud_);
            loop_rate.sleep();
        }
    }

private:
    void loadPointCloud()
    {
        cloud_ = std::make_shared<PointCloud>();
        // Load the PCD file
        if (pcl::io::loadPCDFile<PointT>(data_path_, *cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", data_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu data points", cloud_->points.size());

        // Convert to PointCloud2 format
        pcl::toROSMsg(*cloud_, sensor_cloud_);
        sensor_cloud_.header.frame_id = "camera_rgb_optical_frame";
    }

    std::string data_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_sensor_;
    // rclcpp::Publisher<PointCloud>::SharedPtr pub_cloud_;
    sensor_msgs::msg::PointCloud2 sensor_cloud_;
    PointCloud::Ptr cloud_;
    int proc_count;
};

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and run the PointcloudPublisherNode
    rclcpp::spin(std::make_shared<PointcloudPublisherNode>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
