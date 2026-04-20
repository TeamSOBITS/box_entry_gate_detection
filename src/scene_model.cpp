#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PointcloudPublisherNode : public rclcpp::Node {
public:
    PointcloudPublisherNode() : Node("pointcloud_publisher_node") {
        // Declare and get parameters
        this->declare_parameter<std::string>("data_path", "test_pcd.pcd");
        this->declare_parameter<bool>("apply_rotation", false);
        this->get_parameter("data_path", data_path_);
        this->get_parameter("apply_rotation", apply_rotation_);

        RCLCPP_INFO(this->get_logger(), "====================");
        RCLCPP_INFO(this->get_logger(), "Load Data from: %s", data_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Apply Rotation: %s", apply_rotation_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "====================");

        // Create publisher
        pub_cloud_sensor_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_data", 1);

        // Load and publish point cloud
        loadPointCloud();
    }

private:
    void loadPointCloud() {
        PointCloud::Ptr cloud = std::make_shared<PointCloud>();

        // Load the PCD file
        if (pcl::io::loadPCDFile<PointT>(data_path_, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", data_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu data points", cloud->points.size());

        // Apply rotation if needed
        if (apply_rotation_) {
            rotatePointCloud(cloud);
        }

        // Convert to PointCloud2 format
        sensor_msgs::msg::PointCloud2 sensor_cloud;
        pcl::toROSMsg(*cloud, sensor_cloud);
        sensor_cloud.header.frame_id = "camera_rgb_optical_frame";

        // Publish the sensor_cloud
        rclcpp::Rate loop_rate(3);
        while (rclcpp::ok()) {
            pub_cloud_sensor_->publish(sensor_cloud);
            loop_rate.sleep();
        }
    }

    void rotatePointCloud(PointCloud::Ptr cloud) {
        PointCloud::Ptr cloud_transformed = std::make_shared<PointCloud>();

        // Rotation about the x-axis by theta
        Eigen::Matrix4f rotation_matrix_x;
        float cos_theta_x = 0.0;
        float sin_theta_x = 1.0;
        rotation_matrix_x << 
            1,         0,           0, 0,
            0, cos_theta_x, -sin_theta_x, 0,
            0, sin_theta_x,  cos_theta_x, 0,
            0,         0,           0, 1;
        pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_x);
        *cloud = *cloud_transformed;

        // Rotation about the y-axis by theta
        Eigen::Matrix4f rotation_matrix_y;
        float cos_theta_y = 0.0;
        float sin_theta_y = -1.0;
        rotation_matrix_y << 
            cos_theta_y,  0, sin_theta_y, 0,
                    0,  1,         0, 0,
           -sin_theta_y,  0, cos_theta_y, 0,
                    0,  0,         0, 1;
        pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_y);
        *cloud = *cloud_transformed;

        // Rotation about the z-axis by theta
        Eigen::Matrix4f rotation_matrix_z;
        float cos_theta_z = 0.0;
        float sin_theta_z = -1.0;
        rotation_matrix_z << 
            cos_theta_z, -sin_theta_z, 0, 0,
            sin_theta_z,  cos_theta_z, 0, 0,
                    0,          0, 1, 0,
                    0,          0, 0, 1;
        pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_z);
        *cloud = *cloud_transformed;
    }

    std::string data_path_;
    bool apply_rotation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_sensor_;
};

int main(int argc, char *argv[]) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and run the PointcloudPublisherNode
    rclcpp::spin(std::make_shared<PointcloudPublisherNode>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}