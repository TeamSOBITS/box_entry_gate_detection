#include "rclcpp/rclcpp.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <mutex>

int user_data;
std::mutex user_data_mutex;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 0.0;
    o.y = 0;
    o.z = 0;
    // viewer.addSphere(o, 0.25, "sphere", 0);
    RCLCPP_INFO(rclcpp::get_logger("pcl_visualizer_node"), "i only run once");
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    //FIXED* Possible Race Condition
    std::lock_guard<std::mutex> lock(user_data_mutex);
    user_data++;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pcl_visualizer_node");

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (pcl::io::loadPCDFile("/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/saved_PointCloud_0.pcd", *cloud) == -1) {
        RCLCPP_ERROR(node->get_logger(), "Couldn't read file saved_PointCloud_0.pcd");
        return -1;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer

    //This will only get called once
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

     //This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewerPsycho);
    //FIXED* Possible Race Condition
    while (!viewer.wasStopped()) {
        std::lock_guard<std::mutex> lock(user_data_mutex);
        user_data++;
    }

    rclcpp::shutdown();
    return 0;
}