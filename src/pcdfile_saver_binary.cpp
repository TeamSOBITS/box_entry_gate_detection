#include <rclcpp/rclcpp.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

class PCDFileTranslationNode : public rclcpp::Node {
public:
    PCDFileTranslationNode() : Node("pcdfile_translation") {
        // Declare and get the parameter
        this->declare_parameter<std::string>("pcdfile", "");
        std::string pcdfile;
        this->get_parameter("pcdfile", pcdfile);

        if (pcdfile.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'pcdfile' is empty. Please provide a valid file name.");
            return;
        }

        // Set up file paths
        std::stringstream load_file;
        std::stringstream save_file;
        load_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/ascii/" << pcdfile;
        save_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/binary/" << pcdfile;

        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Load the PCD file
        if (pcl::io::loadPCDFile(load_file.str(), cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", load_file.str().c_str());
            return;
        }

        if ((cloud.height * cloud.width) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Loaded point cloud is empty.");
            return;
        }

        // Save the PCD file
        if (pcl::io::savePCDFile(save_file.str(), cloud, false) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't save file %s", save_file.str().c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------");
        RCLCPP_INFO(this->get_logger(), "%s loaded.", load_file.str().c_str());
        RCLCPP_INFO(this->get_logger(), "%s saved.", save_file.str().c_str());
        RCLCPP_INFO(this->get_logger(), "------------------------------");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDFileTranslationNode>());
    rclcpp::shutdown();
    return 0;
}
