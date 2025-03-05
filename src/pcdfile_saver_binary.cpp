#include <rclcpp/rclcpp.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/image_encodings.hpp>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/msg/string.hpp>

class PCDFileTranslationNode : public rclcpp::Node
{
public:
    PCDFileTranslationNode() : Node("pcdfile_translation")
    {
        // パラメータの取得
        this->declare_parameter<std::string>("pcdfile", "");
        std::string pcdfile;
        this->get_parameter("pcdfile", pcdfile);

        // PCDファイルのパス設定
        std::stringstream load_file;
        std::stringstream save_file;
        
        load_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/ascii/" << pcdfile;
        
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // PCDファイルを読み込む
        if (pcl::io::loadPCDFile(load_file.str(), cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", load_file.str().c_str());
            return;
        }

        if ((cloud.height * cloud.width) == 0) {
            return;
        }

        save_file << "/home/rg-25/catkin_ws/src/box_entry_gate_detection/pcd/binary/" << pcdfile;

        // PCDファイルを保存
        if (pcl::io::savePCDFile(save_file.str(), cloud, false) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't save file %s", save_file.str().c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------");
        RCLCPP_INFO(this->get_logger(), "%s load.", load_file.str().c_str());
        RCLCPP_INFO(this->get_logger(), "%s saved.", save_file.str().c_str());
        RCLCPP_INFO(this->get_logger(), "------------------------------");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDFileTranslationNode>());
    rclcpp::shutdown();
    return 0;
}
