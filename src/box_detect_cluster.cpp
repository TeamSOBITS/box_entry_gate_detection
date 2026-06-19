#include "rclcpp/rclcpp.hpp" // Include the ROS2 core library
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <iostream>// Include standard I/O library
#include <stdio.h>// Include standard I/O header
#include <memory>
/* tf2 */
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h> // Include the TF2 buffer for storing transformations
#include <tf2_ros/transform_listener.h>// Include the transform listener for listening to TF data
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>// Include geometry messages for transformation
#include <tf2_ros/transform_broadcaster.h>// Include the transform broadcaster for sending transformations
/* Point Cloud Library */
/// Include PCL ROS point cloud support
#include <pcl_ros/transforms.hpp>// Include PCL transforms for point clouds
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>// Include PCL point types
#include <pcl/filters/passthrough.h>// Include passthrough filter for point clouds
#include <pcl/filters/voxel_grid.h> // Include voxel grid filter for downsampling point clouds
#include <pcl/filters/extract_indices.h>// Include filters for extracting indices from point clouds
#include <pcl/common/common.h>// Include common operations on point clouds
#include <pcl/kdtree/kdtree.h>// Include K-D tree for nearest neighbor search
#include <pcl/segmentation/extract_clusters.h>// Include cluster extraction methods
#include <pcl/segmentation/sac_segmentation.h> // Include RANSAC segmentation methods
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
/* rviz */
#include <visualization_msgs/msg/marker_array.hpp>// Include marker array for visualization in RViz

#include <cmath>// Include math functions
#include <chrono>// Include time measurement utilities

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class BoxDetection : public rclcpp_lifecycle::LifecycleNode {

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr entry_gate_pub_;
    rclcpp::QoS qos_profile_; // depth = 1

    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_clusters_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_marker_;
    std::shared_ptr<tf2_ros::Buffer>               tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tfListener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    geometry_msgs::msg::TransformStamped entry_gate;

    PointCloud::Ptr cloud_transformed_ = std::make_shared<PointCloud>();
    PointCloud::Ptr boxDetect_cloud_ = std::make_shared<PointCloud>();

    std::string                               base_frame_name_;
    std::string                               sub_point_topic_name;

    /*  Eigen::Vector */
    Eigen::Vector4f                           min_pt_, max_pt_, center_pt_;
    Eigen::Vector4f                           box_min_pt_, box_max_pt_,box_center_pt_;
    Eigen::Vector4f                           entry_gate_min_pt_, entry_gate_max_pt_, entry_gate_center_pt_;
    double                                    depth_x_max_, depth_x_min_, depth_z_max_, depth_z_min_;
    double                                    shift_x_, shift_y_, shift_z_;
    double                                    cluster_ss_;
    bool                                      active_;

  public:
    explicit BoxDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
      rclcpp_lifecycle::LifecycleNode("box_detection", options),
      entry_gate(), 
      cloud_transformed_(std::make_shared<PointCloud>()), 
      boxDetect_cloud_(std::make_shared<PointCloud>()),
      qos_profile_(rclcpp::QoS(1)) {
        active_ = false;
        this->declare_parameter("depth_range_min_x", 0.0);
        this->declare_parameter("depth_range_max_x", 0.0);
        this->declare_parameter("depth_range_min_z", 0.0);
        this->declare_parameter("depth_range_max_z", 0.0);
        this->declare_parameter("cluster_ss", 0.0);
        this->declare_parameter("base_frame_name", "base_footprint");
        this->declare_parameter("sub_point_topic_name", "");
        this->declare_parameter("shift_x", 0.0);                                                                                        
        this->declare_parameter("shift_y", 0.0);
        this->declare_parameter("shift_z", 0.0);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        // Get parameters
        depth_x_min_ = this->get_parameter("depth_range_min_x").as_double();
        depth_x_max_ = this->get_parameter("depth_range_max_x").as_double();
        depth_z_min_ = this->get_parameter("depth_range_min_z").as_double();
        depth_z_max_ = this->get_parameter("depth_range_max_z").as_double();
        cluster_ss_ = this->get_parameter("cluster_ss").as_double();
        base_frame_name_ = this->get_parameter("base_frame_name").as_string();
        sub_point_topic_name = this->get_parameter("sub_point_topic_name").as_string();
        shift_x_ = this->get_parameter("shift_x").as_double();
        shift_y_ = this->get_parameter("shift_y").as_double();
        shift_z_ = this->get_parameter("shift_z").as_double();

      // Create a ROS subscriber and publisher
      RCLCPP_INFO(this->get_logger(), "Message: %s", sub_point_topic_name.c_str());
      qos_profile_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      qos_profile_.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
      qos_profile_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

      tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
      tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      entry_gate_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/entry_gate_edge", 1);
      box_clusters_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_cluster", 1);
      target_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_placeable_point", 1);
      box_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_point", 1);

      RCLCPP_INFO(this->get_logger(), "Configured box_detection lifecycle node.");
      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Activating box_detection lifecycle node.");
      active_ = true;

      entry_gate_pub_->on_activate();
      box_clusters_->on_activate();
      target_marker_->on_activate();
      box_marker_->on_activate();

      cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          sub_point_topic_name, qos_profile_, std::bind(&BoxDetection::DetectPointCb, this, std::placeholders::_1));

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Deactivating box_detection lifecycle node.");
      active_ = false;
      cloud_sub_.reset();

      if (entry_gate_pub_) entry_gate_pub_->on_deactivate();
      if (box_clusters_) box_clusters_->on_deactivate();
      if (target_marker_) target_marker_->on_deactivate();
      if (box_marker_) box_marker_->on_deactivate();

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Cleaning up box_detection lifecycle node.");
      active_ = false;
      cloud_sub_.reset();
      entry_gate_pub_.reset();
      box_clusters_.reset();
      target_marker_.reset();
      box_marker_.reset();
      tfBroadcaster_.reset();
      tfListener_.reset();
      tfBuffer_.reset();

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
      RCLCPP_INFO(this->get_logger(), "Shutting down box_detection lifecycle node.");
      active_ = false;
      cloud_sub_.reset();
      entry_gate_pub_.reset();
      box_clusters_.reset();
      target_marker_.reset();
      box_marker_.reset();
      tfBroadcaster_.reset();
      tfListener_.reset();
      tfBuffer_.reset();

      return CallbackReturn::SUCCESS;
    }

  void DetectPointCb(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
    if (!active_) {
      return;
    }

    PointCloud::Ptr cloud            (new PointCloud());
    PointCloud::Ptr cloud_object     (new PointCloud());
    // Transform lookup
    pcl::fromROSMsg(*pcl_msg, *cloud);

    if (!tfBuffer_->canTransform(base_frame_name_, pcl_msg->header.frame_id, rclcpp::Time(0), std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "Waiting for transform from %s to %s...",
                    pcl_msg->header.frame_id.c_str(), base_frame_name_.c_str());
        return;
    }    

    auto transform_stamped = tfBuffer_->lookupTransform(
        base_frame_name_, pcl_msg->header.frame_id, tf2::TimePointZero);

    // Convert to Eigen Matrix
    Eigen::Isometry3d transform_iso = tf2::transformToEigen(transform_stamped.transform);
    Eigen::Matrix4f transform_matrix = transform_iso.matrix().cast<float>();

    // Transform the point cloud
    PointCloud::Ptr transformed_cloud(new PointCloud);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);
    pcl::PassThrough<PointT> pass_x;
    pcl::PassThrough<PointT> pass_z;
    PointCloud::Ptr filtered_x_cloud(new PointCloud);
    PointCloud::Ptr filtered_z_cloud(new PointCloud);
    pass_x.setFilterFieldName( "x" );
    pass_x.setFilterLimits( depth_x_min_, depth_x_max_);
    pass_x.setInputCloud(transformed_cloud);
    try {
        pass_x.filter(*filtered_x_cloud);
        filtered_x_cloud->header.frame_id = base_frame_name_;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return ;
    }
    pass_z.setInputCloud(filtered_x_cloud);
    pass_z.setFilterFieldName( "z" );
    pass_z.setFilterLimits( depth_z_min_, depth_z_max_);
    try {
        pass_z.filter(*filtered_z_cloud);
        filtered_z_cloud->header.frame_id = base_frame_name_;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return ;
    }

      //Downsampling
      PointCloud::Ptr cloud_vg(new PointCloud);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud (filtered_z_cloud);
      vg.setLeafSize (0.03, 0.03, 0.03);
      vg.setDownsampleAllData(true);
      vg.filter (*cloud_vg);
      cloud_vg->header.frame_id = base_frame_name_;
  
      //Clustering
      pcl::search::KdTree<PointT>::Ptr box_tree = std::make_shared<pcl::search::KdTree<PointT>>();
      box_tree-> setInputCloud(cloud_vg);
      std::vector<pcl::PointIndices> box_cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (cluster_ss_); // 5cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (box_tree);
      ec.setInputCloud(cloud_vg);
      ec.extract (box_cluster_indices);

      //Visualization
      visualization_msgs::msg::MarkerArray marker_array;
      int target_index = -1;

      //Add color to point clouds
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      pcl::copyPointCloud(*cloud_vg, *cloud_color);

      //Visualize clusters
      try{
          for (const auto& indices : box_cluster_indices){
            //pcl::getMinMax3D(*cloud_vg, *it, min_pt_,  max_pt_);
            pcl::getMinMax3D(*cloud_vg, indices, min_pt_,  max_pt_);
            Eigen::Vector4f cluster_size =  max_pt_ - min_pt_;
            center_pt_ = ((max_pt_ - min_pt_) / 2 ) + min_pt_;
            if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
                //visualization_msgs::Marker marker =  makeMarker(base_frame_name_, "box", min_pt_, max_pt_, 0.0f, 1.0f, 0.0f, 0.5f);
                visualization_msgs::msg::Marker marker =  makeMarker(base_frame_name_, "box", min_pt_, max_pt_, 0.0f, 1.0f, 0.0f, 0.5f);
                  // Detect the nearest cluster
                  if(target_index < 0){
                    target_index = marker_array.markers.size();
                    box_max_pt_ = max_pt_;
                    box_min_pt_ = min_pt_;
                    box_center_pt_ = center_pt_;
                  }
                  else{
                    // d1 : Distance to the cluster with the target index
                    float d1 = sqrt(pow( marker_array.markers[target_index].pose.position.x , 2) + pow( marker_array.markers[target_index].pose.position.y , 2));
                    // d2 : Distance to the cluster that matches the newly detected specific size
                    float d2 = sqrt(pow( marker.pose.position.x , 2) + pow( marker.pose.position.y , 2));
                    if(d2 < d1){
                      target_index = marker_array.markers.size();
                      box_max_pt_ = max_pt_;
                      box_min_pt_ = min_pt_;
                      box_center_pt_ = center_pt_ ;
                    }
                  }//else
                  marker_array.markers.push_back(marker); 
            }//if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
            else{
              no_detect("No detect the target : xtion");
              return;
            }
          }//for (box_cluster)
        }//try
        catch (const std::exception &e){
          RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }

        //Use only the point cloud within the closest cluster
        try{
          int count = 0;
          for (const auto& indices : box_cluster_indices) {
            pcl::getMinMax3D(*cloud_vg, indices, entry_gate_min_pt_,  entry_gate_max_pt_);
            for (int index : indices.indices) {
                if (box_min_pt_.z() < cloud_vg->points[index].z && cloud_vg->points[index].z < box_max_pt_.z() &&
                    box_min_pt_.y() < cloud_vg->points[index].y && cloud_vg->points[index].y < box_max_pt_.y()) {
                      entry_gate_center_pt_ = ((entry_gate_max_pt_ - entry_gate_min_pt_) / 2 ) + entry_gate_min_pt_;
                      if(target_index == count){
                        cloud_color-> points[index].r = 0;
                        cloud_color-> points[index].g = 255;
                        cloud_color-> points[index].b = 0;
                      }
                      else{

                        cloud_color-> points[index].r = 255;
                        cloud_color-> points[index].g = 0;
                        cloud_color-> points[index].b = 0;                          
                      }
                }//if
                else{
                      cloud_color-> points[index].r = 255;
                      cloud_color-> points[index].g = 255;
                      cloud_color-> points[index].b = 255;                             
                }//else
            }//for(pit)
                  count++;
          }//for(it)

          //if(marker_array.markers.empty() == false){
          if (!marker_array.markers.empty()) {
            if(target_index >= 0){
              marker_array.markers[target_index].ns = "target_clusters";
              marker_array.markers[target_index].color.r = 0.0f;
              marker_array.markers[target_index].color.g = 0.0f;
              marker_array.markers[target_index].color.b = 1.0f;
              marker_array.markers[target_index].color.a = 0.5f;
              box_marker();
            }
              //box_clusters_.publish(marker_array);
              //entry_gate_pub_.publish(cloud_color);
              box_clusters_->publish(marker_array);
              // Convert and publish the colored point cloud
              sensor_msgs::msg::PointCloud2::SharedPtr color_cloud_msg(new sensor_msgs::msg::PointCloud2);
              pcl::toROSMsg(*cloud_color, *color_cloud_msg);
              entry_gate_pub_->publish(*color_cloud_msg);                
              //entry_gate_pub_->publish(*cloud_color);
              send_tf_frame();
          }


    }//try
    catch (const std::exception &e){
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }

    //box_tree.reset(new pcl::search::KdTree<PointT>());
    box_tree = std::make_shared<pcl::search::KdTree<PointT>>();


  }//DetectPointCb
  // Create a Marker for displaying the cluster as a cuboid
  //visualization_msgs::Marker makeMarker(
  visualization_msgs::msg::Marker makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,const
    float r, float g, float b, float a) const{

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    //marker.header.stamp = ros::Time::now();
    marker.header.stamp = this->now(); 
    marker.ns = marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    //marker.lifetime = ros::Duration(0.5);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5); 
    return marker;
  }
  
  void box_marker(){
    //visualization_msgs::MarkerArray marker;
    visualization_msgs::msg::MarkerArray marker;
    std::string   points_ns[3] = {"box_min_pt","box_max_pt","box_center_pt"};

    marker.markers.resize(3);
    marker.markers[0].header.frame_id = base_frame_name_;
    //marker.markers[0].header.stamp = ros::Time::now();
    marker.markers[0].header.stamp = this->now();
    marker.markers[0].action = visualization_msgs::msg::Marker::ADD;
    marker.markers[0].type = visualization_msgs::msg::Marker::SPHERE;
    marker.markers[0].ns = points_ns[0];
    marker.markers[0].id = 4;
    marker.markers[0].color.a = 1.0;
    marker.markers[0].color.r = 1.0;
    marker.markers[0].color.g = 0.0;
    marker.markers[0].color.b = 0.5;
    marker.markers[0].scale.x = 0.01;//[m]
    marker.markers[0].scale.y = 0.01;//[m]
    marker.markers[0].scale.z = 1.0;//[m]
    marker.markers[0].pose.position.x = box_min_pt_.x();//min_pt_[divide].x()
    marker.markers[0].pose.position.y = box_min_pt_.y();//min_pt_.y()
    marker.markers[0].pose.position.z = box_min_pt_.z();//min_pt_[divide].z()
    marker.markers[0].pose.orientation.x = 0;
    marker.markers[0].pose.orientation.y = -0.55;//1
    marker.markers[0].pose.orientation.z = 0;
    marker.markers[0].pose.orientation.w = 1;

    marker.markers[1].header.frame_id = base_frame_name_;
    //marker.markers[1].header.stamp = ros::Time::now();
    marker.markers[1].header.stamp = this->now();
    marker.markers[1].action = visualization_msgs::msg::Marker::ADD;
    marker.markers[1].type = visualization_msgs::msg::Marker::SPHERE;
    marker.markers[1].ns = points_ns[1];
    marker.markers[1].id = 5;
    marker.markers[1].color.a = 1.0;
    marker.markers[1].color.r = 1.0;
    marker.markers[1].color.g = 0.6;
    marker.markers[1].color.b = 0.0;
    marker.markers[1].scale.x = 0.01;//[m]
    marker.markers[1].scale.y = 0.01;//[m]
    marker.markers[1].scale.z = 1.0;//[m]
    marker.markers[1].pose.position.x = box_max_pt_.x();//max_pt_[divide].x()
    marker.markers[1].pose.position.y = box_max_pt_.y();//max_pt_[divide].y()
    marker.markers[1].pose.position.z = box_max_pt_.z();//max_pt_[divide].z()
    marker.markers[1].pose.orientation.x = 0;
    marker.markers[1].pose.orientation.y = -0.55;//1
    marker.markers[1].pose.orientation.z = 0;
    marker.markers[1].pose.orientation.w = 1;

    marker.markers[2].header.frame_id = base_frame_name_;
    marker.markers[2].header.stamp = this->now();
    marker.markers[2].action = visualization_msgs::msg::Marker::ADD;
    marker.markers[2].type = visualization_msgs::msg::Marker::SPHERE;
    marker.markers[2].ns = points_ns[2];
    marker.markers[2].id = 6;
    marker.markers[2].color.a = 1.0;
    marker.markers[2].color.r = 0.5;
    marker.markers[2].color.g = 0.6;
    marker.markers[2].color.b = 0.0;
    marker.markers[2].scale.x = 0.01;//[m]
    marker.markers[2].scale.y = 0.01;//[m]
    marker.markers[2].scale.z = 1.0;//[m]
    marker.markers[2].pose.position.x = box_center_pt_.x();//max_pt_[divide].x()
    marker.markers[2].pose.position.y = box_center_pt_.y();//max_pt_[divide].y()
    marker.markers[2].pose.position.z = box_center_pt_.z();//max_pt_[divide].z()
    marker.markers[2].pose.orientation.x = 0;
    marker.markers[2].pose.orientation.y = -0.55;//1
    marker.markers[2].pose.orientation.z = 0;
    marker.markers[2].pose.orientation.w = 1;
    box_marker_->publish(marker);
  }
  bool send_tf_frame(){ 
    geometry_msgs::msg::TransformStamped entry_gate_tf;
    entry_gate_tf.transform.translation.x = box_center_pt_.x() + shift_x_;
    entry_gate_tf.transform.translation.y = box_center_pt_.y() + shift_y_;
    entry_gate_tf.transform.translation.z = box_center_pt_.z() + shift_z_;
    entry_gate_tf.transform.rotation.x = 0;
    entry_gate_tf.transform.rotation.y = 0;
    entry_gate_tf.transform.rotation.z = 0;
    entry_gate_tf.transform.rotation.w = 1;
    entry_gate_tf.header.stamp = this->now(); 
    entry_gate_tf.header.frame_id = base_frame_name_;
    entry_gate_tf.child_frame_id = "placeable_point";
    tfBroadcaster_->sendTransform(entry_gate_tf);
    return true;
  }
  
  //When no detection occurs
  void no_detect(const std::string &msg){
    RCLCPP_ERROR(this->get_logger(), "No detect target");
    RCLCPP_INFO(this->get_logger(), "Reason: %s", msg.c_str());
  }
};
    
int main(int argc, char *argv[]){
  /* Initialization of the node */
  rclcpp::init(argc, argv);
  auto box_detection_node = std::make_shared<BoxDetection>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start box_detection.");

  rclcpp::spin(box_detection_node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;

}
