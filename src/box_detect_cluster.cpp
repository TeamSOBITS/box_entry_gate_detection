#include "rclcpp/rclcpp.hpp" // Include the ROS2 core library
#include <iostream>// Include standard I/O library
#include <stdio.h>// Include standard I/O header
#include <memory>
/* tf2 */
#include <tf2_ros/buffer.h> // Include the TF2 buffer for storing transformations
#include <tf2_ros/transform_listener.h>// Include the transform listener for listening to TF data
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>// Include geometry messages for transformation
#include <tf2_ros/transform_broadcaster.h>// Include the transform broadcaster for sending transformations
/* Point Cloud Library */
/// Include PCL ROS point cloud support
#include <pcl_ros/transforms.hpp>// Include PCL transforms for point clouds
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
#include <std_srvs/srv/set_bool.hpp>// #include "sobits_msgs/RunCtrl.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


class BoxDetection : public rclcpp:: Node {

    private:
  
      /* Node Handle */
      //ros::NodeHandle                           nh_;
      //ros::NodeHandle                           pnh_;
     // rclcpp::Node::SharedPtr nd;
  
      //ros::Subscriber                           cloud_sub_;
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
      
      //ros::Publisher                            pcl_rosmsg_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_rosmsg_;

      //ros::Publisher                            transform_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transform_;
      
      //ros::Publisher                            filter_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filter_pub_;


      //ros::Publisher                            voxel_grid_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;

      //ros::Publisher                            entry_gate_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr entry_gate_pub_;

      //ros::Publisher                            cut_x_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cut_x_pub_;

      //ros::Publisher                            box_clusters_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_clusters_;

      //ros::Publisher                            target_marker_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_marker_;

      //ros::Publisher                            box_marker_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_marker_;
      
  
      //ros::ServiceServer                        run_ctrl_server_;
      rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr run_ctrl_server_;
  
      //tf2_ros::Buffer                           buffer;
      //tf2_ros::TransformListener                tflistener;
      //tf2_ros::TransformBroadcaster             br;
      std::shared_ptr<tf2_ros::Buffer> buffer_;  // Use smart pointer for buffer
      std::shared_ptr<tf2_ros::TransformListener> tflistener_;  // Use smart pointer for listener
      tf2_ros::TransformBroadcaster br_;  // Direct member, or use smart pointer if needed

      //geometry_msgs::TransformStamped           entry_gate;
      geometry_msgs::msg::TransformStamped entry_gate;
  
  
      //PointCloud::Ptr                           cloud_transformed_;
      PointCloud::Ptr cloud_transformed_ = std::make_shared<PointCloud>();

      //PointCloud::Ptr                           boxDetect_cloud_;
      PointCloud::Ptr boxDetect_cloud_ = std::make_shared<PointCloud>();
  
      std::string                               base_frame_name_;
      // std::string                               map_frame_name_;
      std::string                               sub_point_topic_name;
  
      /*  Eigen::Vector */
      Eigen::Vector4f                           min_pt_, max_pt_, center_pt_;
      Eigen::Vector4f                           box_min_pt_, box_max_pt_,box_center_pt_;
      Eigen::Vector4f                           entry_gate_min_pt_, entry_gate_max_pt_, entry_gate_center_pt_;
  
      double                                    depth_x_max_, depth_x_min_, depth_z_max_, depth_z_min_;
      double                                    shift_x_, shift_y_, shift_z_;
      double                                    cluster_ss_;
  
  
      // bool                                      input_port_ok = false;
      bool                                      execute_flag;  //To avoid continuously publishing tf
  
    public:
      BoxDetection() : Node("box_detection")
        //, pnh_("~")
        //, buffer()
        , buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
        //, tflistener(buffer)
        , tflistener_(std::make_shared<tf2_ros::TransformListener>(*buffer_))
        //, br()
        , br_(this)
        , entry_gate()
        //, cloud_transformed_(new PointCloud())
        , cloud_transformed_(std::make_shared<PointCloud>())
        //, boxDetect_cloud_(new PointCloud()){
        , boxDetect_cloud_(std::make_shared<PointCloud>()) {
        execute_flag = false;
      // input_port_ok = false;;
       // load rosparam
       //ros::param::get("/box_entry_gate_detection/execute_default", execute_flag);
       this->declare_parameter("execute_default", false);

       //ros::param::get("/box_entry_gate_detection/depth_range_min_x", depth_x_min_);
       this->declare_parameter("depth_range_min_x", 0.0);

       //ros::param::get("/box_entry_gate_detection/depth_range_max_x", depth_x_max_);
       this->declare_parameter("depth_range_max_x", 0.0);

       //ros::param::get("/box_entry_gate_detection/depth_range_min_z", depth_z_min_);
       this->declare_parameter("depth_range_min_z", 0.0);

       //ros::param::get("/box_entry_gate_detection/depth_range_max_z", depth_z_max_);
       this->declare_parameter("depth_range_max_z", 0.0);

       //ros::param::get("/box_entry_gate_detection/cluster_ss", cluster_ss_);
       this->declare_parameter("cluster_ss", 0.0);

       //ros::param::get("/box_entry_gate_detection/base_frame_name", base_frame_name_);
       this->declare_parameter("base_frame_name", "base_footprint");

      //  ros::param::get("/box_entry_gate_detection/map_frame_name", map_frame_name_);
       //ros::param::get("/box_entry_gate_detection/sub_point_topic_name", sub_point_topic_name);
       this->declare_parameter("sub_point_topic_name", "");
  
       //ros::param::get("/box_entry_gate_detection/shift_x", shift_x_);
       //ros::param::get("/box_entry_gate_detection/shift_y", shift_y_);
       //ros::param::get("/box_entry_gate_detection/shift_z", shift_z_);
       this->declare_parameter("shift_x", 0.0);
       this->declare_parameter("shift_y", 0.0);
       this->declare_parameter("shift_z", 0.0);

       // Get parameters
       execute_flag = this->get_parameter("execute_default").as_bool();
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
       //ROS_INFO("%s",sub_point_topic_name.c_str());
       RCLCPP_INFO(this->get_logger(), "Message: %s", sub_point_topic_name.c_str());

       //cloud_sub_ = nh_.subscribe(sub_point_topic_name, 1, &BoxDetection::DetectPointCb, this);
       cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_point_topic_name, 1, std::bind(&BoxDetection::DetectPointCb, this, std::placeholders::_1));
      
        //run_ctrl_server_ = nh_.advertiseService("run_ctrl", &BoxDetection::run_ctrl_server, this);
        run_ctrl_server_ = this->create_service<std_srvs::srv::SetBool>(
            "run_ctrl", std::bind(&BoxDetection::execute_ctrl_server, this, std::placeholders::_1, std::placeholders::_2));
      //Publisher
       //pcl_rosmsg_          =  nh_.advertise<sensor_msgs::PointCloud2>("/pcl_rosMsg",1);
       //transform_           =  nh_.advertise<sensor_msgs::PointCloud2>("/transform",1);
       //cut_x_pub_           =  nh_.advertise<sensor_msgs::PointCloud2>("/cut_x_cloud",1);
       //filter_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/filter_cloud",1);
       //voxel_grid_pub_      =  nh_.advertise<sensor_msgs::PointCloud2>("/voxel_grid",1);
       //entry_gate_pub_      =  nh_.advertise<sensor_msgs::PointCloud2>("/entry_gate_edge",1);
       //box_clusters_        =  nh_.advertise<visualization_msgs::MarkerArray>("box_cluster", 1);
       //target_marker_       =  nh_.advertise<visualization_msgs::MarkerArray>("box_placeable_point", 1);
       //box_marker_          =  nh_.advertise<visualization_msgs::MarkerArray>("box_point", 1);
       // Publisher
       pcl_rosmsg_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_rosMsg", 1);
       transform_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transform", 1);
       cut_x_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cut_x_cloud", 1);
       filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filter_cloud", 1);
       voxel_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_grid", 1);
       entry_gate_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/entry_gate_edge", 1);
       box_clusters_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_cluster", 1);
       target_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_placeable_point", 1);
       box_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("box_point", 1);

      }
      bool execute_ctrl_server(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        execute_flag = req->data;  // Access the boolean request data
        if (execute_flag) {
            RCLCPP_INFO(this->get_logger(), "Start Box_Detect.");
            res->message = "Start Box_Detect.";  // Set a message in the response
        } else {
            RCLCPP_INFO(this->get_logger(), "Stop Box_Detect.");
            res->message = "Stop Box_Detect.";  // Set a message in the response
        }
        res->success = true;  // Indicate the service call was successful
        return true;
}

  
      //bool run_ctrl_server( sobits_msgs::RunCtrl::Request&  req,
      //                      sobits_msgs::RunCtrl::Response& res) {
      //  execute_flag = req.request;
       // if (execute_flag == true) {
       //   ROS_INFO("Start Box_Detect.");
          // input_port_ok = true;
        //  } 
       // else{
       //   ROS_INFO("Stop Box_Detect.");
          // input_port_ok = false;
      //    }
      //  res.response = true;
      //  return true;
      
      //void DetectPointCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){
        void DetectPointCb(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg) {
          geometry_msgs::msg::TransformStamped tfGeom;
          PointCloud::Ptr cloud_transformed_ = std::make_shared<PointCloud>(); // Initialization
      
          try {
              /* Change sensor_msgs/PointCloud2 to pcl/PointCloud */
              PointCloud from_msg_cloud;
      
              if ((pcl_msg->width * pcl_msg->height) == 0) {
                  return; // Unable to retrieve point cloud data successfully
              }
      
              pcl::fromROSMsg(*pcl_msg, from_msg_cloud);
              sensor_msgs::msg::PointCloud2 ros_cloud_msg;
              pcl::toROSMsg(from_msg_cloud, ros_cloud_msg);
              pcl_rosmsg_->publish(ros_cloud_msg); 
      
              /* Coordinate frame transformation -> Using target_frame as the reference */
              if (!base_frame_name_.empty()) { // Presence/Absence of the Frame
                  try {
                      auto tfGeom = buffer_->lookupTransform(base_frame_name_, from_msg_cloud.header.frame_id, tf2::TimePointZero);
                      pcl_ros::transformPointCloud(base_frame_name_, from_msg_cloud, *cloud_transformed_, *buffer_);
                      cloud_transformed_->header.frame_id = base_frame_name_;
      
                      sensor_msgs::msg::PointCloud2 ros_cloud_transformed_msg;
                      pcl::toROSMsg(*cloud_transformed_, ros_cloud_transformed_msg);
                      transform_->publish(ros_cloud_transformed_msg);
                  } catch (tf2::TransformException &ex) {
                      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                      return;
                  }
              }
          } catch (std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
          }
      
  
        //Processing for point clouds
        //Adjust X-axis range
        //PointCloud::Ptr cloud_cut_x(new PointCloud);
        PointCloud::Ptr cloud_cut_x = std::make_shared<PointCloud>();
        pcl::PassThrough<PointT> pass_x;
        pass_x.setInputCloud (cloud_transformed_);
        pass_x.setFilterFieldName ("x");
        pass_x.setFilterLimits (depth_x_min_, depth_x_max_);
        pass_x.filter (*cloud_cut_x);
        cloud_cut_x->header.frame_id = base_frame_name_;

        // Correctly set the timestamp
        builtin_interfaces::msg::Time time_msg;
        time_msg.sec = this->now().seconds();      // Use this->now()
        time_msg.nanosec = this->now().nanoseconds(); // Use nanoseconds for precision

        // Convert to uint64_t nanoseconds
        uint64_t timestamp_nanosec = (uint64_t)time_msg.sec * 1000000000ull + time_msg.nanosec; // Explicit cast and ull suffix

        cloud_cut_x->header.stamp = timestamp_nanosec;

        //cloud_cut_x->header.stamp = rclcpp::Time(); // Set the timestamp
        //cut_x_pub_.publish(cloud_cut_x);

        // Convert to ROS PointCloud2 before publishing
        sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_cut_x, *ros_cloud); // Convert
        cut_x_pub_->publish(*ros_cloud);        // Publish the ROS2 message

       // cut_x_pub_->publish(*cloud_cut_x);
        //if (cloud_cut_x->points.size() == 0) {
        if (cloud_cut_x->points.empty()) {
          return;
        }
  
        //Adjust Z-axis range
        //PointCloud::Ptr cloud_filtered(new PointCloud());
        PointCloud::Ptr cloud_filtered = std::make_shared<PointCloud>();
        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud (cloud_cut_x);
        pass_z.setFilterFieldName ("z");
        pass_z.setFilterLimits (depth_z_min_, depth_z_max_);
        pass_z.filter (*cloud_filtered);
        cloud_filtered->header.frame_id = base_frame_name_;

        // Correct timestamp assignment for PCLHeader
        builtin_interfaces::msg::Time time_msg_filtered;
        time_msg_filtered.sec = this->now().seconds();
        time_msg_filtered.nanosec = this->now().nanoseconds();
        uint64_t timestamp_nanosec_filtered = (uint64_t)time_msg_filtered.sec * 1000000000ull + time_msg_filtered.nanosec;

        cloud_filtered->header.stamp = timestamp_nanosec_filtered;
        //cloud_filtered->header.stamp = rclcpp::Time(); // Set the timestamp
        //filter_pub_.publish(cloud_filtered);

        sensor_msgs::msg::PointCloud2::SharedPtr filtered_ros_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_filtered, *filtered_ros_cloud);
        filter_pub_->publish(*filtered_ros_cloud);
        //filter_pub_->publish(*cloud_filtered);
        //if (cloud_filtered->points.size() == 0) {
        if (cloud_filtered->points.empty()) {
            return;   
        }
  
  
        //Downsampling
        //PointCloud::Ptr cloud_vg(new PointCloud());
        PointCloud::Ptr cloud_vg = std::make_shared<PointCloud>();
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud (cloud_filtered);
        vg.setLeafSize (0.03, 0.03, 0.03);
        vg.setDownsampleAllData(true);
        vg.filter (*cloud_vg);
        cloud_vg->header.frame_id = base_frame_name_;

        // Correct timestamp assignment for PCLHeader
        builtin_interfaces::msg::Time time_msg_vg;
        time_msg_vg.sec = this->now().seconds();
        time_msg_vg.nanosec = this->now().nanoseconds();
        uint64_t timestamp_nanosec_vg = (uint64_t)time_msg_vg.sec * 1000000000ull + time_msg_vg.nanosec;
        cloud_vg->header.stamp = timestamp_nanosec_vg;

        //cloud_vg->header.stamp = rclcpp::Time(); // Set the timestamp
        //voxel_grid_pub_.publish(cloud_vg);

        // Convert to ROS2 message and publish

        sensor_msgs::msg::PointCloud2::SharedPtr vg_ros_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_vg, *vg_ros_cloud);
        voxel_grid_pub_->publish(*vg_ros_cloud);
        //voxel_grid_pub_->publish(*cloud_vg);
        //if (cloud_vg->points.size() == 0) {
        if (cloud_vg->points.empty()) {
            return;
        }
    
        //Clustering
        //pcl::search::KdTree<PointT>::Ptr box_tree(new pcl::search::KdTree<PointT>);
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
        //visualization_msgs::MarkerArray marker_array;
        visualization_msgs::msg::MarkerArray marker_array;
        int target_index = -1;
  
        //Add color to point clouds
        //pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud <pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*cloud_vg, *cloud_color);
  
        //Visualize clusters
        try{
            //for(std::vector<pcl::PointIndices>::const_iterator it = box_cluster_indices.begin(),
            //                                               it_end = box_cluster_indices.end();
            //                                                  it != it_end; ++it){
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
                      box_center_pt_ = ((max_pt_ - min_pt_) / 2 ) + min_pt_;
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
            //for(std::vector<pcl::PointIndices>::const_iterator it = box_cluster_indices.begin(),
            //                                               it_end = box_cluster_indices.end();
            //                                                  it != it_end; ++it){
            for (const auto& indices : box_cluster_indices) {
              //for(std::vector<int>::const_iterator pit = it-> indices.begin(); pit != it-> indices.end(); pit++){
              for (int index : indices.indices) {
                  //if(box_min_pt_.z() < cloud_vg->points[*pit].z  && cloud_vg->points[*pit].z < box_max_pt_.z()
                  //    && box_min_pt_.y() < cloud_vg->points[*pit].y  && cloud_vg->points[*pit].y < box_max_pt_.y()){
                  if (box_min_pt_.z() < cloud_vg->points[index].z && cloud_vg->points[index].z < box_max_pt_.z() &&
                      box_min_pt_.y() < cloud_vg->points[index].y && cloud_vg->points[index].y < box_max_pt_.y()) {
                        //pcl::getMinMax3D(*cloud_vg, *it, entry_gate_min_pt_,  entry_gate_max_pt_);
                        pcl::getMinMax3D(*cloud_vg, indices, entry_gate_min_pt_,  entry_gate_max_pt_);
                        entry_gate_center_pt_ = ((entry_gate_max_pt_ - entry_gate_min_pt_) / 2 ) + entry_gate_min_pt_;
                        if(target_index == count){
                          //cloud_color-> points[*pit].r = 0;
                          //cloud_color-> points[*pit].g = 255;
                          //cloud_color-> points[*pit].b = 0;
                          cloud_color-> points[index].r = 0;
                          cloud_color-> points[index].g = 255;
                          cloud_color-> points[index].b = 0;
                        }
                        else{
                          //target_marker();
                          //cloud_color-> points[*pit].r = 255;
                          //cloud_color-> points[*pit].g = 0;
                          //cloud_color-> points[*pit].b = 0;
                          cloud_color-> points[index].r = 255;
                          cloud_color-> points[index].g = 0;
                          cloud_color-> points[index].b = 0;                          
                        }
                  }//if
                  else{
                        //cloud_color-> points[*pit].r = 255;
                        //cloud_color-> points[*pit].g = 255;
                        //cloud_color-> points[*pit].b = 255;
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
            }
  
  
      }//try
      catch (const std::exception &e){
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        //ROS_ERROR("%s", e.what());
        //ROS_INFO("in the cluster");
      }
  
      //Free up memory and handle the pointer given as an argument
      //cloud_transformed_.reset(new PointCloud());
      cloud_transformed_ = std::make_shared<PointCloud>();
      //cloud_cut_x.reset(new PointCloud());
      cloud_cut_x = std::make_shared<PointCloud>();

      //cloud_filtered.reset(new PointCloud());
      cloud_filtered = std::make_shared<PointCloud>();

      //cloud_vg.reset(new PointCloud());
      cloud_vg = std::make_shared<PointCloud>();

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
        marker.header.stamp = rclcpp::Clock().now(); 
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
        marker.markers[0].header.stamp = rclcpp::Clock().now();
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
       marker.markers[1].header.stamp = rclcpp::Clock().now();
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
       //marker.markers[2].header.stamp = ros::Time::now();
       marker.markers[2].header.stamp = rclcpp::Clock().now();
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

   //box_marker_.publish(marker);
    box_marker_->publish(marker);

    }//box_marker
    bool send_tf_frame(){ //CORRECT THIS 
        if(execute_flag){
            //geometry_msgs::TransformStamped entry_gate_tf;
            geometry_msgs::msg::TransformStamped entry_gate_tf;
            entry_gate_tf.transform.translation.x = box_center_pt_.x() + shift_x_;
            entry_gate_tf.transform.translation.y = box_center_pt_.y() + shift_y_;
            entry_gate_tf.transform.translation.z = box_center_pt_.z() + shift_z_;
            entry_gate_tf.transform.rotation.x = 0;
            entry_gate_tf.transform.rotation.y = 0;
            entry_gate_tf.transform.rotation.z = 0;
            entry_gate_tf.transform.rotation.w = 1;
            //entry_gate_tf.header.stamp = ros::Time::now();
            entry_gate_tf.header.stamp = rclcpp::Clock().now(); 
            entry_gate_tf.header.frame_id = base_frame_name_;
            entry_gate_tf.child_frame_id = "placeable_point";
            //br.sendTransform(entry_gate_tf); // Broadcast the TF for the cluster
            br_.sendTransform(entry_gate_tf); // Broadcast the TF for the cluster
        } else {
            RCLCPP_INFO(this->get_logger(), "TF stopping");
            //std::cout << "tf stopping" << std::endl;
        }
        return true;
      }//send_tf_frame// COORRECT THIS 
    
      //When no detection occurs
      //void no_detect(std::string msg){
      void no_detect(const std::string &msg){
        //ROS_ERROR("No detect target");
        //std::cout << "Reason:\t" << msg << std::endl;
        RCLCPP_ERROR(this->get_logger(), "No detect target");
        RCLCPP_INFO(this->get_logger(), "Reason: %s", msg.c_str());

      }//no_detect
    };//class BoxDetection
    
    int main(int argc, char *argv[]){

      /* Initialization of the node */
      //ros::init(argc, argv, "box_detection_node");
      rclcpp::init(argc, argv);
      auto box_detection_node = std::make_shared<BoxDetection>();
      //ROS_INFO("Start box_detection.");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start box_detection.");

      /* Creating an instance of the BoxDetection */
      //BoxDetection box_detection_node;
      rclcpp::Rate rate(10);
      while (rclcpp::ok()) {
          box_detection_node->send_tf_frame();//CORRECT THIS
          rate.sleep();
  
          // Correctly cast to rclcpp::Node::SharedPtr
          auto node_ptr = std::static_pointer_cast<rclcpp::Node>(box_detection_node);
          rclcpp::spin_some(node_ptr);
      }
  
      rclcpp::shutdown();
      return 0;
  
  }

   ///   while (ros::ok()){
   //     box_detection_node.send//tf_frame();
   //     ros::Duration(0.1).sleep();
   //     ros::spinOnce();
   //   }


                




  
