#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
/* tf */
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
/* rviz */
#include <visualization_msgs/MarkerArray.h>
/*平方根*/
#include <cmath>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class BoxDetection{

  private:

    /* Node Handle */
    ros::NodeHandle                           nh_;                              //pub/sub
    ros::NodeHandle                           pnh_;                             //param

    ros::Subscriber                           cloud_sub_;
    ros::Publisher                            box_pub_;
    ros::Publisher                            pcl_rosmsg_;
    ros::Publisher                            transform_;
    ros::Publisher                            filter_pub_;
    ros::Publisher                            voxel_grid_pub_;
    ros::Publisher                            entry_gate_pub_;
    ros::Publisher                            cut_x_pub_;
    ros::Publisher                            box_clusters_;
    ros::Publisher                            target_marker_;
    ros::Publisher                            box_marker_;


    tf::TransformListener                     listener;
    tf::TransformBroadcaster                  br;
    tf::Transform                             entry_gate;


    PointCloud::Ptr                           cloud_transformed_;
    PointCloud::Ptr                           boxDetect_cloud_;
    PointCloud::Ptr                           cloud_mode_;

    std::string                               base_frame_name_;
    std::string                               points_topic_name_;
    /*  Eigen::Vector */
    Eigen::Vector4f                           min_pt_, max_pt_;
    Eigen::Vector4f                           box_min_pt_, box_max_pt_,box_center_pt_;
    Eigen::Vector4f                           entry_gate_min_pt_, entry_gate_max_pt_, entry_gate_center_pt_;

    float                                     depth_x_max_, depth_x_min_, depth_z_max_, depth_z_min_;


  public:
    BoxDetection()
      :nh_()
      ,pnh_("~")
      ,boxDetect_cloud_(new PointCloud()){
     // load rosparam
     ros::param::get("depth_range_min_x", depth_x_min_);
     ros::param::get("depth_range_max_x", depth_x_max_);
     ros::param::get("depth_range_min_z", depth_z_min_);
     ros::param::get("depth_range_max_z", depth_z_max_);
     ros::param::get("base_frame_name", base_frame_name_);

     base_frame_name_ = "base_footprint";//default
     points_topic_name_ = "/sensor_data";

     ROS_INFO("0");
     cloud_sub_ = nh_.subscribe(points_topic_name_, 1, &BoxDetection::DetectPointCb, this);

     //エラーの時
     if(!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(points_topic_name_, ros::Duration(3.0))){
         ROS_ERROR("timeout");
         exit(EXIT_FAILURE);
       }
    /*Publisher*/
     box_pub_             =  nh_.advertise<sensor_msgs::PointCloud2>("/box_detection",1);
     pcl_rosmsg_          =  nh_.advertise<sensor_msgs::PointCloud2>("/pcl_rosMsg",1);
     transform_           =  nh_.advertise<sensor_msgs::PointCloud2>("/transform",1);
     cut_x_pub_           =  nh_.advertise<sensor_msgs::PointCloud2>("/cut_x_cloud",1);
     filter_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/filter_cloud",1);
     voxel_grid_pub_      =  nh_.advertise<sensor_msgs::PointCloud2>("/voxel_grid",1);
     entry_gate_pub_      =  nh_.advertise<sensor_msgs::PointCloud2>("/entry_gate_edge",1);
     box_clusters_        =  nh_.advertise<visualization_msgs::MarkerArray>("box_cluster", 1);
     target_marker_       =  nh_.advertise<visualization_msgs::MarkerArray>("target_point", 1);
     box_marker_          =  nh_.advertise<visualization_msgs::MarkerArray>("box_point", 1);
    }


  void DetectPointCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){
      PointCloud::Ptr cloud_transformed_(new PointCloud());//初期化
      try {
           /* sensor_msgs/PointCloud2データ -> pcl/PointCloudに変換 */
           PointCloud from_msg_cloud;
           pcl::fromROSMsg(*pcl_msg, from_msg_cloud);
           pcl_rosmsg_.publish(from_msg_cloud);
           /* 座標フレーム(原点)の変換 -> target_frameを基準にする  */
           if (base_frame_name_.empty() == false) {//フレームの有無
               try {
                   listener.waitForTransform(base_frame_name_, from_msg_cloud.header.frame_id, ros::Time(0), ros::Duration(1.0));
                   pcl_ros::transformPointCloud(base_frame_name_, ros::Time(0), from_msg_cloud, from_msg_cloud.header.frame_id,  *cloud_transformed_, listener);
                   cloud_transformed_->header.frame_id = base_frame_name_; //pubするときにfame_nameが消えるから、直接入力しなｋればならない
                   transform_.publish(*cloud_transformed_);
               }
               catch (tf::TransformException ex){
                   ROS_ERROR("%s", ex.what());
                   return;
               }
           }
      }
      catch (std::exception &e){
        ROS_ERROR("%s", e.what());
        ROS_INFO("tf_transform");
      }

      //点群に対しての処理を書く
      // filtering X limit
      PointCloud::Ptr cloud_cut_x(new PointCloud);
      pcl::PassThrough<PointT> pass_x;
      pass_x.setInputCloud (cloud_transformed_);
      pass_x.setFilterFieldName ("x");
      pass_x.setFilterLimits (depth_x_min_, depth_x_max_);
      pass_x.filter (*cloud_cut_x);
      cloud_cut_x->header.frame_id = base_frame_name_;
      cut_x_pub_.publish(cloud_cut_x);

      // filtering Z limit
      PointCloud::Ptr cloud_filtered(new PointCloud());
      pcl::PassThrough<PointT> pass_z;
      pass_z.setInputCloud (cloud_cut_x);
      pass_z.setFilterFieldName ("z");
      pass_z.setFilterLimits (depth_z_min_, depth_z_max_);
      pass_z.filter (*cloud_filtered);
      cloud_filtered->header.frame_id = base_frame_name_;
      filter_pub_.publish(cloud_filtered);

      //ダウンサンプリング
      PointCloud::Ptr cloud_vg(new PointCloud());
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud (cloud_filtered);
      vg.setLeafSize (0.01, 0.01, 0.01);
      vg.setDownsampleAllData(true);
      vg.filter (*cloud_vg);
      cloud_vg->header.frame_id = base_frame_name_;
      voxel_grid_pub_.publish(cloud_vg);

      //クラスタリング
      pcl::search::KdTree<PointT>::Ptr box_tree(new pcl::search::KdTree<PointT>);
      box_tree-> setInputCloud(cloud_vg);
      std::vector<pcl::PointIndices> box_cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.1); // 10cm
      ec.setMinClusterSize (100);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (box_tree);
      ec.setInputCloud(cloud_vg);
      ec.extract (box_cluster_indices);

      //可視化
      visualization_msgs::MarkerArray marker_array;
      int target_index = -1;

      //点群に色をつける
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud <pcl::PointXYZRGB>());
      pcl::copyPointCloud(*cloud_vg, *cloud_color);

      //クラスタを可視化
      try{
          for(std::vector<pcl::PointIndices>::const_iterator it = box_cluster_indices.begin(),
                                                         it_end = box_cluster_indices.end();
                                                            it != it_end; ++it){

            pcl::getMinMax3D(*cloud_vg, *it, min_pt_,  max_pt_);
            Eigen::Vector4f cluster_size =  max_pt_ - min_pt_;
            if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
                visualization_msgs::Marker marker =  makeMarker(base_frame_name_, "box", min_pt_, max_pt_, 0.0f, 1.0f, 0.0f, 0.5f);
                  // 最も近いクラスタを検出
                  if(target_index < 0){
                    //初めに見つけたクラスタは一旦いれとく
                    target_index = marker_array.markers.size();
                    box_max_pt_ = max_pt_;
                    box_min_pt_ = min_pt_;
                    box_center_pt_ = max_pt_ - min_pt_;
                  }
                  else{
                    // d1 : target_indexのクラスタまでの距離
                    float d1 = sqrt(pow( marker_array.markers[target_index].pose.position.x , 2) + pow( marker_array.markers[target_index].pose.position.y , 2));
                    // ｄ2 : 新たに検出された特定のサイズに合致するクラスタまでの距離
                    float d2 = sqrt(pow( marker.pose.position.x , 2) + pow( marker.pose.position.y , 2));
                    if(d2 < d1){
                      target_index = marker_array.markers.size();
                      box_max_pt_ = max_pt_;
                      box_min_pt_ = min_pt_;
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
        catch (std::exception &e){
          ROS_ERROR("%s", e.what());
          ROS_INFO("visualing_cluster");
        }

        //最も近いクラスタ内の点群だけ使用
        try{
          int count = 0;
          for(std::vector<pcl::PointIndices>::const_iterator it = box_cluster_indices.begin(),
                                                         it_end = box_cluster_indices.end();
                                                            it != it_end; ++it){
            for(std::vector<int>::const_iterator pit = it-> indices.begin(); pit != it-> indices.end(); pit++){
                if(box_min_pt_.z() < cloud_vg->points[*pit].z  && cloud_vg->points[*pit].z < box_max_pt_.z()
                    && box_min_pt_.y() < cloud_vg->points[*pit].y  && cloud_vg->points[*pit].y < box_max_pt_.y()){
                      pcl::getMinMax3D(*cloud_vg, *it, entry_gate_min_pt_,  entry_gate_max_pt_);
                      entry_gate_center_pt_ = ((entry_gate_max_pt_ - entry_gate_min_pt_) / 2 ) + entry_gate_min_pt_;
                      if(target_index == count){
                        cloud_color-> points[*pit].r = 0;
                        cloud_color-> points[*pit].g = 255;
                        cloud_color-> points[*pit].b = 0;
                      }
                      else{
                        target_marker();
                        cloud_color-> points[*pit].r = 255;
                        cloud_color-> points[*pit].g = 0;
                        cloud_color-> points[*pit].b = 0;
                      }

                }//if
                else{
                      cloud_color-> points[*pit].r = 255;
                      cloud_color-> points[*pit].g = 255;
                      cloud_color-> points[*pit].b = 255;
                }//else
            }//for(pit)
                  count++;
          }//for(it)


          if(marker_array.markers.empty() == false){
            if(target_index >= 0){
              marker_array.markers[target_index].ns = "target_clusters";
              marker_array.markers[target_index].color.r = 0.0f;
              marker_array.markers[target_index].color.g = 0.0f;
              marker_array.markers[target_index].color.b = 1.0f;
              marker_array.markers[target_index].color.a = 0.5f;
              box_marker();
            }
              box_clusters_.publish(marker_array);
              entry_gate_pub_.publish(cloud_color);
          }


    }//try
    catch (std::exception &e){
      ROS_ERROR("%s", e.what());
      ROS_INFO("in the cluster");
    }
    /* メモリを解放し、引数で与えられたポインタを扱う */
    cloud_transformed_.reset(new PointCloud());
    cloud_cut_x.reset(new PointCloud());
    cloud_filtered.reset(new PointCloud());
    cloud_vg.reset(new PointCloud());
    box_tree.reset(new pcl::search::KdTree<PointT>());


 }//DetectPointCb

  /* クラスタを直方体で表示するためのMarkerを作成 */
    visualization_msgs::Marker makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
    float r, float g, float b, float a) const{

        visualization_msgs::Marker marker;

        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = marker_ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

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

        marker.lifetime = ros::Duration(0.5);
        return marker;
      }


  void target_marker(){
 		visualization_msgs::MarkerArray marker;
    std::string   points_ns[3] = {"min_pt","center_pt","max_pt"};

        marker.markers.resize(3);
     		marker.markers[0].header.frame_id = base_frame_name_;
     		marker.markers[0].header.stamp = ros::Time::now();
     		marker.markers[0].action = visualization_msgs::Marker::ADD;
     		marker.markers[0].type = visualization_msgs::Marker::SPHERE;
     		marker.markers[0].ns = points_ns[0];
     		marker.markers[0].id = 1;
     		marker.markers[0].color.a = 1.0;
     		marker.markers[0].color.r = 1.0;
     		marker.markers[0].color.g = 0.0;
     		marker.markers[0].color.b = 0.0;
     		marker.markers[0].scale.x = 0.01;//[m]
     		marker.markers[0].scale.y = 0.01;//[m]
     		marker.markers[0].scale.z = 1.0;//[m]
     		marker.markers[0].pose.position.x = entry_gate_min_pt_.x();//min_pt_[divide].x()
     		marker.markers[0].pose.position.y = entry_gate_min_pt_.y();//min_pt_.y()
     		marker.markers[0].pose.position.z = entry_gate_min_pt_.z();//min_pt_[divide].z()
     		marker.markers[0].pose.orientation.x = 0;
     		marker.markers[0].pose.orientation.y = 1;
     		marker.markers[0].pose.orientation.z = 0;
     		marker.markers[0].pose.orientation.w = 1;

        marker.markers[1].header.frame_id = base_frame_name_;
        marker.markers[1].header.stamp = ros::Time::now();
        marker.markers[1].action = visualization_msgs::Marker::ADD;
        marker.markers[1].type = visualization_msgs::Marker::SPHERE;
        marker.markers[1].ns = points_ns[1];
        marker.markers[1].id = 2;
        marker.markers[1].color.a = 1.0;
        marker.markers[1].color.r = 0.0;
        marker.markers[1].color.g = 1.0;
        marker.markers[1].color.b = 0.0;
        marker.markers[1].scale.x = 0.01;//[m]
        marker.markers[1].scale.y = 0.01;//[m]
        marker.markers[1].scale.z = 1.0;//[m]
        marker.markers[1].pose.position.x = entry_gate_center_pt_.x();//center_pt_[divide].x()
        marker.markers[1].pose.position.y = entry_gate_center_pt_.y();//center_pt_[divide].y()
        marker.markers[1].pose.position.z = entry_gate_center_pt_.z();//center_pt_[divide].z()
        marker.markers[1].pose.orientation.x = 0;
        marker.markers[1].pose.orientation.y = 1;
        marker.markers[1].pose.orientation.z = 0;
        marker.markers[1].pose.orientation.w = 1;

        marker.markers[2].header.frame_id = base_frame_name_;
        marker.markers[2].header.stamp = ros::Time::now();
        marker.markers[2].action = visualization_msgs::Marker::ADD;
        marker.markers[2].type = visualization_msgs::Marker::SPHERE;
        marker.markers[2].ns = points_ns[2];
        marker.markers[2].id = 3;
        marker.markers[2].color.a = 1.0;
        marker.markers[2].color.r = 0.0;
        marker.markers[2].color.g = 0.0;
        marker.markers[2].color.b = 1.0;
        marker.markers[2].scale.x = 0.01;//[m]
        marker.markers[2].scale.y = 0.01;//[m]
        marker.markers[2].scale.z = 1.0;//[m]
        marker.markers[2].pose.position.x = entry_gate_max_pt_.x();//max_pt_[divide].x()
        marker.markers[2].pose.position.y = entry_gate_max_pt_.y();//max_pt_[divide].y()
        marker.markers[2].pose.position.z = entry_gate_max_pt_.z();//max_pt_[divide].z()
        marker.markers[2].pose.orientation.x = 0;
        marker.markers[2].pose.orientation.y = 1;
        marker.markers[2].pose.orientation.z = 0;
        marker.markers[2].pose.orientation.w = 1;

    target_marker_.publish(marker);

 	}//target_marker


  void box_marker(){
 		visualization_msgs::MarkerArray marker;
    std::string   points_ns[2] = {"box_min_pt","box_max_pt"};

        marker.markers.resize(2);
     		marker.markers[0].header.frame_id = base_frame_name_;
     		marker.markers[0].header.stamp = ros::Time::now();
     		marker.markers[0].action = visualization_msgs::Marker::ADD;
     		marker.markers[0].type = visualization_msgs::Marker::SPHERE;
     		marker.markers[0].ns = points_ns[0];
     		marker.markers[0].id = 4;
     		marker.markers[0].color.a = 1.0;
     		marker.markers[0].color.r = 1.0;
     		marker.markers[0].color.g = 1.0;
     		marker.markers[0].color.b = 0.0;
     		marker.markers[0].scale.x = 0.01;//[m]
     		marker.markers[0].scale.y = 0.01;//[m]
     		marker.markers[0].scale.z = 1.0;//[m]
     		marker.markers[0].pose.position.x = box_min_pt_.x();//min_pt_[divide].x()
     		marker.markers[0].pose.position.y = box_min_pt_.y();//min_pt_.y()
     		marker.markers[0].pose.position.z = box_min_pt_.z();//min_pt_[divide].z()
     		marker.markers[0].pose.orientation.x = 0;
     		marker.markers[0].pose.orientation.y = 1;
     		marker.markers[0].pose.orientation.z = 0;
     		marker.markers[0].pose.orientation.w = 1;


        marker.markers[1].header.frame_id = base_frame_name_;
        marker.markers[1].header.stamp = ros::Time::now();
        marker.markers[1].action = visualization_msgs::Marker::ADD;
        marker.markers[1].type = visualization_msgs::Marker::SPHERE;
        marker.markers[1].ns = points_ns[1];
        marker.markers[1].id = 5;
        marker.markers[1].color.a = 1.0;
        marker.markers[1].color.r = 1.0;
        marker.markers[1].color.g = 0.0;
        marker.markers[1].color.b = 1.0;
        marker.markers[1].scale.x = 0.01;//[m]
        marker.markers[1].scale.y = 0.01;//[m]
        marker.markers[1].scale.z = 1.0;//[m]
        marker.markers[1].pose.position.x = box_max_pt_.x();//max_pt_[divide].x()
        marker.markers[1].pose.position.y = box_max_pt_.y();//max_pt_[divide].y()
        marker.markers[1].pose.position.z = box_max_pt_.z();//max_pt_[divide].z()
        marker.markers[1].pose.orientation.x = 0;
        marker.markers[1].pose.orientation.y = 1;
        marker.markers[1].pose.orientation.z = 0;
        marker.markers[1].pose.orientation.w = 1;

    box_marker_.publish(marker);

 	}//box_marker

  bool send_tf_frame(){
    if(box_center_pt_.x() != 0 && entry_gate_center_pt_.y() != 0 && entry_gate_center_pt_.z() != 0 ){
      entry_gate.setOrigin( tf::Vector3(box_min_pt_.x(), entry_gate_center_pt_.y(), entry_gate_center_pt_.z()) );
      entry_gate.setRotation( tf::Quaternion(0, 0, 0) );
      br.sendTransform(tf::StampedTransform(entry_gate, ros::Time(0), base_frame_name_, "entry_gate_point" ));//TFの送信
    }//if
    else{
      std::cout << "no entry_gate" << std::endl;
    }
    return true;
  }//send_tf_frame


  // 検出しなかったとき
  void no_detect(std::string msg){
   ROS_ERROR("No detect target");
   std::cout << "Reason:\t" << msg << std::endl;
  }//no_detect
};//class BoxDetection

int main(int argc, char *argv[]){
  /* ノードの初期化 */
  ros::init(argc, argv, "box_detection_node");
  /* BoxDetection のインスタンスを作成 */
  BoxDetection box_detection_node;
  while (ros::ok()){
    box_detection_node.send_tf_frame();
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

}
