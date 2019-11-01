#include <ros/ros.h>
#include <iostream>
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
    ros::Publisher                            cut_x_pub_;
    ros::Publisher                            box_clusters_;
    ros::Publisher                            entry_gate_clusters_;
    ros::Publisher                            coordinate_point_;


    tf::TransformListener                     listener;


    PointCloud::Ptr                           pcl_cloud_;
    PointCloud::Ptr                           cloud_transformed_;
    PointCloud::Ptr                           cluster_max_,cluster_min_;
    PointCloud::Ptr                           entry_gate_cloud_;
    PointCloud::Ptr                           boxDetect_cloud_;

    std::string                               base_frame_name_;
    std::string                               points_topic_name_;
    /*  Eigen::Vector */
    Eigen::Vector4f                           min_pt_, max_pt_, center_pt_;
    Eigen::Vector4f                           nearest_min_pt_, nearest_max_pt_, nearest_cluster_;

    int                                       edge = 0;
    float                                     cluster_width_, cluster_hight_;
    float                                     max_cloud_x_, max_cloud_z_, min_cloud_x_, min_cloud_z_;
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

     base_frame_name_ = "base_footprint";
     points_topic_name_ = "/camera/depth/points" ;

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
     box_clusters_        =  nh_.advertise<visualization_msgs::MarkerArray>("box_cluster", 1);
     coordinate_point_    =  nh_.advertise<visualization_msgs::MarkerArray>("box_point", 1);
     entry_gate_clusters_ =  nh_.advertise<visualization_msgs::MarkerArray>("entry_gate_cluster", 1);
     //ROS_INFO("1");
    }


    void DetectPointCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){
      PointCloud::Ptr cloud_transformed_(new PointCloud());//初期化
      try {
         /* sensor_msgs/PointCloud2データ ->　pcl/PointCloudに変換 */
         PointCloud from_msg_cloud;
         pcl::fromROSMsg(*pcl_msg, from_msg_cloud);
         pcl_rosmsg_.publish(from_msg_cloud);
         /* 座標フレーム(原点)の変換　-> target_frameを基準にする  */
         if (base_frame_name_.empty() == false) {//フレームの有無
             try {
                 listener.waitForTransform(base_frame_name_, from_msg_cloud.header.frame_id, ros::Time(0), ros::Duration(1.0));
                 //std::cout << from_msg_cloud.header.frame_id << std::endl;
                 //cloud_transformed_.reset(new PointCloud());//初期化
                 pcl_ros::transformPointCloud(base_frame_name_, ros::Time(0), from_msg_cloud, from_msg_cloud.header.frame_id,  *cloud_transformed_, listener);
                 //std::cout << "points:" << cloud_transformed_->points.size() << std::endl;
                 cloud_transformed_->header.frame_id = base_frame_name_; //pubするときにfame_nameが消えるから、直接入力しなｋればならない
                 transform_.publish(*cloud_transformed_);

             }
             catch (tf::TransformException ex){
                 ROS_ERROR("%s", ex.what());
                 return;
             }
         }
       //ここに cloud_transformed__ に対するフィルタ処理を書く
       //  ROS_INFO("width: %u, height: %u", cloud_transformed_->width, cloud_transformed_->height);
      }
      catch (std::exception &e){
        ROS_ERROR("%s", e.what());
      }

      //点群に対しての処理を書く
      // filtering X limit(念の為)
      //std::cout << "cloud_transformed_:" << cloud_transformed_->points.size() << std::endl;
      PointCloud::Ptr cloud_cut_x(new PointCloud);
      pcl::PassThrough<PointT> pass_x;
      pass_x.setInputCloud (cloud_transformed_);
      pass_x.setFilterFieldName ("x");
      pass_x.setFilterLimits (0.1, 1.5);
      pass_x.filter (*cloud_cut_x);
      cloud_cut_x->header.frame_id = base_frame_name_;
      //std::cout << "cloud_cut_x : " << cloud_cut_x->points.size() << std::endl;
      cut_x_pub_.publish(cloud_cut_x);

      // filtering Z limit（念の為）
      PointCloud::Ptr cloud_filtered(new PointCloud());
      pcl::PassThrough<PointT> pass_z;
      pass_z.setInputCloud (cloud_cut_x);
      pass_z.setFilterFieldName ("z");
      pass_z.setFilterLimits (0.01, 1.0);
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
      //std::cout << "voxel_grid : " << cloud_vg->points.size() << std::endl;
      voxel_grid_pub_.publish(cloud_vg);

      //ROS_INFO("4-3");
      //クラスタリング
      pcl::search::KdTree<PointT>::Ptr box_tree(new pcl::search::KdTree<PointT>);
      box_tree-> setInputCloud(cloud_vg);
      std::vector<pcl::PointIndices> box_cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (1000);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (box_tree);
      ec.setInputCloud(cloud_vg);
      ec.extract (box_cluster_indices);
      visualization_msgs::MarkerArray marker_array;
      int marker_id[2];
      int target_index = -1;
      //size_t ok = 0;

      //ROS_INFO("5");
      float threshold_value = 0.02;
      std::cout << " target_index_origin =  " << target_index << std::endl;

      for(std::vector<pcl::PointIndices>::const_iterator it     = box_cluster_indices.begin(),
                                                         it_end = box_cluster_indices.end();
                                                         it != it_end;
                                                         ++it, ++marker_id[0]){
          pcl::getMinMax3D(*cloud_vg, *it, min_pt_,  max_pt_);
          Eigen::Vector4f cluster_size =  max_pt_ -  min_pt_;
          /* 検出したクラスタの重心を計算 */
          pcl::compute3DCentroid(*cloud_vg, center_pt_);//重心を計算
          //std::cout << marker_id[0] << std::endl;

          if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
            /*
            marker_array.markers.push_back(
              makeMarker(base_frame_name_, "box", marker_id, min_pt_, max_pt_, 0.0f, 1.0f, 0.0f, 0.5f));
            //std::cout << "marker_array:" << marker_array << std::endl;
            */
            visualization_msgs::Marker marker =
            makeMarker(
                frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f);

              /* 最も近いクラスタを検出 */
              if(target_index < 0){
                target_index = marker_array.markers.size();
                /*
                nearest_min_pt_ = min_pt_;
                nearest_max_pt_ = max_pt_;
                nearest_cluster_ = center_pt_;
                */
              }
              else{
                /* d1 : target_indexのクラスタまでの距離 */
                float d1 = sqrt(pow(marker_array.markers[target_index].pose.position.x, 2) + pow(marker_array.markers[target_index].pose.position.y, 2));
                /* ｄ2 : 新たに検出された特定のサイズに合致するクラスタまでの距離 */
                float d2 = sqrt(pow(center_pt_.x(), 2) + pow(center_pt_.y(), 2));
                if(d2 < d1){
                  std::cout << " d1::  " << d1 << std::endl;
                  std::cout << " d2::  " << d2 << std::endl;
                  /*
                  nearest_min_pt_ = min_pt_;
                  nearest_max_pt_ = max_pt_;
                  nearest_cluster_ = center_pt_;
                  target_index = false;
                  */
                  target_index = marker_array.markers.size();
                }
              }//else
              //for(std::vector<int>::const_iterator pit = it-> indices.begin (); pit != it-> indices.end (); pit++){}//for

          }//if
          else{
            no_detect("No detect the target : xtion");
            return;
          }//else
      }//for
      /*クラスタの有無*/
      if (marker_array.markers.empty() == false){

          box_clusters_.publish(marker_array);
      }
//ROS_INFO("6");
    /*
    //std::cout << entry_gate_cloud_ << std::endl;
    //投入口のクラスタリング
    pcl::search::KdTree<PointT>::Ptr entry_gate_tree(new pcl::search::KdTree<PointT>);
    entry_gate_tree-> setInputCloud( entry_gate_cloud_);
    //ROS_INFO("6-1");
    std::vector<pcl::PointIndices>  entry_gate_cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec2;
    ec2.setClusterTolerance (0.02); // 2cm
    ec2.setMinClusterSize (1000);
    ec2.setMaxClusterSize (25000);
    ec2.setSearchMethod (entry_gate_tree);
    //ROS_INFO("6-2");
    ec2.setInputCloud( entry_gate_cloud_);
    //ROS_INFO("6-3");
    ec2.extract (entry_gate_cluster_indices);
    //ROS_INFO("6-4");
    visualization_msgs::MarkerArray entry_gate_marker_array;
    int entry_gate_marker_id = 0;
    //ROS_INFO("6-5");
    for(std::vector<pcl::PointIndices>::const_iterator pit     = entry_gate_cluster_indices.begin(),
                                                       pit_end = entry_gate_cluster_indices.end();
                                                       pit != pit_end;
                                                       ++pit){
      pcl::getMinMax3D(* entry_gate_cloud_, *pit,  entry_gate_min_pt_,  entry_gate_max_pt_);
      Eigen::Vector4f entry_gate_cluster_size =  entry_gate_max_pt_ -  entry_gate_min_pt_;
      if(entry_gate_cluster_size.x() > 0 && entry_gate_cluster_size.y() > 0 && entry_gate_cluster_size.z() > 0){
        /* Markerをmarker_array.markersに追加
         entry_gate_marker_array.markers.push_back(
         entry_gate_makeMarker(
             base_frame_name_, "entry_gate", entry_gate_marker_id, entry_gate_min_pt_, entry_gate_max_pt_, 0.0f, 1.0f, 0.0f, 0.5f));

      }
      /* 検出したクラスタの重心を計算
      Eigen::Vector4f xyz_centroid; //クラスタの重心
      pcl::compute3DCentroid(* entry_gate_cloud_, xyz_centroid);//重心を計算
    }
    */

    //ROS_INFO("7");
  }//DetectPointCb

  /* クラスタを直方体で表示するためのMarkerを作成 */
    visualization_msgs::Marker makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int marker_id[2],
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
    float r, float g, float b, float a) const{
        visualization_msgs::Marker marker;
        int divide;
        if(marker_ns == "box"){divide = 0;}
        else if(marker_ns == "entry_gate"){divide = 1;}
        else{ROS_INFO("No cluster");return marker;}
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = marker_ns[divide];
        marker.id = marker_id[divide];
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

        marker.lifetime = ros::Duration(0.3);

        return marker;
      }
    /*クラスタを直方体で表示するためのMarkerを作成（投入口用）
    visualization_msgs::Marker entry_gate_makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int marker_id[0],
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
    float r, float g, float b, float a) const{
        visualization_msgs::Marker entry_gate_marker;
        entry_gate_marker.header.frame_id = frame_id;
        entry_gate_marker.header.stamp = ros::Time::now();
        entry_gate_marker.ns = marker_ns;
        entry_gate_marker.id = marker_id[0];
        entry_gate_marker.type = visualization_msgs::Marker::CUBE;
        entry_gate_marker.action = visualization_msgs::Marker::ADD;

        entry_gate_marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
        entry_gate_marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
        entry_gate_marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

        entry_gate_marker.pose.orientation.x = 0.0;
        entry_gate_marker.pose.orientation.y = 0.0;
        entry_gate_marker.pose.orientation.z = 0.0;
        entry_gate_marker.pose.orientation.w = 1.0;

        entry_gate_marker.scale.x = max_pt.x() - min_pt.x();
        entry_gate_marker.scale.y = max_pt.y() - min_pt.y();
        entry_gate_marker.scale.z = max_pt.z() - min_pt.z();

        entry_gate_marker.color.r = r;
        entry_gate_marker.color.g = g;
        entry_gate_marker.color.b = b;
        entry_gate_marker.color.a = a;

        entry_gate_marker.lifetime = ros::Duration(0.3);
        return entry_gate_marker;
    }
    */


  void target_marker(const std::string &marker_ns){
 		visualization_msgs::MarkerArray marker;
    std::string   points_ns[3] = {"min_pt","center_pt","max_pt"};
    int           divide;//boxかentry_gateの判別
    if(marker_ns == "box"){divide = 0;}
    else if(marker_ns == "entry_gate"){divide = 1;}

        marker.markers.resize(3);
     		marker.markers[0].header.frame_id = base_frame_name_;
     		marker.markers[0].header.stamp = ros::Time::now();
     		marker.markers[0].action = visualization_msgs::Marker::ADD;
     		marker.markers[0].type = visualization_msgs::Marker::SPHERE;
     		marker.markers[0].ns = points_ns[0];
     		marker.markers[0].id = 1;
     		marker.markers[0].color.a = 1.0;
     		marker.markers[0].color.r = 0.0;
     		marker.markers[0].color.g = 0.0;
     		marker.markers[0].color.b = 1.0;
     		marker.markers[0].scale.x = 0.01;//[m]
     		marker.markers[0].scale.y = 0.01;//[m]
     		marker.markers[0].scale.z = 1.0;//[m]
     		marker.markers[0].pose.position.x = min_pt_.x();//min_pt_[divide].x()
     		marker.markers[0].pose.position.y = min_pt_.y();//min_pt_.y()
     		marker.markers[0].pose.position.z = min_pt_.z();//min_pt_[divide].z()
     		marker.markers[0].pose.orientation.x = 0;
     		marker.markers[0].pose.orientation.y = 0;
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
        marker.markers[1].pose.position.x = center_pt_.x();//center_pt_[divide].x()
        marker.markers[1].pose.position.y = center_pt_.y();//center_pt_[divide].y()
        marker.markers[1].pose.position.z = center_pt_.z();//center_pt_[divide].z()
        marker.markers[1].pose.orientation.x = 0;
        marker.markers[1].pose.orientation.y = 0;
        marker.markers[1].pose.orientation.z = 0;
        marker.markers[1].pose.orientation.w = 1;

        marker.markers[2].header.frame_id = base_frame_name_;
        marker.markers[2].header.stamp = ros::Time::now();
        marker.markers[2].action = visualization_msgs::Marker::ADD;
        marker.markers[2].type = visualization_msgs::Marker::SPHERE;
        marker.markers[2].ns = points_ns[2];
        marker.markers[2].id = 3;
        marker.markers[2].color.a = 1.0;
        marker.markers[2].color.r = 1.0;
        marker.markers[2].color.g = 0.0;
        marker.markers[2].color.b = 0.0;
        marker.markers[2].scale.x = 0.01;//[m]
        marker.markers[2].scale.y = 0.01;//[m]
        marker.markers[2].scale.z = 1.0;//[m]
        marker.markers[2].pose.position.x = max_pt_.x();//max_pt_[divide].x()
        marker.markers[2].pose.position.y = max_pt_.y();//max_pt_[divide].y()
        marker.markers[2].pose.position.z = max_pt_.z();//max_pt_[divide].z()
        marker.markers[2].pose.orientation.x = 0;
        marker.markers[2].pose.orientation.y = 0;
        marker.markers[2].pose.orientation.z = 0;
        marker.markers[2].pose.orientation.w = 1;

    coordinate_point_.publish(marker);

 	}//target_marker




  /* 検出しなかったとき */
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
  ROS_INFO("Hello Point Cloud!");
  while (ros::ok()){
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

}
