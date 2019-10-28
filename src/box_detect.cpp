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


    tf::TransformListener                     listener;


    PointCloud::Ptr                           pcl_cloud_;
    PointCloud::Ptr                           cloud_transformed_;
    PointCloud::Ptr                           cluster_max_,cluster_min_;
    PointCloud::Ptr                           entry_gate_cloud_;
    PointCloud::Ptr                           boxDetect_cloud_;

    std::string                               base_frame_name_;
    std::string                               points_topic_name_;
    /*  Eigen::Vector */
    Eigen::Vector4f                           xyz_centroid_min_;//中心座標
    Eigen::Vector4f                           min_pt_, max_pt_;
    Eigen::Vector4f                           entry_gate_min_pt_, entry_gate_max_pt_;

    float                                     cluster_width_,cluster_hight_;
    float                                     max_cloud_x_,max_cloud_z_,min_cloud_x_,min_cloud_z_;
    float                                     depth_x_max,depth_x_min,depth_z_max,depth_z_min;

  public:
    BoxDetection()
      :nh_()
      ,pnh_("~")
      ,boxDetect_cloud_(new PointCloud()){
     // load rosparam
     ros::param::get("depth_range_min_x", depth_x_min);
     ros::param::get("depth_range_max_x", depth_x_max);
     ros::param::get("depth_range_min_z", depth_z_min);
     ros::param::get("depth_range_max_z", depth_z_max);

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
     box_pub_             = nh_.advertise<sensor_msgs::PointCloud2>("/box_detection",1);
     pcl_rosmsg_          =  nh_.advertise<sensor_msgs::PointCloud2>("/pcl_rosMsg",1);
     transform_           = nh_.advertise<sensor_msgs::PointCloud2>("/transform",1);
     cut_x_pub_           =  nh_.advertise<sensor_msgs::PointCloud2>("/cut_x_cloud",1);
     filter_pub_          =  nh_.advertise<sensor_msgs::PointCloud2>("/filter_cloud",1);
     voxel_grid_pub_      =  nh_.advertise<sensor_msgs::PointCloud2>("/voxel_grid",1);
     box_clusters_        = nh_.advertise<visualization_msgs::MarkerArray>("box_cluster", 1);
     entry_gate_clusters_ = nh_.advertise<visualization_msgs::MarkerArray>("entry_gate_cluster", 1);
     //ROS_INFO("1");
    }


    PointCloud::ConstPtr GetBoxDtectionCloud() const{
        return BoxDetection::boxDetect_cloud_;
        //ROS_INFO("2");
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
      std::cout << "cloud_transformed_:" << cloud_transformed_->points.size() << std::endl;
      PointCloud::Ptr cloud_cut_x(new PointCloud);
      pcl::PassThrough<PointT> pass_x;
      pass_x.setInputCloud (cloud_transformed_);
      pass_x.setFilterFieldName ("x");
      pass_x.setFilterLimits (0.1, 3.0);
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
      std::cout << "voxel_grid : " << cloud_vg->points.size() << std::endl;
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
      visualization_msgs::MarkerArray box_marker_array;
      int box_marker_id = 0;

      //ROS_INFO("5");
      int j=0;
      float threshold_value = 0.02;

      for(std::vector<pcl::PointIndices>::const_iterator it     = box_cluster_indices.begin(),
                                                         it_end = box_cluster_indices.end();
                                                         it != it_end;
                                                         ++it, ++box_marker_id){
          pcl::getMinMax3D(*cloud_vg, *it, min_pt_,  max_pt_);
          Eigen::Vector4f cluster_size =  max_pt_ -  min_pt_;

          std::cout << box_marker_id << std::endl;
          if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
            box_marker_array.markers.push_back(
            box_makeMarker(base_frame_name_, "box", box_marker_id, min_pt_, max_pt_, 0.0f, 1.0f, 0.0f, 0.5f));
            //std::cout << "box_marker_array:" << box_marker_array << std::endl;
            for(std::vector<int>::const_iterator pit = it-> indices.begin (); pit != it-> indices.end (); pit++){

              /*クラスタの一番近いクラスタを用いた縁検出*/
              if(box_marker_id == 0){
                if(cloud_vg-> points[*pit].x - cloud_vg-> points[*pit].x > threshold_value){
                  std::cout << "find edge points!" << std::endl;
                  cloud_vg-> points[*pit] =  entry_gate_cloud_->points[j];
                  j++;
                }
              }

            }
          }//if
          else{
            no_detect("No detect the target : xtion");
            return;
          }//else
      }//for
      /*クラスタの有無*/
      if (box_marker_array.markers.empty() == false){
          box_clusters_.publish(box_marker_array);
      }
      ROS_INFO("6");
    std::cout << entry_gate_cloud_ << std::endl;
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
        /* Markerをbox_marker_array.markersに追加 */
         entry_gate_marker_array.markers.push_back(
         entry_gate_makeMarker(
             base_frame_name_, "entry_gate_cluster", entry_gate_marker_id, entry_gate_min_pt_, entry_gate_max_pt_, 0.0f, 1.0f, 0.0f, 0.5f));

      }
      /* 検出したクラスタの重心を計算 */
      Eigen::Vector4f xyz_centroid; //クラスタの重心
      pcl::compute3DCentroid(* entry_gate_cloud_, xyz_centroid);//重心を計算
    }

    ROS_INFO("7");
  }//DetectPointCb

  /* クラスタを直方体で表示するためのMarkerを作成（箱用） */
    visualization_msgs::Marker box_makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int marker_id,
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
    float r, float g, float b, float a) const
    {
        visualization_msgs::Marker box_marker;
        box_marker.header.frame_id = frame_id;
        box_marker.header.stamp = ros::Time::now();
        box_marker.ns = marker_ns;
        box_marker.id = marker_id;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.action = visualization_msgs::Marker::ADD;

        box_marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
        box_marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
        box_marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

        box_marker.pose.orientation.x = 0.0;
        box_marker.pose.orientation.y = 0.0;
        box_marker.pose.orientation.z = 0.0;
        box_marker.pose.orientation.w = 1.0;

        box_marker.scale.x = max_pt.x() - min_pt.x();
        box_marker.scale.y = max_pt.y() - min_pt.y();
        box_marker.scale.z = max_pt.z() - min_pt.z();

        box_marker.color.r = r;
        box_marker.color.g = g;
        box_marker.color.b = b;
        box_marker.color.a = a;

        box_marker.lifetime = ros::Duration(0.3);
        return box_marker;
    }
    /* クラスタを直方体で表示するためのMarkerを作成（投入口用） */
    visualization_msgs::Marker entry_gate_makeMarker(
    const std::string &frame_id, const std::string &marker_ns,
    int box_marker_id,
    const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
    float r, float g, float b, float a) const
    {
        visualization_msgs::Marker entry_gate_marker;
        entry_gate_marker.header.frame_id = frame_id;
        entry_gate_marker.header.stamp = ros::Time::now();
        entry_gate_marker.ns = marker_ns;
        entry_gate_marker.id = box_marker_id;
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
  ROS_INFO("1-1");
  while (ros::ok()){
    ROS_INFO("1-2");
    ros::Duration(0.1).sleep();
    ROS_INFO("1-3");
    ros::spinOnce();
    ROS_INFO("1-4");
  }
  /*
  while (ros::ok()) {
    ros::spin();
    if (!viewer.wasStopped()) {
      viewer.showCloud(box_detection_node.GetBoxDtectionCloud());
    }
    spin_rate.sleep();
  }
  */
}
