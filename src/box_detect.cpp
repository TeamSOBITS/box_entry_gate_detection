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




typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class BoxDetection{

  private:

    /* Node Handle */
    ros::NodeHandle                           nh_;                              //pub/sub
    ros::NodeHandle                           pnh_;                             //param

    ros::Subscriber                           cloud_sub_;
    ros::Publisher                            plane_pub_;

    tf::TransformListener                     listener;


    PointCloud::Ptr                           pcl_cloud_;
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
     ros::param::get("depth_range_min_x", this->depth_x_min);
     ros::param::get("depth_range_max_x", this->depth_x_max);
     ros::param::get("depth_range_min_z", this->depth_z_min);
     ros::param::get("depth_range_max_z", this->depth_z_max);

     ros::param::get("base_frame_name", this->base_frame_name_);

     base_frame_name_ = "base_footprint";
     points_topic_name_ = "/camera/depth/points" ;

     ROS_INFO("0");
     cloud_sub_ = nh_.subscribe(points_topic_name_, 1, &BoxDetection::DetectPointCb, this);
     std::cout << base_frame_name_ << std::endl;

     //エラーの時
     if(!ros::topic::waitForMessage<sensor_msgs::PointCloud2>(points_topic_name_, ros::Duration(3.0))){
         ROS_ERROR("timeout");
         exit(EXIT_FAILURE);
       }
     plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/box_detection",1);//publishするときのtarget_frame
     ROS_INFO("1");
    }


    PointCloud::ConstPtr GetBoxDtectionCloud() const{
        return BoxDetection::boxDetect_cloud_;
        ROS_INFO("2");
    }


    void DetectPointCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){

      PointCloud::Ptr      from_msg_cloud(new PointCloud);
      pcl::fromROSMsg(*pcl_msg, *from_msg_cloud);//msg型　→　pointcloud型へ変換
      if (from_msg_cloud-> points.size() == 0) {
          ROS_ERROR("No from_msg_cloud");
          return;
      }
      std::string    camera_frame_name = pcl_msg-> header.frame_id;


      ROS_INFO("3-1");
      //ここで座標変換可能か確認
      bool key = listener.canTransform(base_frame_name_, camera_frame_name, ros::Time(0));
      if (key == false) {
        ROS_ERROR("box_detection_node : pcl canTransform fail");
        return;
      }
      ROS_INFO("3-2");
      // base_flame基準のpointcloudに変換
      PointCloud::Ptr cloud_transformed(new PointCloud);
      pcl_ros::transformPointCloud(base_frame_name_, ros::Time(0), *from_msg_cloud, camera_frame_name, *cloud_transformed, listener);
      ROS_INFO("3-3");

      //点群に対しての処理を書く
      // filtering X limit(念の為)
      PointCloud::Ptr cloud_cut_x(new PointCloud);
      pcl::PassThrough<pcl::PointXYZ> pass_x;
      pass_x.setInputCloud (cloud_transformed);
      pass_x.setFilterFieldName ("x");
      pass_x.setFilterLimits (this-> depth_x_min, this-> depth_x_max);
      pass_x.filter (*cloud_cut_x);

      // filtering Z limit（念の為）
      PointCloud::Ptr cloud_filtered(new PointCloud);
      pcl::PassThrough<pcl::PointXYZ> pass_z;
      pass_z.setInputCloud (cloud_cut_x);
      pass_z.setFilterFieldName ("z");
      pass_z.setFilterLimits (this-> depth_z_min, this-> depth_z_max);
      pass_z.filter (*cloud_filtered);

      //ダウンサンプリング
      PointCloud::Ptr cloud_vg(new PointCloud);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud (cloud_filtered);
      vg.setLeafSize (0.001f, 0.001f, 0.001f);
      vg.setDownsampleAllData(true);
      vg.filter (*cloud_vg);

      //クラスタリング
      pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree-> setInputCloud(cloud_vg);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance (0.02); // 2cm
      ec.setMinClusterSize (1000);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud(cloud_vg);
      ec.extract (cluster_indices);

      ROS_INFO("5");
      int j=0;
      float threshold_value = 0.02;

      for(std::vector<pcl::PointIndices>::const_iterator it     = cluster_indices.begin(),
                                                         it_end = cluster_indices.end();
                                                         it != it_end;
                                                         ++it){
        pcl::getMinMax3D(*cloud_vg, *it, this-> min_pt_, this-> max_pt_);
        Eigen::Vector4f cluster_size = this-> max_pt_ - this-> min_pt_;
        if(cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0){
          for(std::vector<int>::const_iterator pit = it-> indices.begin (); pit != it-> indices.end (); pit++){
            if(cloud_vg-> points[*pit].x - cloud_vg-> points[*pit].x > threshold_value){
              cloud_vg-> points[*pit] = this-> entry_gate_cloud_->points[j];
              j++;
            }//if
          }//for
        }//if
        else{
          no_detect("No detect the target : xtion");
          return;
        }//else
      }//for
      ROS_INFO("6");

    //投入口のクラスタリング
    tree-> setInputCloud(this-> entry_gate_cloud_);
    std::vector<pcl::PointIndices>  entry_gate_cluster_indices;
    ec.setSearchMethod (tree);
    ec.setInputCloud(this-> entry_gate_cloud_);
    ec.extract (entry_gate_cluster_indices);
    for(std::vector<pcl::PointIndices>::const_iterator pit     = entry_gate_cluster_indices.begin(),
                                                       pit_end = entry_gate_cluster_indices.end();
                                                       pit != pit_end;
                                                       ++pit){
      pcl::getMinMax3D(*this-> entry_gate_cloud_, *pit, this-> entry_gate_min_pt_, this-> entry_gate_max_pt_);
      /* 検出したクラスタの重心を計算 */
      Eigen::Vector4f xyz_centroid; //クラスタの重心
      pcl::compute3DCentroid(*this-> entry_gate_cloud_, xyz_centroid);//重心を計算
    }
    ROS_INFO("7");
  }//DetectPointCb

  /* 検出しなかったとき */
 void no_detect(std::string msg){
   ROS_ERROR("No detect target");
   ROS_INFO("8");
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
