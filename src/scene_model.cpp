/* PCDファイルからのポイントクラウドデータの読み取り、sensor_msgs::PointCloud2型変数に変換してパブリッシュする */

/* ROSの基本ヘッダ */
#include <ros/ros.h>
/* 入出力関連ヘッダ */
#include <iostream>
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>            //pcl::PointCloud<T>をROSメッセージとしてPublishおよびSubscribeできる
#include <pcl/point_types.h>                //PCLで実装されたすべてのPointTポイントタイプ構造体を定義する
#include <pcl_ros/transforms.h>                   //ポイントクライドを任意の座標フレームに変換する
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
/* sensor_msgs */
#include <sensor_msgs/PointCloud2.h>
#include "unistd.h"



typedef pcl::PointXYZ PointT; //pcl::PointXYZ → PointT　点群データ構造体
typedef pcl::PointCloud<PointT> PointCloud; //スマートポインタ　pcl::PointCloud<pcl::PointXYZ> → PointCloud　

class PointcloudPublisherNode
{
  private:
    ros::NodeHandle nh_;                                 //pub/sub
    ros::NodeHandle pnh_;                                //parameter
    /* Publisher */
    ros::Publisher pub_cloud_sensor_;                          //点群のパブリッシャー
    ros::Publisher pub_cloud_;                          //点群のパブリッシャー
    /* param */
    std::string data_path_;           //pcdファイルのパス


  /* PointCloudの回転(必要になったら) */
  void rotation(){
    PointCloud::Ptr cloud (new PointCloud());     //点群データを格納するためのPointCloud型変数（ポインター）
    PointCloud::Ptr cloud_transformed (new PointCloud());     //点群データを格納するためのPointCloud型変数（ポインター）

    /* x軸を中心にtheta回転させるコード */
    Eigen::Matrix4f rotation_matrix_x;
    float cos_theta = 0.0;
    float sin_theta = 1.0;
    //行列を作成する 4x4
    rotation_matrix_x << \
      1,         0,           0, 0, \
      0, cos_theta, - sin_theta, 0, \
      0, sin_theta,   cos_theta, 0, \
      0,          0,           0, 1;

    //回転
    //pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_x );
    //cloud = cloud_transformed;


    /* y軸を中心にtheta回転させるコード */
    Eigen::Matrix4f rotation_matrix_y;
    cos_theta = 0.0;
    sin_theta = -1.0;
    //行列を作成する 4x4
      rotation_matrix_y << \
        cos_theta,  0,    sin_theta, 0, \
                0,  1,            0, 0, \
      -sin_theta,  0,    cos_theta, 0, \
                0,  0,            0, 1;
    //回転
    pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_y );
    cloud = cloud_transformed;

    /* z軸を中心にtheta回転させるコード */
    Eigen::Matrix4f rotation_matrix_z;
    cos_theta = 0.0;
    sin_theta = -1.0;
    //行列を作成する 4x4
    rotation_matrix_z << \
      cos_theta,  - sin_theta,   0, 0, \
      sin_theta,    cos_theta,   0, 0, \
              0,            0,   1, 0, \
              0,            0,   0, 1;

    //回転
    //pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_matrix_z );
    //cloud = cloud_transformed;
    return;
  }


  /* メイン関数 */
  int main(void){
    PointCloud::Ptr cloud (new PointCloud());     //点群データを格納するためのPointCloud型変数（ポインター）
    PointCloud::Ptr cloud_transformed (new PointCloud());     //点群データを格納するためのPointCloud型変数（ポインター）

    /* 作成したPointCloudを読み込む */
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (this->data_path_, *cloud) == -1){
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    /* 読み込まれたポイントクラウドデータの中身を出力 */
    std::cout << "===========================================================\n" << std::endl;
    std::cout << "Loaded\n"
              << cloud->width * cloud->height
              << "data points from test_pcd.pcd with the following fields:\n"
              << std::endl;
    /*
    std::cout << "\tx : [m]\t\ty : [m]\t\tz : [m]" << std::endl;
    std::cout << "-----------------------------------------------------------" << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i){
      std::cout << i+1  << "\tx : "   << cloud->points[i].x
                        << "\ty : "   << cloud->points[i].y
                        << "\tz : "   << cloud->points[i].z << std::endl;
    }
    */
    std::cout << "===========================================================\n" << std::endl;

    /* sensor_msgs */
    sensor_msgs::PointCloud2 sensor_cloud;  //点群データを格納するためのsensor_msgs::PointCloud2型変数

    /* PointCloud型変数をsensor_msgs::PointCloud2型変数に変換 */
    pcl::toROSMsg(*cloud, sensor_cloud);

    cloud->header.frame_id = "camera_rgb_optical_frame";
    sensor_cloud.header.frame_id = "camera_rgb_optical_frame";

    /* sensor_cloudをパブリッシュ */
    ros::Rate loop_rate(3);
    while (ros::ok()){
      this->pub_cloud_.publish(cloud);
      this->pub_cloud_sensor_.publish(sensor_cloud);
      loop_rate.sleep();
    }
    return 0;
  }


  public:
    PointcloudPublisherNode()
      : nh_()
      , pnh_("~"){
      ros::param::get("data_path", this->data_path_);
      std::cout << "====================\nLoad Data" << std::endl;
      std::cout << "data_path = "  << this->data_path_ << std::endl;
      std::cout << "====================\n" << std::endl;

      /* 点群データのパブリッッシャーの定義 */
      this->pub_cloud_sensor_ = nh_.advertise<sensor_msgs::PointCloud2>("/sensor_data", 1);
      this->pub_cloud_ = nh_.advertise<PointCloud>("/PointCloud", 1);
      main();
    }
};

int main(int argc, char *argv[]){
  //ノードの初期化
  ros::init(argc, argv, "pointcloud_publisher_node");
  //PointcloudPublisherNodeのインスタンスを作成
  PointcloudPublisherNode pointcloud_publisher;
  ros::spin();
}
