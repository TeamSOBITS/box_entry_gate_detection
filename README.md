<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

# box_entry_gate_detection

## 概要
本パッケージはゴミ箱の投入口を検出するために活用されます．RGB-Dカメラを用いて，3次元点群データを取り込み，その点群情報を処理してゴミ箱の投入口を正確に特定します．特に，物体をゴミ箱に捨てるというタスクにおいて，非常に役に立ちます．

## 環境
- OS : Ubuntu 20.04
- ROS distribution : Noetic Ninjemys
- PCL

## インストール

### PCLパッケージ
```bash
# システムをアップデート
$ sudo apt-get update
# libpcl-devパッケージをインストール
$ sudo apt-get install libpcl-dev
```
### インストール方法
```bash
# catkin_ws/srcに移動します
$ cd ~/catkin_ws/src
# 本パッケージをgit cloneします
$ git clone https://github.com/TeamSOBITS/box_entry_gate_detection.git
# パッケージのbuildをします
$ cd ~/catkin_ws && catkin_make
```

## 実行・操作方法
```
$ roslaunch box_entry_gate_detection box_detection.launch
```

## パッケージに関する情報
### Publications:
* /pcl_rosMsg [sensor_msgs/PointCloud2]
* /transform [sensor_msgs/PointCloud2]
* /cut_x_cloud [sensor_msgs/PointCloud2]
* /filter_cloud [sensor_msgs/PointCloud2]
* /voxel_grid [sensor_msgs/PointCloud2]
* /entry_gate_edge [sensor_msgs/PointCloud2]
* /box_entry_gate_detection/box_cluster [visualization_msgs/MarkerArray]
* /box_entry_gate_detection/box_placeable_point [visualization_msgs/MarkerArray]
* /box_entry_gate_detection/box_point [visualization_msgs/MarkerArray]
* /tf [tf2_msgs/TFMessage]

### Subscriptions:
 * /hsrb/head_rgbd_sensor/depth_registered/points [sensor_msgs/PointCloud2]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

## 注意事項
- 使用するRGB-Dカメラに合わせてlaunchファイルの以下のtopicのvalueの値を変更してください
```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

## ライセンス
- このウェブサイトのソースコードは，BSDライセンスに基づいてライセンスされており，[LICENSE](https://github.com/TeamSOBITS/box_entry_gate_detection/blob/noetic-devel/LICENSE)ファイルに記載されています．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>