<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

# box_entry_gate_detection

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>

## 概要
本パッケージはゴミ箱の投入口を検出するために活用されます．RGB-Dカメラを用いて，3次元点群データを取り込み，その点群情報を処理してゴミ箱の投入口を正確に特定します．特に，物体をゴミ箱に捨てるというタスクにおいて，非常に役に立ちます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- セットアップ -->
## 環境構築

ここで，本リポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法
1. ROSの`src`フォルダに移動します．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ cd src/
   ```
2. 本リポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/box_entry_gate_detection
   ```
3. リポジトリの中へ移動します．
   ```sh
   $ cd box_entry_gate_detection/
   ```
4. 依存パッケージをインストールします．
   ```sh
   $ bash install.sh
   ```
5. パッケージをコンパイルします．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ catkin_make
   ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


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

> [!WARNING]
> 使用するRGB-Dカメラに合わせてlaunchファイルの以下のtopicのvalueの値を変更してください
```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>
