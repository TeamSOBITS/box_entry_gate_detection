<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# box_entry_gate_detection

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
    <li><a href="#パラメータ">パラメータ</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>

## 概要
本パッケージはゴミ箱の投入口を検出するために活用されます．RGB-Dカメラを用いて，3次元点群データを取り込み，その点群情報を処理してゴミ箱の投入口を正確に特定します．特に，物体をゴミ箱に捨てるというタスクにおいて，非常に役に立ちます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- セットアップ -->
## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法
1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd ~/colcon_ws/src/
   ```
2. 本リポジトリをcloneします．
   ```sh
   $ git clone -b humble https://github.com/TeamSOBITS/box_entry_gate_detection
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
   $ cd ~/colcon_ws/
   $ colcon build --symlink-install
   $ source ~/colcon_ws/install/setup.sh
   ```
<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 実行・操作方法
```
$ ros2 launch box_entry_gate_detection box_detection.launch.py
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パラメータ
以下が[box_detection.launch.py](launch/box_detection.launch.py)で設定できるパラメータです.

| パラメータ名 | 説明 | デフォルト値 |
|:---:|:---:|:---:|
| execute_default | 起動時処理を行うかどうか | false |
| sub_point_topic_name | subscribeする点群のtopic名 | /head_rgbd_sensor/depth_registered/points|
| base_frame_name | 基準フレーム名 | base_footprint|
| depth_range_min_x | 処理を行う範囲，x軸の最小値 | 0.0|
| depth_range_max_x | 処理を行う範囲，x軸の最大値 | 1.5|
| depth_range_min_z | 処理を行う範囲，z軸の最小値 | 0.1|
| depth_range_max_z | 処理を行う範囲，z軸の最大値 | 0.7|
| cluster_ss | クラスタリング時，距離の閾値 | 0.05|
| shift_x | tfの位置を調整する値（x軸） | 0.0|
| shift_y | tfの位置を調整する値（y軸） | 0.0|
| shift_z | tfの位置を調整する値（z軸） | 0.0|

> [!WARNING]
> 使用するRGB-Dカメラに合わせてtopic_nameのvalueの値を変更してください

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パッケージに関する情報
### Publications:
* /entry_gate_edge [sensor_msgs/PointCloud2]
* /box_entry_gate_detection/box_cluster [visualization_msgs/MarkerArray]
* /box_entry_gate_detection/box_placeable_point [visualization_msgs/MarkerArray]
* /box_entry_gate_detection/box_point [visualization_msgs/MarkerArray]
* /tf [tf2_msgs/TFMessage]


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/box_entry_gate_detection.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/box_entry_gate_detection/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/box_entry_gate_detection.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/box_entry_gate_detection/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/box_entry_gate_detection.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/box_entry_gate_detection/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/box_entry_gate_detection.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/box_entry_gate_detection/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/box_entry_gate_detection.svg?style=for-the-badge
[license-url]: LICENSE

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>
