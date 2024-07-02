<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# box_entry_gate_detection

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#launch-and-usage">Launch and Usage</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
  </ol>
</details>

## Introduction
This package is utilized to detect the entry gate of a garbage box. By using an RGB-D camera, it captures 3D point cloud data, processes the point cloud information, and identifies the entry gate of the garbage box. This is particularly useful in tasks involving the disposal of objects into the garbage box.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ cd src/
   ```
2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/box_entry_gate_detection
   ```
3. Navigate into the repository.
   ```sh
   $ cd box_entry_gate_detection/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```
5. Compile the package.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Launch and Usage
```
$ roslaunch box_entry_gate_detection box_detection.launch
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Package Information
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
> Modify the value of sub_point_topic_name in the launch file according to the RGB-D camera you are using.
```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

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

<p align="right">(<a href="#readme-top">back to top</a>)</p>