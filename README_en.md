<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

# box_entry_gate_detection

## Introduction
This package is utilized to detect the entry gate of a garbage box. By using an RGB-D camera, it captures 3D point cloud data, processes the point cloud information, and identifies the entry gate of the garbage box. This is particularly useful in tasks involving the disposal of objects into the garbage box.

## Environments
- OS : Ubuntu 20.04
- ROS distribution : Noetic Ninjemys
- PCL

## Installation

### PCL Package
```bash
# Update system
$ sudo apt-get update
# Install libpcl-dev package
$ sudo apt-get install libpcl-dev
```
### Installation Steps
```bash
# Move to catkin_ws/src
$ cd ~/catkin_ws/src
# Clone this package
$ git clone https://github.com/TeamSOBITS/box_entry_gate_detection.git
# Build the package
$ cd ~/catkin_ws && catkin_make
```

## How to use
```
$ roslaunch box_entry_gate_detection box_detection.launch
```

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

## Caution
- Modify the value of sub_point_topic_name in the launch file according to the RGB-D camera you are using.
```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

## License
- The source code of this website is licensed under the BSD License, and the details can be found in the [LICENSE](https://github.com/TeamSOBITS/box_entry_gate_detection/blob/noetic-devel/LICENSE) file.

<p align="right">(<a href="#readme-top">back to top</a>)</p>