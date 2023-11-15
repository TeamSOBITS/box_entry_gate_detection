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

## Caution
- Modify the value of sub_point_topic_name in the launch file according to the RGB-D camera you are using.
```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>