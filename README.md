# box_entry_gate_detection
ゴミ箱の投入口検出

## Enviromenet
- Ubuntu  : 16.04
- ROS : Kinetic
- PCL

## How to use

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/TeamSOBITS/box_entry_gate_detection.git
$ git branch (robocup2020_TESTになっていたらOK)
    （masterの場合）
    $ git checkout robocup2020_TEST
$ cd ~/catkin_ws && catkin_make
$ roslaunch box_entry_gate_detection box_detection.launch
```
自分のPCに合わせてlaunchファイルの以下のtopicのvalueの値を変更してください

```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

