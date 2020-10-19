# box_entry_gate_detection
ゴミ箱の投入口検出

## Enviromenet
- Ubuntu  : 16.04
- ROS : Kinetic
- PCL

## Check

```bash
$ roslaunch box_entry_gate_detection box_detection.launch
```
自分のPCに合わせてlaunchファイルの以下のtopicのvalueの値を変更してください

```bash
<param name="sub_point_topic_name" type="str" value="/hsrb/head_rgbd_sensor/depth/points"/>
```

