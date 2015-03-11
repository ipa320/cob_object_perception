# ReadMe: Fast, edge-aware normal estimation

This is a brief explanation how to use/display the 3d egde detection and edge-aware normal estimation algorithms.

Prerequisites
=============
You need to clone this repository into your catkin workspace as well as the following repositories:
- https://github.com/ipa-rmb/cob_environment_perception
- https://github.com/ipa-rmb/cob_perception_common
Then build everything.

Usage
=====
Start the Kinect driver, e.g.
```bash
roslaunch openni2_launch openni2.launch camera:=cam3d depth_registration:=true
```
Then read out the camera topic names using
```bash
rostopic list
```
and adapt the topic remapping in file
```bash
cob_surface_classification/ros/launch/surface_classification.launch
```
for node surface_classification according to your sensor topic names.

Finally, start 3d edge and normal estimation with
```bash
roslaunch cob_surface_classification surface_classification.launch
```

You may change the parameters of the algorithm at runtime using rqt_reconfigure
```bash
rosrun rqt_reconfigure rqt_reconfigure
```



