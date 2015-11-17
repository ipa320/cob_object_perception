# ReadMe: Fast, edge-aware normal estimation

This is a brief explanation how to use/display the 3d egde detection and edge-aware normal estimation algorithms presented in IROS 2015 submission
Fast and Accurate Normal Estimation by Efficient 3d Edge Detection (Richard Bormann, Joshua Hampp and Martin Hägele).

Prerequisites
=============
Please install the following ROS packages:
```bash
sudo apt-get install ros-indigo-cob-common ros-indigo-cob-script-server libcgal-dev libwxgtk3.0-dev libdmtx-dev libmagick++-dev scons libfftw3-dev
```

You need to clone this repository into your catkin workspace as well as the following repositories:
- https://github.com/ipa-rmb/cob_environment_perception
- https://github.com/ipa-rmb/cob_perception_common
- https://github.com/ipa-rmb/cob_perception_data

Then build everything using one core with
```bash
catkin_make -j1
```

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
at entry "surface classification".

A window "color image with edges" will display the live 3d edge detection results with color coded depth edges (blue) and surface discontinuities (green). If you like to display the normals as well, activate the edge display window (i.e. click into it) and press key 'n'. You may change the normal estimation method to any other by changing the value of parameter 'ne_normal_estimation_method' in rqt_reconfigure.


