cob_object_perception
===========

## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo_dev`](https://github.com/ipa320/cob_object_perception/tree/indigo_dev) | --- | --- |
| Status  |  supported | not supported |  not supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=cob_object_perception) | --- | --- |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ipa320/cob_object_perception.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/cob_object_perception)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| cob_object_perception | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__cob_object_perception__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__cob_object_perception__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__cob_object_perception__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__cob_object_perception__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__cob_object_perception__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__cob_object_perception__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__cob_object_perception__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__cob_object_perception__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__cob_object_perception__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__cob_object_perception__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__cob_object_perception__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__cob_object_perception__ubuntu_xenial_amd64__binary/) |


## Installation Prerequisites
Please download all other dependent packages from source using:
```
cd /path/to/catkin_workspace/src && git clone https://github.com/ipa320/cob_object_perception.git
cd /path/to/catkin_workspace/src && wstool init
cd /path/to/catkin_workspace/src && wstool merge /path/to/catkin_workspace/src/cob_object_perception/.travis.rosinstall
cd /path/to/catkin_workspace/src && wstool update

```
And then install all released dependencies:
```
cd /path/to/catkin_workspace && rosdep install --from-path src/ -y -i
```
