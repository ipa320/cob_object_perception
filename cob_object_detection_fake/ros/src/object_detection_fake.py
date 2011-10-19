#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('cob_object_detection')
roslib.load_manifest('gazebo')

import rospy
import os
import tf

from cob_object_detection.srv import *
from cob_object_detection.msg import *

from gazebo.srv import *
from std_msgs.msg import String

def handle_detect_object(req):
	
	# Move training table on top of gripper
	srv_set_model_state = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
	res_set = srv_set_model_state()
		
	names=[]
	for n in res_set.model_names:
		if n.find(str(req.object_name.data))!=-1:
			names.insert(0,n)
			
	if len(names)==0:
		names = res_set.model_names

	r = DetectObjectsResponse()
	
	for n in names:
		
		print "simulating detection of "+n
		
		srv_set_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		res_set = srv_set_model_state(n,'')
		
		d=Detection()
		d.pose.pose = res_set.pose
		
		r.object_list.detections.insert(0,d)
	
	return r
	
def detect_object():	
	rospy.init_node('detect_object')
	s = rospy.Service('detect_object', DetectObjects, handle_detect_object)
	print "Ready."
	rospy.spin()


if __name__ == "__main__":
	detect_object()

