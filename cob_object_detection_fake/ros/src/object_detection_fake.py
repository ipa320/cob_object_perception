#!/usr/bin/python
import roslib
roslib.load_manifest('cob_object_detection_fake')

import rospy

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *

from gazebo.srv import *

def handle_detect_object(req):
	
	srv_set_model_state = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
	res_set = srv_set_model_state()
	
	names=[]
	for name in res_set.model_names:
		if name == req.object_name.data:
			names.append(name)
			
	if len(names)==0:
		rospy.logwarn("%s not in database, searching for all objects", req.object_name.data)
		names = res_set.model_names

	resp = DetectObjectsResponse()
	
	for name in names:
		
		rospy.loginfo("fake detection of %s",name)
		
		srv_set_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		res_set = srv_set_model_state(name,'')
		
		detection=Detection()
		detection.pose.pose = res_set.pose
		detection.label = name
		
		resp.object_list.detections.insert(0,detection)
	
	return resp
	
def detect_object():	
	rospy.init_node('detect_object')
	s = rospy.Service('detect_object', DetectObjects, handle_detect_object)
	rospy.loginfo("Fake object detection ready.")
	rospy.spin()


if __name__ == "__main__":
	detect_object()

