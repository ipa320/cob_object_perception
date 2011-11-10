#!/usr/bin/python
import roslib
roslib.load_manifest('cob_object_detection_fake')

import rospy

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *

from gazebo.srv import *

def handle_detect_object(req):
	
	srv_get_world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
	res_world = srv_get_world_properties()

	
	names=[]
	for name in res_world.model_names:
		if name == req.object_name.data:
			names.append(name)
			
	if len(names)==0:
		rospy.logwarn("%s not in database, searching for all objects", req.object_name.data)
		names = res_world.model_names

	resp = DetectObjectsResponse()

	for name in names:
		
		rospy.loginfo("fake detection of %s",name)
		
		srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		res_model = srv_get_model_state(name,'')

		print res_model

		srv_get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		res_link = srv_get_link_state(name + "::box_body","robot::head_axis_link")

		print res_link

		detection=Detection()
		detection.pose.header.frame_id='/head_axis_link'
		detection.pose.pose = res_link.link_state.pose
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

