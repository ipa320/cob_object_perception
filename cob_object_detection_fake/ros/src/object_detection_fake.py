#!/usr/bin/python
import roslib
roslib.load_manifest('cob_object_detection_fake')

import rospy

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *

from gazebo.srv import *

def handle_detect_object(req):
	name = req.object_name.data

	rospy.loginfo("fake detection of %s",name)
		
	# get world properties from gazebo, get all objects in gazebo (including ground_plance etc.)
	srv_get_world_properties = rospy.ServiceProxy('gazebo/get_world_properties', GetWorldProperties)
	res_world = srv_get_world_properties()

	resp = DetectObjectsResponse()
	
	# check if name is an object in gazebo, otherwise return empty response
	if name not in res_world.model_names:
		rospy.logerr("object %s not available in gazebo, available objects are %s", name, res_world.model_names)
		return resp
	
	# get model state
	srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	res_model = srv_get_model_state(name,'')
	#print res_model

	try:
		# get link state
		srv_get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		res_link = srv_get_link_state(name + "::box_body","robot::head_axis_link")
		#print res_link
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		rospy.logerr("can't get pose of object %s from gazebo, model structure doesn't fit", name)
		return resp

	# compose Detection message
	detection=Detection()
	detection.pose.header.frame_id='/head_axis_link'
	detection.pose.pose = res_link.link_state.pose
	detection.label = name
	
	# insert object to detection_list
	resp.object_list.detections.insert(0,detection)

	rospy.loginfo("found object %s",name)
	return resp
	
def detect_object():	
	rospy.init_node('detect_object')
	s = rospy.Service('detect_object', DetectObjects, handle_detect_object)
	rospy.loginfo("Fake object detection ready.")
	rospy.spin()


if __name__ == "__main__":
	detect_object()

