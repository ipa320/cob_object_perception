#!/usr/bin/python
import roslib
roslib.load_manifest('cob_object_detection_fake')

import rospy

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *

from gazebo.srv import *

from numpy import *
from numpy.linalg import inv

import tf

roslib.load_manifest('cob_script_server')
from simple_script_server import *


def handle_detect_object(req):
	name = req.object_name.data
	tf_listener = tf.TransformListener()
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
	res_model = srv_get_model_state(name,'robot::head_axis_link')
	trans_o_h = (res_model.pose.position.x, res_model.pose.position.y, res_model.pose.position.z)
	rot_o_h = (res_model.pose.orientation.x, res_model.pose.orientation.y, res_model.pose.orientation.z, res_model.pose.orientation.w)
	
	frame_o_h = quaternion_matrix(rot_o_h)
	frame_o_h[0][3] = trans_o_h[0]
	frame_o_h[1][3] = trans_o_h[1]
	frame_o_h[2][3] = trans_o_h[2]

	print "frame_o_h"
	print frame_o_h
	
	tf_listener.waitForTransform('/head_color_camera_l_link', '/head_axis_link', rospy.Time(0), rospy.Duration(1))
	(trans_h_l, rot_h_l) = tf_listener.lookupTransform('/head_color_camera_l_link', '/head_axis_link', rospy.Time(0))
	
	frame_h_l = quaternion_matrix(rot_h_l)
	frame_h_l[0][3] = trans_h_l[0]
	frame_h_l[1][3] = trans_h_l[1]
	frame_h_l[2][3] = trans_h_l[2]

	print "frame_h_l"
	print frame_h_l

	#frame = numpy.linalg.inv(frame)
	#rot = quaternion_from_matrix(frame)
	#tmp_mat = hsplit(frame, [3])
	#trans = vsplit(tmp_mat[1], [3])[0]

	# Transform
	frame_o_l = numpy.dot(frame_h_l, frame_o_h)
	print "frame_o_l"
	print frame_o_l

	rot_o_l = quaternion_from_matrix(frame_o_l)
	tmp_mat = hsplit(frame_o_l, [3])
	trans_o_l = vsplit(tmp_mat[1], [3])[0]

	# Only valid for milk box
	box_min = (-0.06, -0.095, 0, 1)
	box_max = (0.06, 0.095, 0.2, 1)

	print "BOX MIN"
	print box_min
	box_min = numpy.dot(frame_o_l, box_min)
	print box_min


	print "BOX MAX"		
	print box_max
	box_max = numpy.dot(frame_o_l, box_min)
	print box_max

	# compose Detection message
	detection=Detection()
	detection.pose.header.frame_id='/head_color_camera_l_link'
	detection.pose.pose.position.x = trans_o_l[0]
	detection.pose.pose.position.y = trans_o_l[1]
	detection.pose.pose.position.z = trans_o_l[2]
	detection.pose.pose.orientation.x = rot_o_l[0]
	detection.pose.pose.orientation.y = rot_o_l[1]
	detection.pose.pose.orientation.z = rot_o_l[2]
	detection.pose.pose.orientation.w = rot_o_l[3]
	detection.label = name
	detection.bounding_box_min.x = box_min[0]
	detection.bounding_box_min.y = box_min[1]
	detection.bounding_box_min.z = box_min[2]
	detection.bounding_box_max.x = box_max[0]
	detection.bounding_box_max.y = box_max[1]
	detection.bounding_box_max.z = box_max[2]

	# insert object to detection_list
	resp.object_list.detections.insert(0,detection)

	rospy.loginfo("found object %s at:",name)
	print frame_o_l
	return resp

def detect_object():	
	rospy.init_node('detect_object')
	s = rospy.Service('detect_object', DetectObjects, handle_detect_object)


	rospy.loginfo("Fake object detection ready.")
	rospy.spin()


if __name__ == "__main__":
	detect_object()

