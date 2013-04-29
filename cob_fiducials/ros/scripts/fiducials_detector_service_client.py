#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_object_detection')

import sys

import rospy

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *

from std_msgs.msg import String

def recognition_service_client(object_id):
    rospy.wait_for_service('/fiducials/get_fiducials')
    try:
        recognition_service = rospy.ServiceProxy('/fiducials/get_fiducials', DetectObjects)
	req = DetectObjectsRequest()
	req.object_name = String(object_id) #Not yet implemented
	req.roi.x_offset = 0; #Not yet implemented
	req.roi.y_offset = 0; #Not yet implemented
	req.roi.width = 0; #Not yet implemented
	req.roi.height = 0; #Not yet implemented
	print "[fiducials] request ", req
        res = recognition_service(req)
	if len(res.object_list.detections) <= 0:
		return 'failed'
	for i in range(0, len(res.object_list.detections)):
		print '------------------ fiducials {0:2d} ------------------'.format(i)
		print res.object_list.detections[i]

        return res
    except rospy.ServiceException, e:
        print "[fiducials] Service call failed: %s"%e

def usage():
    return "%s [object_name]"%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 2:
		object_id = sys.argv[1]
	else:
		print usage()
		sys.exit(1)
	recognition_service_client(object_id)

