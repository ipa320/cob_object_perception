#! /usr/bin/env python

import roslib; roslib.load_manifest('cob_marker')
import rospy

import actionlib
import cob_object_detection_msgs.msg
from tf.transformations import *

def marker_client():
    client = actionlib.SimpleActionClient('/cob_marker/object_detection', cob_object_detection_msgs.msg.DetectObjectsAction)
    client.wait_for_server()

    goal = cob_object_detection_msgs.msg.DetectObjectsGoal()
    goal.object_name.data = "MAG"

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('cob_marker_client')
        result = marker_client()
        print "Result:"
        print result
        br = tf.TransformBroadcaster()
        for i in range(0,5):
            br.sendTransform((result.pose.position.x, result.pose.position.y,
                result.pose.position.z), (result.pose.orientation.x, result.pose.orientation.y,
                result.pose.orientation.z, result.pose.orientation.w),rospy.Time.now(),
                "obj_pose","/head_cam3d_link")
            rospy.sleep(0.05)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
