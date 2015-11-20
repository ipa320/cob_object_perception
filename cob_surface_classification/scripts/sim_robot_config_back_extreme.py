#!/usr/bin/env python

PKG = 'cob_surface_classification'
NODE = 'sim_robot_config_back'

import roslib; roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
sss = simple_script_server()

def main():
	# init node
	rospy.init_node(NODE)
	#print "==> node %s started" %NODE
	rospy.sleep(0.5)

	# set torso parameters
	#rospy.set_param('/script_server/arm/wave_in', [[1.5, 0.25, 0.0, -1.0, 0.0, 1.5, 0.0]])
	#rospy.set_param('/script_server/arm/wave_out', [[1.5, 0.0, 0.0, -0.75, 0.0, 1.0, 0.0]])
	rospy.set_param('/script_server/torso/default_vel', [1.5])
	sss.move("torso", "back_extreme")

if __name__ == "__main__":
	main()