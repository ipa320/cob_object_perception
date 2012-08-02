#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: cob_object_detection
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: July 2012
#
# \brief
# Implements Tests for Object Detection
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import os
import sys, subprocess
import time
import math
from math import fmod, pi
import signal
import types
import unittest

import roslib
roslib.load_manifest("cob_object_detection_msgs")

import rospy

import rostest
import rosparam

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from std_msgs.msg import String

from std_srvs.srv import Empty

import rosbag

from multiprocessing import Process

import yaml

class TestObjectDetection(unittest.TestCase):
	 
    
    def tearDown(self):

	os.system("killall play")
    
    def setUp(self):

	# positions and tolerance for comparing with the acquired data from the simulation or from the rosbag

		self.bagfiles = rospy.get_param("test_bag")
		self.PKG = rospy.get_param("PKG")
		self.mode = rospy.get_param("mode")

		self.bags = self.get_bags(self.bagfiles)
		self.tolerance = 0.5
		self.objID = None

    def updateTolerance(self, bag=0, chunk=0):

		if("tolerance" in chunk):
			self.tolerance = chunk['tolerance']

		elif("tolerance" in bag):
			self.tolerance = bag['tolerance']

		elif rospy.has_param('tolerance'):

		   	self.tolerance = rospy.get_param('tolerance')

###############################################################################
    ## Alternative function for launching the bagfiles
    def playback_bag(self, bagPath):
	os.system('rosbag play -d 5 -k --clock %s' % bagPath)
		  
    def get_bags(self, bagfiles):
 
		bags = []
	
		for item in bagfiles:
			bags.append(item)
		return bags

    def getNumberofObjects(self,inBag):

		self.objects = inBag['objects']
		if isinstance(self.objects, types.NoneType):
			objectsQTY = 0
		else:
	 		objectsQTY = len(self.objects)

		return objectsQTY

    def getLabel(self, index):

		self.objID = self.objects[index]['label']

    def process_objects(self, index):

		posX = (float)(self.objects[index]['position'][0])
        	posY = (float)(self.objects[index]['position'][1])
        	posZ = (float)(self.objects[index]['position'][2])

		return posX, posY, posZ

    def object_detector(self):

	for i in range(len(self.bags)):
			
			bagPath = roslib.packages.get_pkg_dir(self.PKG) + self.bags[i]['bag_path']
			yamlPath = roslib.packages.get_pkg_dir(self.PKG) + self.bags[i]['yaml_path']
#			if(i==1):
#				raise rospy.exceptions.ROSException("COB3 - has found no object!!%s"%bagPath, yamlPath)
			inBag = yaml.load(open(yamlPath).read())
			
			
	 # Wait the initialization of the Object Detection Service    
			if(self.mode == "delay"):
				rosbag_process = subprocess.Popen("rosbag play -d 2 -k --clock %s" % bagPath, shell=True)
			else:
				rosbag_process = subprocess.Popen("rosbag play --clock %s" % bagPath, shell=True)
			##out = rosbag_process.communicate()

		 	rospy.wait_for_service('/object_detection/detect_object', 10)
        		
		# Alternative bagfile launching
        		#bag_playback = Process(target=self.playback_bag, args=(bagPath,))
    			#bag_playback.start() 
			
        		
			try:
				objQTY = self.getNumberofObjects(inBag)
				
				for t in range(objQTY):

		    			recognition_service = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)

					req = DetectObjectsRequest()

					self.getLabel(t)
			    		req.object_name = String(self.objID)
						
			# Definition of the Region of Interest
			    		req.roi.x_offset = 0;
			    		req.roi.y_offset = 0;
			    		req.roi.width = 0;
			    		req.roi.height = 0;
					
			    		res = recognition_service(req)
					
					[posX, posY, posZ] =  self.process_objects(t)

					self.updateTolerance(bag = self.bags[i], chunk = self.objects[t])

					addInfo = "Object type: " + self.objID + ", path: " + bagPath + ", index: " + (str)(t) + "  " + (str)(posX)

					if(len(res.object_list.detections) > 0):
					
			    # Get the Cartesian Coordinates positions for the detected objects
				    		
				    		for i in range(len(res.object_list.detections)):

							positionX = res.object_list.detections[i].pose.pose.position.x
							positionY = res.object_list.detections[i].pose.pose.position.y
							positionZ = res.object_list.detections[i].pose.pose.position.z

						# Test assertions for guaranteeing the correct loading of the Object Detection System Component
							self.assertTrue(abs(positionX - posX) <= self.tolerance, "Failed on the x axis comparison%s"%addInfo)
							self.assertTrue(abs(positionY - posY) <= self.tolerance, "Failed on the y axis comparison%s"%addInfo)
							self.assertTrue(abs(positionZ - posZ) <= self.tolerance, "Failed on the z axis comparison%s"%addInfo)

			  		self.assertTrue(objQTY == len(res.object_list.detections), "Number of objects in the Bagfiles are not equal to number of objects found%s"%addInfo)

				
	        	except rospy.ServiceException, e:
	            		raise rospy.exceptions.ROSException("Service not available!!%s"%e)
       
	  		rosbag_process.send_signal(signal.SIGINT)
         	        os.kill(rosbag_process.pid, signal.SIGKILL)
			os.system("killall play")

		# Alternative bagfile launching

			#if bag_playback.is_alive():
        		#	rospy.loginfo('terminating playback process')
        		#	bag_playback.terminate()
        		#	time.sleep(1)
        		#	rospy.loginfo('playback process terminated? %s' % str(not bag_playback.is_alive()))

    def test_object_detection(self):
  	
		self.object_detector()
        
        
# Main Function for the tests run 
if __name__ == '__main__':

	rospy.init_node('test', anonymous=True)
	rostest.rosrun('cob_object_detection_msgs', 'Diagnostics',
              TestObjectDetection, sys.argv)
