#!/usr/bin/python

PKG = 'cob_datamatrix' # this package name
import roslib; roslib.load_manifest(PKG)

import ctypes
import struct
import math
from functools import partial
import array
import time
import Queue
import threading

import numpy
import copy
import rospy

from cob_object_detection_msgs.srv import *
from sensor_msgs.msg import *
from std_msgs.msg import *

def TriggerDataMatrix():
    rospy.wait_for_service('/object_detection/detect_object')
    try:
        cb = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
        resp1 = cb(String(), RegionOfInterest())
        return resp1.object_list
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def main():
    TriggerDataMatrix()

if __name__ == "__main__":
    main()
