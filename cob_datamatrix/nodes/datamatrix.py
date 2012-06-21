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
import cv
import cv_bridge
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import geometry_msgs.msg
from geometry_msgs.msg import *

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from cob_datamatrix.srv import *

import tf
import tf_conversions.posemath as pm
import PyKDL
import tf.msg

import pydmtx


_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from
    
    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    yield unpack_from(data, offset)
                    offset += point_step
                    
                    
def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    #print "Queue empty"
                    break
            #print "run"
            self.function(m)

class DataMatrix:
    def __init__(self):
        self.i = 0
        self.tracking = {}

        self.listener = tf.TransformListener()
        self.srv = rospy.Service('/object_detection/detect_object', DetectObjects, self.handle_find)
        self.srv_dm = rospy.Service('/cobject_detection/trigger_datamatrix', DetectObjects, self.handle_find)
        self.find = False
        self.triggermode = rospy.get_param('~triggermode', False)

        self.pointcloud = None
        self.dm = pydmtx.DataMatrix()

        if self.triggermode==False: self.subscribe()

        self.pub_detection = rospy.Publisher("/obj_detection", cob_object_detection_msgs.msg.Detection)

    def subscribe(self):
            def topic(lr, sub):
                return rospy.resolve_name("stereo") + "/%s/%s" % ({'l' : 'left', 'r' : 'right'}[lr], sub)

            tosync_stereo = [
                (topic('l', "image_rect_color"), sensor_msgs.msg.Image),
                (topic('l', "camera_info"), sensor_msgs.msg.CameraInfo),
                (topic('r', "image_rect_color"), sensor_msgs.msg.Image),
                (topic('r', "camera_info"), sensor_msgs.msg.CameraInfo)
            ]
            tosync_mono = [
                ("image_stream", sensor_msgs.msg.Image),
                ("camera_info", sensor_msgs.msg.CameraInfo),
            ]
            tosync_rgbd = [
                ("/image_rgb", sensor_msgs.msg.Image),
                ("/camera_info", sensor_msgs.msg.CameraInfo),
                #("/camera/depth/points", sensor_msgs.msg.PointCloud2)
            ]

            self.pcl_sub = rospy.Subscriber("pointcloud_depth", sensor_msgs.msg.PointCloud2, self.pc_cb)

            #self.q_stereo = Queue.Queue()
            #tss = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_stereo], 10)
            #tss.registerCallback(self.queue_stereo)

            #sth = ConsumerThread(self.q_stereo, self.handle_stereo)
            #sth.setDaemon(True)
            #sth.start()

            self.subscribers_mono = [message_filters.Subscriber(topic, type) for (topic, type) in tosync_mono]
            self.subscribers_rgbd = [message_filters.Subscriber(topic, type) for (topic, type) in tosync_rgbd]

            self.q_mono = Queue.Queue()
            tss = message_filters.TimeSynchronizer(self.subscribers_mono, 10)
            tss.registerCallback(self.queue_mono)


            mth = ConsumerThread(self.q_mono, self.handle_mono)
            mth.setDaemon(True)
            mth.start()

            self.q_rgbd = Queue.Queue()
            tss = message_filters.TimeSynchronizer(self.subscribers_rgbd, 1)
            tss.registerCallback(self.queue_rgbd)

            rth = ConsumerThread(self.q_rgbd, self.handle_rgbd)
            rth.setDaemon(True)
            rth.start()


    def unsubscribe(self):
            self.pcl_sub.unregister()
            #[s.unregister() for s in self.subscribers_rgbd]
            #[s.unregister() for s in self.subscribers_mono]


        # message_filters.Subscriber("/wide_stereo/left/image_raw", sensor_msgs.msg.Image).registerCallback(self.getraw)

    def pc_cb(self, a):
        #print "got cookies"
        self.pointcloud = a

    def handle_find(self, para):
        timeout = rospy.get_param('~timeout', 60)
        names = []
        if isinstance(para,DetectDatamatrixRequest):
                timeout = para.timeout
                names = para.names
        else:
                names = [para.object_name.data]

        print "called"
        start_tm = time.clock()
        self.subscribe()
        while True:
            self.find = True
            while self.find == True and (timeout<0 or time.clock()-start_tm<timeout):
                print "sleeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep"
                time.sleep(0.1)
            if self.find==True:
                break
            #transfrom object position to an pose
            pose_obj = PoseStamped()
            pose_obj_bl = PoseStamped()
            pose_obj.header.frame_id = "/head_color_camera_r_link"
        
            pose_obj.pose = copy.deepcopy(self.current_pose)

            if (self.last_detection_msg.label in names or len(names)==0) and not math.isnan(self.last_detection_msg.pose.pose.position.x):
                print "FOUND FOUND"
                self.tracking = {}
                resp = DetectionArray()
                resp.detections.append(self.last_detection_msg)
                #resp.marker = pose_obj_bl.pose.position
                    #print "PoseCL: ", self.current_pose
                    #print "PoseBL: ", pose_obj_bl
                self.unsubscribe()
                a = DetectObjectsResponse()
                a.object_list = resp
                return a
            else: break
        print "nothing found"
        self.unsubscribe()
        self.tracking = {}

        return DetectObjectsResponse()

    def getraw(self, rawimg):
        print "raw", rawimg.encoding
        rawimg.encoding = "mono8"
        img = CvBridge().imgmsg_to_cv(rawimg)
        cv.SaveImage("/tmp/raw.png", img)

    def got_info(self, lr, cammsg):
        cam = image_geometry.PinholeCameraModel()
        cam.fromCameraInfo(cammsg)
        self.cams[lr] = cam

    def track(self, img):
        print "track"
        if len(self.tracking) == 0:
            print "start decode"
            self.dm.decode(img.width,
                      img.height,
                      buffer(img.tostring()),
                      max_count = 1,
                      )
            print "decode done"
            if self.dm.count() > 0:
                    (code, corners) =  self.dm.stats(1)
                    self.tracking[code] = corners
                    print "found: ", code
        else:
            for (code, corners) in self.tracking.items():
                print "tracking: ", code
                xs = [x for (x,y) in corners]
                ys = [y for (x,y) in corners]
                border = 32
                x0 = max(0, min(xs) - border)
                x1 = min(img.width, max(xs) + border)
                y0 = max(0, min(ys) - border)
                y1 = min(img.height, max(ys) + border)
                sub = cv.GetSubRect(img, (x0, y0, x1 - x0, y1 - y0))
                cv.PolyLine(img, [ corners ], True, cv.RGB(0, 255, 0))
                if 0:
                    cv.ShowImage("DataMatrix", img)
                    cv.WaitKey(6)
                self.dm.decode(sub.width,
                          sub.height,
                          buffer(sub.tostring()),
                          max_count = 1,
                          )
                if self.dm.count() == 1:
                    (code, newcorners) = self.dm.stats(1)
                    self.tracking[code] = [(x0 + x, y0 + y) for (x, y) in newcorners]
                    print "self.tracking[code] ", self.tracking[code]
                else:
                    print "lost", code
                    del self.tracking[code]
                    # rospy.signal_shutdown(0)
        print "tracking done"

    def toDetectionMsg(self, header, code, posemsg):
        ts = cob_object_detection_msgs.msg.Detection()
        ts.header.frame_id = header.frame_id
        ts.header.stamp = header.stamp
        ts.label = code
        ts.detector = "datamatrix"

        ts.pose.pose.position.x = posemsg.position[0]
        ts.pose.pose.position.y = posemsg.position[1]
        ts.pose.pose.position.z = posemsg.position[2]
        ts.pose.pose.orientation = posemsg.orientation

        rot=PyKDL.Rotation()
        rot.DoRotX(-math.pi/2)
        ts.pose.pose = pm.toMsg( PyKDL.Frame(rot,PyKDL.Vector(0,0,0))*pm.fromMsg(ts.pose.pose) ) #switch z and y according to Jan Fischer's description

        #tfm = tf.msg.tfMessage([ts])
        print ts.pose.pose
        return ts

    def broadcast(self, header, code, posemsg):
        self.last_detection_msg=self.toDetectionMsg(header, code, posemsg)
        self.pub_detection.publish(self.last_detection_msg)
        #inform trigger
        self.current_pose = Pose()
        self.current_pose.position.x = posemsg.position[0]
        self.current_pose.position.y = posemsg.position[0]
        self.current_pose.position.z = posemsg.position[0]
        self.find = False

    def gotimage(self, imgmsg):
        if imgmsg.encoding == "bayer_bggr8":
            imgmsg.encoding = "8UC1"
        img = CvBridge().imgmsg_to_cv(imgmsg)
        # cv.ShowImage("DataMatrix", img)
        # cv.WaitKey(6)
        print "gotimage"
        if(self.find == False):
                return
        self.track(img)
        self.find = False

        # monocular case
        if 0 and 'l' in self.cams:
            for (code, corners) in self.tracking.items():
                model = cv.fromarray(numpy.array([[0,0,0], [.1, 0, 0], [.1, .1, 0], [0, .1, 0]], numpy.float32))

                rot = cv.CreateMat(3, 1, cv.CV_32FC1)
                trans = cv.CreateMat(3, 1, cv.CV_32FC1)
                cv.FindExtrinsicCameraParams2(model,
                                              cv.fromarray(numpy.array(corners, numpy.float32)),
                                              self.cams['l'].intrinsicMatrix(),
                                              self.cams['l'].distortionCoeffs(),
                                              rot,
                                              trans)

                self.broadcast(imgmsg)
                #ts = geometry_msgs.msg.TransformStamped()
                #ts.header.frame_id = imgmsg.header.frame_id
                #ts.header.stamp = imgmsg.header.stamp
                #ts.child_frame_id = code
                #posemsg = pm.toMsg(pm.fromCameraParams(cv, rot, trans))
                #ts.transform.translation = posemsg.position
                #ts.transform.rotation = posemsg.orientation

                #tfm = tf.msg.tfMessage([ts])
                #self.pub_tf.publish(tfm)

    def queue_mono(self, img, caminfo):
        #print "queue_mono"
        qq = (img, caminfo)
        self.q_mono.put(qq)

    def queue_stereo(self, lmsg, lcmsg, rmsg, rcmsg):
        qq = (lmsg, lcmsg, rmsg, rcmsg)
        self.q_stereo.put(qq)

    def queue_rgbd(self, img, caminfo):
        #print "blub"
        qq = (img, caminfo, self.pointcloud)
        self.q_rgbd.put(qq)

    def handle_mono(self, qq):
        (imgmsg, caminfo) = qq
        #print "mono"
        img = CvBridge().imgmsg_to_cv(imgmsg, "mono8")
        pcm = image_geometry.PinholeCameraModel()
        pcm.fromCameraInfo(caminfo)
        if(self.find == False):
            return
        self.track(CvBridge().imgmsg_to_cv(imgmsg, "rgb8"))

        for (code, corners) in self.tracking.items():
            print corners
            cx = 0.0
            cy = 0.0
            count = 0.0
            for (x,y) in corners:
                cx += x
                cy += y
                count += 1.0
            cx /= count
            cy /= count

            model = cv.fromarray(numpy.array([[0,0,0], [.05, 0, 0], [.05, .05, 0], [0, .05, 0]], numpy.float32))

            rot = cv.CreateMat(3, 1, cv.CV_32FC1)
            trans = cv.CreateMat(3, 1, cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(model,
                                          cv.fromarray(numpy.array(corners, numpy.float32)),
                                          pcm.intrinsicMatrix(),
                                          pcm.distortionCoeffs(),
                                          rot,
                                          trans)

            print int(cx), int(cy)
            points = read_points(self.pointcloud, None, False, [[int(cx), int(cy)]])
            print points
            self.current_pose = Pose()
            for point in points:
                print type(point)
                self.current_pose.position.x = point[0]
                self.current_pose.position.y = point[1]
                self.current_pose.position.z = point[2]
                rospy.loginfo(point)
            self.find = False
                
            #create TF message
            #ts = geometry_msgs.msg.TransformStamped()
            #ts.header.frame_id = imgmsg.header.frame_id
            #ts.header.stamp = imgmsg.header.stamp
            #ts.child_frame_id = code
            #ts.transform.translation = posemsg.position
            #ts.transform.rotation = posemsg.orientation
            #tfm = tf.msg.tfMessage([ts])
            #self.pub_tf.publish(tfm)

    def handle_stereo(self, qq):
        (lmsg, lcmsg, rmsg, rcmsg) = qq
        limg = CvBridge().imgmsg_to_cv(lmsg, "mono8")
        rimg = CvBridge().imgmsg_to_cv(rmsg, "mono8")

        if 0:
            cv.SaveImage("/tmp/l%06d.png" % self.i, limg)
            cv.SaveImage("/tmp/r%06d.png" % self.i, rimg)
            self.i += 1

        scm = image_geometry.StereoCameraModel()
        scm.fromCameraInfo(lcmsg, rcmsg)

        bm = cv.CreateStereoBMState()
        if "wide" in rospy.resolve_name("stereo"):
            bm.numberOfDisparities = 160
        if 0:
            disparity = cv.CreateMat(limg.rows, limg.cols, cv.CV_16SC1)
            started = time.time()
            cv.FindStereoCorrespondenceBM(limg, rimg, disparity, bm)
            print time.time() - started
            ok = cv.CreateMat(limg.rows, limg.cols, cv.CV_8UC1)
            cv.CmpS(disparity, 0, ok, cv.CV_CMP_GT)
            cv.ShowImage("limg", limg)
            cv.ShowImage("disp", ok)
            cv.WaitKey(6)
        self.track(CvBridge().imgmsg_to_cv(lmsg, "rgb8"))
        if len(self.tracking) == 0:
            print "No markers found"
        for code, corners in self.tracking.items():
            corners3d = []
            for (x, y) in corners:
                limr = cv.GetSubRect(limg, (0, y - bm.SADWindowSize / 2, limg.cols, bm.SADWindowSize + 1))
                rimr = cv.GetSubRect(rimg, (0, y - bm.SADWindowSize / 2, rimg.cols, bm.SADWindowSize + 1))
                tiny_disparity = cv.CreateMat(limr.rows, limg.cols, cv.CV_16SC1)
                cv.FindStereoCorrespondenceBM(limr, rimr, tiny_disparity, bm)
                if tiny_disparity[7, x] < 0:
                    return
                corners3d.append(scm.projectPixelTo3d((x, y), tiny_disparity[7, x] / 16.))
                if 0:
                    cv.ShowImage("d", disparity)
            (a, b, c, d) = [numpy.array(pt) for pt in corners3d]
            def normal(s, t):
                return (t - s) / numpy.linalg.norm(t - s)
            x = PyKDL.Vector(*normal(a, b))
            y = PyKDL.Vector(*normal(a, d))
            f = PyKDL.Frame(PyKDL.Rotation(x, y, x * y), PyKDL.Vector(*a))
            msg = pm.toMsg(f)
            # print "%10f %10f %10f" % (msg.position.x, msg.position.y, msg.position.z)
            print code, msg.position.x, msg.position.y, msg.position.z
            self.broadcast(lmsg.header, code, msg)

    def fromCameraParams(self, cv, rvec, tvec):
                m = numpy.array([ [ 0, 0, 0, tvec[0,0] ],
                                  [ 0, 0, 0, tvec[1,0] ], 
                                  [ 0, 0, 0, tvec[2,0] ], 
                                  [ 0, 0, 0, 1.0       ] ], dtype = numpy.float32)
                cv.Rodrigues2(rvec, cv.fromarray(m[:3,:3]))
                return pm.fromMatrix(m)

    def handle_rgbd(self, qq):
        if self.triggermode==True and self.find==False:
            return
        (imgmsg, caminfo, pointcloud) = qq
        if not pointcloud or pointcloud._type != 'sensor_msgs/PointCloud2':
            return
        print "rgbd"
        img = CvBridge().imgmsg_to_cv(imgmsg, "mono8")
        pcm = image_geometry.PinholeCameraModel()
        pcm.fromCameraInfo(caminfo)

        self.track(CvBridge().imgmsg_to_cv(imgmsg, "rgb8"))

        for (code, corners) in self.tracking.items():
                cx = 0.0
                cy = 0.0
                count = 0.0
                for (x,y) in corners:
                        cx += x
                        cy += y
                        count += 1.0
                cx /= count
                cy /= count      


                model = cv.fromarray(numpy.array([[0,0,0], [.1, 0, 0], [.1, .1, 0], [0, .1, 0]], numpy.float32))

                rot = cv.CreateMat(3, 1, cv.CV_32FC1)
                trans = cv.CreateMat(3, 1, cv.CV_32FC1)
                cv.FindExtrinsicCameraParams2(model,
                                              cv.fromarray(numpy.array(corners, numpy.float32)),
                                              pcm.intrinsicMatrix(),
                                              pcm.distortionCoeffs(),
                                              rot,
                                              trans)  

                #ts = geometry_msgs.msg.TransformStamped()
                #ts.header.frame_id = imgmsg.header.frame_id
                #ts.header.stamp = imgmsg.header.stamp
                #ts.child_frame_id = code
                #rot = cv.fromarray(rot)
                #trans = cv.fromarray(trans)
                posemsg = pm.toMsg(self.fromCameraParams(cv, rot, trans))
                points = read_points(pointcloud, None, False, [(int(cx), int(cy))])
                #pointcloud.data(cy*pointcloud.row_step+cx*pointcloud.point_step)
                print type(points)
                for point in points:
                    print type(point)
                    print point
                    posemsg.position = point
                #ts.transform.rotation = posemsg.orientation
                #print pcm.fullProjectionMatrix()
                #posemsg.position = points[0]
                print "Pose: ", posemsg.position
                self.broadcast(imgmsg.header, code, posemsg)

                #tfm = tf.msg.tfMessage([ts])
                #self.pub_tf.publish(tfm)
        #self.not_found += 1

def main():
    rospy.init_node('cob_datamatrix')
    node = DataMatrix()
    rospy.spin()

if __name__ == "__main__":
    main()
