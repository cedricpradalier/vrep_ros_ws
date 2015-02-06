#!/usr/bin/env python
import roslib
roslib.load_manifest('floor_mapper')

import rospy
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import math
import tf
import numpy

class FloorMapper:
    def __init__(self):
        self.proba = None
        self.info = None
        self.br = CvBridge()
        rospy.init_node('floor_projector')

        self.image_size = rospy.get_param("~floor_size_pix",1000)
        image_extent = rospy.get_param("~floor_size_meter",5.0)
        self.target_frame = rospy.get_param("~target_frame","/body")
        self.horizon_offset = rospy.get_param("~horizon_offset_pix",20)
        self.display_map = rospy.get_param("~display_map",True)

        self.floor_map = numpy.zeros((self.image_size,self.image_size))
        self.x_floor = 0.0
        self.y_floor = self.image_size / 2.0
        self.floor_scale = self.image_size / image_extent

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("~floor",Image,queue_size=1)
        rospy.Subscriber("~probabilities",Image,self.store_proba)
        rospy.Subscriber("~info",CameraInfo,self.store_info)
        rospy.loginfo("Waiting for first proba and camera info")
        while (not rospy.is_shutdown()) and ((self.info is None) or (self.proba is None)):
            rospy.sleep(0.1)

    def store_proba(self,proba):
        # print "Got Image"
        if not self.info:
            return
        # print "Processing"
        self.timestamp = proba.header.stamp
        if proba.encoding == "rgb8":
            I = self.br.imgmsg_to_cv2(proba,"rgb8")
            I = cv2.cvtColor(I,cv2.COLOR_RGB2GRAY)
        else:
            I = self.br.imgmsg_to_cv2(proba,"mono8")
        t, self.proba = cv2.threshold(I,0xFE,0xFE,cv2.THRESH_TRUNC)
        try:
            # (trans,rot) = self.listener.lookupTransform(proba.header.frame_id, '/world', proba.header.stamp)
            self.listener.waitForTransform(proba.header.frame_id,self.target_frame,proba.header.stamp,rospy.Duration(1.0))
            trans = numpy.mat(self.listener.asMatrix(self.target_frame,proba.header))
            # print "Transformation"
            # print trans
            dstdir = [trans * v for v in self.dirpts3d]
            # print "Destination dir"
            # print dstdir
            origin = trans * self.origin
            origin = origin / origin[3,0]
            # origin = numpy.matrix([0.0, 0.0, origin[2,0] / origin[3,0], 1.0]).T
            # print "Origin"
            # print origin

            self.dstpts2d = numpy.zeros((4,2))
            for i in range(4):
                self.dstpts2d[i,0] = self.x_floor + (origin[0,0] - dstdir[i][0,0]*origin[2,0]/dstdir[i][2,0])*self.floor_scale
                self.dstpts2d[i,1] = self.y_floor - (origin[1,0] - dstdir[i][1,0]*origin[2,0]/dstdir[i][2,0])*self.floor_scale
            # print numpy.asarray(self.dstpts2d)

            # print "Source points"
            # print numpy.asarray(self.srcpts2d)
            # print "Dest points"
            # print numpy.asarray(self.dstpts2d)
            self.H,_ = cv2.findHomography(self.srcpts2d,self.dstpts2d)
            # print "Homography"
            # print self.H
            # print numpy.asarray(self.H)

            size = (self.proba.shape[0]-self.horizon_offset,self.proba.shape[1])
            # print size

            self.floor_map = cv2.warpPerspective(self.proba[self.horizon_offset:,:],
                    self.H, (self.image_size,self.image_size), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0xFF)

            # cv2.imshow("proba",self.proba[self.horizon_offset:,:])
            # cv2.imshow("floor",self.floor_map)
            # cv2.waitKey(10)
            msg = self.br.cv2_to_imgmsg(self.floor_map,"mono8")
            msg.header.stamp = proba.header.stamp
            msg.header.frame_id = self.target_frame
            self.pub.publish(msg)
            # print "Publishing image : " + str(self.display_map)
            if self.display_map:
                marked=cv2.cvtColor(self.floor_map,cv2.COLOR_GRAY2RGB)
                idx=numpy.nonzero(self.floor_map == 0xFF)
                marked[idx] = (255,0,0)
                cv2.imshow("floor",marked)
                cv2.waitKey(10)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception while looking for transform"
            return
        
        
            

    def store_info(self,info):
        if not self.info:
            # assuming no distortion
            self.f = info.K[0]
            self.xc = info.K[2]
            self.yc = info.K[5]
            print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
            self.origin = numpy.zeros((4,1))
            self.origin[3,0] = 1.0

            self.horizon_offset = int(math.ceil(info.height/2. + self.horizon_offset))
            srcpts = [[0,info.height-1],[info.width-1,info.height-1],\
                    [0,self.horizon_offset], [info.width-1,self.horizon_offset]]
            self.srcpts2d = numpy.zeros((4,2))
            for i in range(4):
                self.srcpts2d[i,0] = srcpts[i][0]
                self.srcpts2d[i,1] = srcpts[i][1] - self.horizon_offset
            self.dirpts3d = []
            for i in range(4):
                v3 = numpy.matrix([-(srcpts[i][0]-self.xc) / self.f, 
                    -(srcpts[i][1]-self.yc) / self.f, 1.0, 0.0]).T
                n = math.sqrt((v3.T*v3).sum())
                self.dirpts3d.append(v3/n)
            self.info = info
        # print self.dirpts3d

    def run(self):
        rospy.loginfo("Starting floor projection")
        rospy.spin()

if __name__=="__main__":
    demo = FloorMapper()
    demo.run()

