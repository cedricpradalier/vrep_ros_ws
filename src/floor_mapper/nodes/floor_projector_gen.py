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
        self.x_floor = rospy.get_param("~floor_origin_x_pix",0)
        self.y_floor = rospy.get_param("~floor_origin_y_pix",self.image_size/2)
        self.display_map = rospy.get_param("~display_map",True)

        self.floor_map = numpy.zeros((self.image_size,self.image_size))
        self.floor_scale = self.image_size / image_extent

        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("~floor",Image,queue_size=1)
        rospy.Subscriber("~image",Image,self.store_proba)
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
        else:
            I = self.br.imgmsg_to_cv2(proba,"mono8")
        try:
            self.floor_map = cv2.warpPerspective(I, # self.proba,
                    self.inv_H, (self.image_size,self.image_size), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0x00)

            # cv2.imshow("proba",self.proba[self.horizon_offset:,:])
            # cv2.imshow("floor",self.floor_map)
            # cv2.waitKey(10)
            msg = self.br.cv2_to_imgmsg(self.floor_map,proba.encoding)
            msg.header.stamp = proba.header.stamp
            msg.header.frame_id = self.target_frame
            self.pub.publish(msg)
            # print "Publishing image : " + str(self.display_map)
            if self.display_map:
                # idx=numpy.nonzero(self.floor_map == 0xFF)
                # marked[idx] = (255,0,0)
                cv2.imshow("floor",self.floor_map)
                cv2.waitKey(10)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Exception while looking for transform"
            return
        
        
            

    def store_info(self,info):
        # print("Got camera info: f %.2f C %.2f %.2f" % (self.f,self.xc,self.yc))
        # assuming no distortion
        self.listener.waitForTransform(info.header.frame_id,self.target_frame,info.header.stamp,rospy.Duration(1.0))
        trans = numpy.mat(self.listener.asMatrix(self.target_frame,info.header))
        K = numpy.mat([[-info.K[0], -info.K[1],info.K[2],0],
            [-info.K[3], -info.K[4],info.K[5],0],
            [info.K[6], info.K[7],info.K[8],0]])
        RT = numpy.mat([[trans[0,0],trans[0,1],trans[0,3]],
            [trans[1,0],trans[1,1],trans[1,3]],
            [trans[2,0],trans[2,1],trans[2,3]]])
        P2W = numpy.mat([[1.0/self.floor_scale,0.0,-self.x_floor/self.floor_scale],
            [0.0,-1.0/self.floor_scale,+self.y_floor/self.floor_scale],
            [0,0,0],
            [0,0,1]])
        self.H = K * numpy.linalg.inv(trans) * P2W
        self.inv_H = numpy.linalg.inv(self.H)
        self.inv_H = self.inv_H / self.inv_H[2,2]

        self.info = info

    def run(self):
        rospy.loginfo("Starting floor projection")
        rospy.spin()

if __name__=="__main__":
    demo = FloorMapper()
    demo.run()

