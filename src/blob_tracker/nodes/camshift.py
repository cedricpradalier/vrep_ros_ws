#!/usr/bin/env python

'''
Camshift tracker
================
This is a demo that shows mean-shift based tracking
You select a color objects such as your face and it tracks it.
This reads from video camera (0 by default, or the camera number the user enters)
http://www.robinhewitt.com/research/track/camshift.html
Usage:
------
    camshift.py [<video source>]
    To initialize tracking, select the object with mouse
Keys:
-----
    ESC,q - exit
    b     - toggle back-projected probability visualization
    h     - toggle histogram visualization
'''

import numpy as np
import cv2

# local module
import rospy
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge

def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)


class App(object):
    def __init__(self, video_src):
        self.frame = np.zeros((320,240,3), np.uint8)
        self.hist = np.zeros((320,240,3), np.uint8)
        self.backproject = np.zeros((320,240,3), np.uint8)
        self.br = CvBridge()
        self.pause = False
        cv2.namedWindow('camshift')
        # cv2.namedWindow('hist')
        cv2.setMouseCallback('camshift', self.onmouse)

        self.selection = None
        self.drag_start = None
        self.tracking_state = 0
        self.show_backproj = False

    def onmouse(self, event, x, y, flags, param):
        down = [cv2.EVENT_LBUTTONDOWN,cv2.EVENT_MBUTTONDOWN,cv2.EVENT_RBUTTONDOWN]
        up = [cv2.EVENT_LBUTTONUP,cv2.EVENT_MBUTTONUP,cv2.EVENT_RBUTTONUP]
        flagb = cv2.EVENT_FLAG_LBUTTON | cv2.EVENT_FLAG_MBUTTON | cv2.EVENT_FLAG_RBUTTON;
        x, y = np.int16([x, y]) # BUG
        if event in down:
            self.drag_start = (x, y)
            self.tracking_state = 0
            rospy.loginfo("L/R button down")
        if event in up:
            rospy.loginfo("L/R button up")
            self.drag_start = None
            if self.selection is not None:
                rospy.loginfo("Completed " + str(self.selection))
                self.tracking_state = 1
        if self.drag_start is not None:
            # rospy.loginfo("Dragging %04X / %04X",flags,flagb)
            h, w = self.frame.shape[:2]
            xo, yo = self.drag_start
            x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
            x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
            self.selection = None
            if x1-x0 > 0 and y1-y0 > 0:
                self.selection = (x0, y0, x1, y1)
                # rospy.loginfo("Dragging " + str(self.selection))

    def hue_histogram_as_image(self,hist):
        bin_count = hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        return img

    def detect_and_draw(self,imgmsg):
        if self.pause:
            return
        frame = self.br.imgmsg_to_cv2(imgmsg, "bgr8")
        self.frame = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

        # rospy.loginfo("Selection %s / TS : %d" % (str(self.selection),self.tracking_state))

        if self.selection:
            x0, y0, x1, y1 = self.selection
            #print(self.selection)
            self.track_window = (x0, y0, x1-x0, y1-y0)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
            self.hist = hist.reshape(-1)

            vis_roi = self.frame[y0:y1, x0:x1]
            cv2.bitwise_not(vis_roi, vis_roi)
            self.frame[mask == 0] = 0

        if self.tracking_state == 1 and is_rect_nonzero(self.track_window):
            self.selection = None
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
            track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
            self.backproject = prob[...,np.newaxis].copy()

            if self.show_backproj:
                self.frame[:] = self.backproject
            try:
                cv2.ellipse(self.frame, track_box, (0, 0, 255), 2)
            except:
                print track_box
            x,y,w,h = self.track_window
            self.bbpub.publish(RegionOfInterest(x,y, w,h,False))
            proba_msg = self.br.cv2_to_imgmsg(self.backproject,"mono8")
            proba_msg.header = imgmsg.header
            self.bppub.publish(proba_msg)

    def run(self):
        self.backproject_mode = False
        rospy.init_node('blob_tracker')
        image_topic = rospy.resolve_name("image")
        self.bbpub = rospy.Publisher("~blob",RegionOfInterest,queue_size=1)
        self.bppub = rospy.Publisher("~backproject",Image,queue_size=1)
        self.disp_hist = rospy.get_param("~display_histogram",False)
        rospy.Subscriber(image_topic, Image, self.detect_and_draw)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # rospy.spin_once()
            if not self.pause:
                if not self.backproject_mode:
                    cv2.imshow( "camshift", self.frame )
                else:
                    cv2.imshow( "camshift", self.backproject)
                if self.disp_hist:
                    cv2.imshow( "Histogram", self.hue_histogram_as_image(self.hist))
            c = cv2.waitKey(7) & 0x0FF
            if c == 27 or c == ord("q"):
                rospy.signal_shutdown("OpenCV said so")
            elif c == ord("p"):
                self.pause = not self.pause
            elif c == ord("h"):
                self.disp_hist = not self.disp_hist
                if not self.disp_hist:
                    cv2.destroyWindow("Histogram")
            elif c == ord("b"):
                self.backproject_mode = not self.backproject_mode
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0
    print __doc__
    App(video_src).run()




