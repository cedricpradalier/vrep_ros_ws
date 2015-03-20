#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_slam')

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import message_filters
import numpy
from numpy import mat
from numpy.linalg import inv
from math import atan2, hypot, pi, cos, sin, fmod

def norm_angle(x):
    return fmod(x+pi,2*pi)-pi


class BubbleSLAM:
    def __init__(self):
        rospy.init_node('bubble_slam')
        rospy.loginfo("Starting bubble rob slam")
        self.ignore_id = rospy.get_param("~ignore_id",self.ignore_id)
        self.target_frame = rospy.get_param("~target_frame",self.target_frame)
        self.body_frame = rospy.get_param("~body_frame",self.body_frame)
        self.odom_frame = rospy.get_param("~odom_frame",self.odom_frame)
        self.ar_precision = rospy.get_param("~ar_precision",self.ar_precision)
        self.position_uncertainty = rospy.get_param("~position_uncertainty",self.position_uncertainty)
        self.angular_uncertainty = rospy.get_param("~angular_uncertainty",self.angular_uncertainty)
        self.initial_x = rospy.get_param("~initial_x",self.initial_x)
        self.initial_y = rospy.get_param("~initial_y",self.initial_y)
        self.initial_theta = rospy.get_param("~initial_theta",self.initial_theta)
        # instantiate the right filter based on launch parameters
        initial_pose = [self.initial_x, self.initial_y, self.initial_theta]
        initial_uncertainty = [0.01, 0.01, 0.01]

        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose",PoseStamped,queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

        rospy.sleep(1.0);
        now = rospy.Time.now()
        self.listener.waitForTransform(self.odom_frame,self.body_frame, now, rospy.Duration(1.0))
        (trans,rot) = listener.lookupTransform(self.odom_frame,self.body_frame, now)
        self.old_odom = listener.fromTranslationRotation(trans,rot)



    def predict(self, Delta):
        
        # Implement Kalman prediction here
        theta = self.X[2,0]
        pose_mat = mat([[cos(theta), -sin(theta), 0, self.X[0,0]], 
                      [sin(theta),  cos(theta), 0, self.X[1,0]],
                      [         0,           0, 1, 0],
                      [         0,           0, 0, 1],
                      ]);
        pose_mat = Delta * mat;
        self.X[0:2,0] = pose_mat[0:2,3:4]
        euler_from_matrix(pose_mat[0:3,0:3], 'rxyz')
        self.X[2,0] = euler[2]; # Ignore the others
        Jx = mat([[1, 0, -sin(theta)*Delta[0,3]-cos(theta)*Delta[1,3]],
                  [0, 1,  cos(theta)*Delta[0,3]-sin(theta)*Delta[1,3]],
                  [0, 0,                       1                       ]])
        Js = Rtheta*iW
        Qs = mat(diag([self.position_uncertainty**2,self.position_uncertainty**2,self.angular_uncertainty**2]))
        P = self.P[0:3,0:3]
        self.P[0:3,0:3] = Jx * P * Jx.T + Qs 
        return (self.X,self.P)


    def update_ar(self, Z, id, uncertainty):
        # Z is a dictionary of id->vstack([x,y])
        print "Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id)
        (n,_) = self.X.shape
        R = mat(diag([uncertainty,uncertainty]))
        theta = self.X[2,0]
        Rtheta = self.getRotation(theta)
        Rmtheta = self.getRotation(-theta)
        H = mat(zeros((0, n)))
        if id in self.idx.keys():
            l = self.idx[id]
            H = mat(zeros((2,n)))
            H[0:2,0:2] = -Rmtheta
            H[0:2,2] = mat(vstack([-(self.X[l+0,0]-self.X[0,0])*sin(theta) + (self.X[l+1,0]-self.X[1,0])*cos(theta), \
                                   (self.X[l+0,0]-self.X[0,0])*cos(theta) - (self.X[l+1,0]-self.X[1,0])*sin(theta)]))
            H[0:2,l:l+2] = Rmtheta
            Zpred = Rmtheta * (self.X[l:l+2,0] - self.X[0:2,0])
            S = H * self.P * H.T + R
            K = self.P * H.T * inv(S)
            self.X = self.X + K * (Z - Zpred)
            self.P = (mat(eye(n)) - K * H) * self.P
        else:
            self.idx[id] = n
            self.X = numpy.concatenate((self.X, self.X[0:2,0]+(Rtheta*Z)))
            Pnew = mat(diag([uncertainty]*(n+2)))
            Pnew[0:n,0:n] = self.P
            self.P = Pnew
        return (self.X,self.P)

    def ar_cb(self, markers):
        for m in markers.markers:
            if m.id > 32:
                continue
            self.listener.waitForTransform(self.body_frame,m.header.frame_id, m.header.stamp, rospy.Duration(1.0))
            m_pose = PointStamped()
            m_pose.header = m.header
            m_pose.point = m.pose.pose.position
            m_pose = self.listener.transformPoint(self.body_frame,m_pose)
            Z = vstack([m_pose.point.x,m_pose.point.y])
            self.lock.acquire()
            if self.ignore_id:
                self.update_ar(Z,0,self.ar_precision)
            else:
                self.update_ar(Z,m.id,self.ar_precision)
            self.lock.release()


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.listener.waitForTransform(self.odom_frame,self.body_frame, now, rospy.Duration(1.0))
            (trans,rot) = listener.lookupTransform(self.odom_frame,self.body_frame, now)
            new_odom = listener.fromTranslationRotation(trans,rot)
            odom = new_odom * inv(self.old_odom)
            self.old_odom = new_odom
            self.lock.acquire()
            self.predict(odom)
            theta = self.X[2,0]
            pose_mat = mat([[cos(theta), -sin(theta), 0, self.X[0,0]], 
                          [sin(theta),  cos(theta), 0, self.X[1,0]],
                          [         0,           0, 1, 0],
                          [         0,           0, 0, 1],
                          ]);
            correction_mat = inv(new_odom) * pose_mat
            self.lock.release()
            scale, shear, angles, trans, persp = decompose_matrix(correction_mat)
            self.broadcaster.sendTransform(trans,angles,now, self.odom_frame,self.target_frame)
            rate.sleep()



if __name__=="__main__":
    demo = BubbleSLAM()
    demo.run()
