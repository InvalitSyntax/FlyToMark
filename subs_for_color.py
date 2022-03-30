import rospy
from aruco_pose.msg import MarkerArray
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Range
from std_msgs.msg import Float64

import time
import threading

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink
import os

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)
send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

class ImageDebug():
    def __init__(self):
        self.color = 'red'
        self.red_fix_range = [(160, 110, 150), (180, 255, 255)]
        self.colors = {
            'red': [(0, 150, 150), (10, 255, 255)],
        }

        self.image_pub_debug = rospy.Publisher('~debug', Image, queue_size=1)
        self.image_pub_HSV = rospy.Publisher('HSV', Image, queue_size=1)
        self.range = None

        self.frame_id = 'body'
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_timeout = rospy.Duration(0.01)  # таймаут ожидания трансформации
        self.pose = PoseStamped()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, queue_size=1)
        self.poseOfArucoInBody = None


    def range_callback(self, msg):
        self.range = msg.range

    def image_callback(self, msg):
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        #self.image_pub_debug.publish(bridge.cv2_to_imgmsg(frameNow, 'bgr8'))
        self.find_mark_pub(frame)
        self.transform()
        print()

    def find_mark_pub(self, frame):
        if self.range is not None:
            hsv_range = self.colors[self.color]
            
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_frame[..., 2] = 255
            #self.image_pub_HSV.publish(bridge.cv2_to_imgmsg(cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR), 'bgr8'))

            mask = cv2.inRange(hsv_frame, hsv_range[0], hsv_range[1])
            if self.color == 'red':
                mask2 = cv2.inRange(hsv_frame, self.red_fix_range[0], self.red_fix_range[1]) #new mask at diffrent diap
                mask = cv2.addWeighted(mask, 1, mask2, 1, 0.0) #combine 2 img

            conturs, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #conturs of all objects

            if len(conturs) != 0:
                cnt = max(conturs, key=cv2.contourArea)

                #centers
                M = cv2.moments(cnt)
                xc_mark = int(M['m10'] / M['m00'])
                yc_mark = int(M['m01'] / M['m00'])

                self.img = cv2.circle(frame, (xc_mark, yc_mark), 5, (255, 0, 0), -1)

                #center of image
                xc_frame = 160
                yc_frame = 120

                #dist at pix to mark at X and Y axis
                xDist, yDist = xc_mark - xc_frame, yc_frame - yc_mark
                #print(f'x: {xDist}\ny: {yDist}')
                mxDist, myDist = xDist * 0.007 * self.range, yDist * 0.007 * self.range
                self.mDist = (mxDist, -myDist)
            else:
                self.mDist = None

            #print(f'{self.mDist}')
            cv2.circle(frame, (160, 120), 3, (0, 255, 255), -1)
            cv2.putText(frame, f'range {round(self.range, 3)}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
            self.image_pub_debug.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

    def transform(self):
        if self.mDist is not None:
            self.pose.header.frame_id = 'main_camera_optical'
            self.pose.pose.position.x = self.mDist[0]
            self.pose.pose.position.y = self.mDist[1]
            self.pose.pose.position.z = self.range + 0.0 # + высота пылесоса
            self.pose.pose.orientation.w = 1
            self.pose.header.stamp = rospy.get_rostime()

            new_pose = self.tf_buffer.transform(self.pose, self.frame_id, self.transform_timeout)
            self.poseOfArucoInBody = new_pose.pose.position
        else:
            self.poseOfArucoInBody = None
        print(self.poseOfArucoInBody)

imgDebug = ImageDebug()
sub1 = rospy.Subscriber('main_camera/image_raw_throttled', Image, imgDebug.image_callback, queue_size=1)
sub2 = rospy.Subscriber('rangefinder/range', Range, imgDebug.range_callback, queue_size=1)

rospy.spin()
