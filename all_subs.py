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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

def distance(x1, y1, x2, y2):
    c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return c

class FindDist:
    def __init__(self):
        self.img = None
        self.markers = None
        self.range = None
        self.dataAruco = None
        self.msg = None
        self.poseOfArucoInBody = None
        self.ID = 90
        self.dist = float('nan')
        self.frame_id = 'body'
        self.image_pub = rospy.Publisher('/debug', Image, queue_size=1)
        self.dist_pub = rospy.Publisher('/dist', Float64, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_timeout = rospy.Duration(0.01)  # таймаут ожидания трансформации
        self.pose = PoseStamped()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, queue_size=1, buff_size=1)

    def range_callback(self, msg):
        self.range = msg.range
        
    def image_callback(self, msg):
        self.img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def markers_callback(self, msg):
        self.markers = msg.markers
        self.msg = msg
        s = self.transform()
        s = self.calculate_dist_and_pub()
    
    def calculate_dist_and_pub(self):
        if self.img is not None and self.markers is not None:
            imgCopy = self.img.copy()
            dataAruco = list(filter(lambda x: x.id == self.ID, self.markers))
            cv2.circle(imgCopy, (160, 120), 3, (0, 255, 255), -1)
            if len(dataAruco) != 0:
                self.dataAruco = dataAruco

                dataAruco = dataAruco[0]
                poseAruco = dataAruco.pose.position

                cs = [dataAruco.c1, dataAruco.c2, dataAruco.c3, dataAruco.c4]
                for i in range(0, len(cs)):
                    cv2.circle(imgCopy, (int(cs[i].x), int(cs[i].y)), 2, (255, 0, 255), -1)

                cs = sorted(cs, key=lambda b: b.x)
                xCent = cs[0].x + (cs[-1].x - cs[0].x) / 2
                cs = sorted(cs, key=lambda b: b.y)
                yCent = cs[0].y + (cs[-1].y - cs[0].y) / 2

                cv2.circle(imgCopy, (int(xCent), int(yCent)), 5, (0, 0, 255), -1)

                self.dist = distance(0, 0, poseAruco.x, poseAruco.y)

                #distX = (poseAruco.y + 0.07)
                #distY = (poseAruco.x)
                #print(f'x= {round(distX, 5)}\ny= {round(distY, 5)}\n')
                #print(self.dist)
                cv2.putText(imgCopy, f'dist {self.dist}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                self.image_pub.publish(bridge.cv2_to_imgmsg(imgCopy, 'bgr8'))
            else:
                self.dist = float('nan')
                self.dataAruco = None
                self.image_pub.publish(bridge.cv2_to_imgmsg(imgCopy, 'bgr8'))
            self.dist_pub.publish(self.dist)

    def transform(self):
        if self.markers != None:
            dataAruco = list(filter(lambda x: x.id == self.ID, self.markers))
            if len(dataAruco) != 0:
                dataAruco = dataAruco[0]

                self.pose.header.frame_id = 'main_camera_optical'
                self.pose.pose.position = dataAruco.pose.position
                self.pose.pose.orientation.w = dataAruco.pose.orientation.w
                self.pose.header.stamp = rospy.get_rostime()

                new_pose = self.tf_buffer.transform(self.pose, self.frame_id, self.transform_timeout)
                self.poseOfArucoInBody = new_pose.pose.position
                print(f'{self.poseOfArucoInBody.x}\n{self.poseOfArucoInBody.y}\n')
            else:
                self.poseOfArucoInBody = None
        

for_dist = FindDist()
rospy.Subscriber('main_camera/image_raw', Image, for_dist.image_callback, queue_size=1)
rospy.Subscriber('aruco_detect/markers', MarkerArray, for_dist.markers_callback, queue_size=1)
rospy.Subscriber('rangefinder/range', Range, for_dist.range_callback, queue_size=1)

while not rospy.is_shutdown():
    print(for_dist.poseOfArucoInBody)