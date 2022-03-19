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
rospy.init_node('computer_vision_sample')
bridge = CvBridge()

def distance(x1, y1, x2, y2):
    c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return c

class FindDist:
    def __init__(self):
        self.img = None
        self.markers = None
        self.image_pub = rospy.Publisher('/debug', Image, queue_size=10)
        self.dist_pub = rospy.Publisher('/dist', Float64, queue_size=10)
        self.ID = 35
        self.dist = float('nan')

    def image_callback(self, msg):
        self.img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def markers_callback(self, msg):
        self.markers = msg.markers
        self.calculate_dist_and_pub()
    
    def calculate_dist_and_pub(self):
        if self.img is not None and self.markers is not None:
            imgCopy = self.img.copy()
            dataAruco = list(filter(lambda x: x.id == self.ID, self.markers))
            cv2.circle(imgCopy, (160, 120), 3, (0, 255, 255), -1)
            if len(dataAruco) != 0:
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
                print(self.dist)
                cv2.putText(imgCopy, f'dist {self.dist}', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                self.image_pub.publish(bridge.cv2_to_imgmsg(imgCopy, 'bgr8'))
            else:
                self.dist = float('nan')
                self.image_pub.publish(bridge.cv2_to_imgmsg(imgCopy, 'bgr8'))
            self.dist_pub.publish(self.dist)


for_dist = FindDist()
rospy.Subscriber('main_camera/image_raw', Image, for_dist.image_callback)
rospy.Subscriber('aruco_detect/markers', MarkerArray, for_dist.markers_callback)

rospy.spin()