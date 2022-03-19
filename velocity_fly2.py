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

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

class FindDist:
    def __init__(self):
        self.img = None
        self.markers = None
        self.range = None
        self.image_pub = rospy.Publisher('/debug', Image, queue_size=10)
        self.dist_pub = rospy.Publisher('/dist', Float64, queue_size=10)
        self.ID = 35
        self.dist = float('nan')

    def range_callback(self, msg):
        self.range = msg.range
        
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
rospy.Subscriber('rangefinder/range', Range, for_dist.range_callback)

navigate_wait(z=1, frame_id='body', yaw=float('nan'), auto_arm=True)
navigate_wait(x=3.2, y=1.2, z=0.5, yaw=math.pi / 2, frame_id='aruco_map')

zFlight = 1
lastPoseOfAruco = [0, 0]
ID = 35
VZ = 0

def logicOfFlight(datas):
    lastPoseOfAruco = [0, 0]
    P = 0.9

    while not rospy.is_shutdown():
        start = time.time()
        #telemetry = get_telemetry(frame_id='aruco_map')
        rangefinder_range = datas.range

        markers = datas.markers
        dataAruco = list(filter(lambda x: x.id == ID, markers))


        # if find mark:
        if len(dataAruco) != 0:
            # pose of marker
            dataAruco = dataAruco[0]
            poseAruco = dataAruco.pose.position

            #print()
            #print(f'x: {poseAruco.x}')
            #print(f'y: {poseAruco.y}')

            maxDistSnizeniya = 0.2
            distX = (poseAruco.y + 0.07)
            distY = (poseAruco.x)

            # когда дрон в кубе 2х2 возле метки увеличиваем P:
            if abs(distX) < 0.1 and abs(distY) < 0.1:
                P = 0.6
            elif abs(distX) < 0.6 and abs(distY) < 0.6:

                # если скорости не хватает что бы достаточно сблизится, постепенно увеличиваем:
                if abs(distX) > maxDistSnizeniya and abs(distY) > maxDistSnizeniya:
                    P += 0.1
                else:
                    # когда уже снижаемся, но всё же постепенно отстаём, тоже наращиваем скорость
                    dx = lastPoseOfAruco[0] - distX
                    dy = lastPoseOfAruco[1] - distY
                    if (dx < 0 or dy < 0) and (abs(distX) > 0.06 or abs(distY) > 0.06):
                        P += 0.1

                    # елси же отставание минимальное, уменьшаем скорость
                    else:
                        P = 1.2
            else:
                P = 0.9



            if P > 1.5:
                P = 1.5

            print(f'P = {P} {distX} {distY}')
            xVelocity = (distX * P)
            yVelocity = (distY * P)

            if abs(distX) < maxDistSnizeniya and abs(distY) < maxDistSnizeniya:
                set_velocity(vx=xVelocity, vy=yVelocity, vz=VZ, frame_id='body')

            else:
                set_velocity(vx=xVelocity, vy=yVelocity, vz=0, frame_id='body')

            lastPoseOfAruco = [distX, distY]
        
        # if don't find mark
        else:
            print('nan')

            # land
            if rangefinder_range < 0.3:
                set_velocity(vx=lastPoseOfAruco[0] * P, vy=lastPoseOfAruco[1] * P, vz=-10, frame_id='body')
                while rospy.wait_for_message('rangefinder/range', Range).range > 0.1:
                    pass
                return

            #telemetry = get_telemetry(frame_id='aruco_map')
            if rangefinder_range < 1.5:
                set_velocity(vx=0, vy=0, vz=0.2, yaw=float('nan'), frame_id='body')

            else:
                # поиск метки во всём поле
                set_position(x=float('nan'), y=float('nan'), z=1.8, yaw=math.pi / 2, frame_id='aruco_map')

            P = 0.9

        if rangefinder_range <= 0.1:
            return

        #rospy.sleep(0.1)
        dt = time.time() - start 
        print(f'dt= {dt}')


logicOfFlight(for_dist)
arming(False)

rospy.sleep(1)