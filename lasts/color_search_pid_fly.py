import rospy
from aruco_pose.msg import MarkerArray
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from clover.srv import SetLEDEffect

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
send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect) 

def reboot_fcu():
    send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    #os.system("systemctl restart clover")

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

def PID(SatUp, SatDwn, Kp, Ki, Kd, prev_err =0, dt = 0, err=0, Proportional=0, Differential=0, Integral=0):
    #start_time = time.time()
    Proportional = Kp*err
    if dt == 0:
        Differential = 0
    else:
        Differential = Kd*(err-prev_err)/dt
    #dt = time.time() - start_time
    Integral += err*dt*Ki
    if Integral > SatUp:
        Integral = SatUp
    else:
        if Integral < SatDwn:
           Integral = SatDwn
    Output = Proportional + Differential + Integral
    if Output > SatUp:
          Output = SatUp
    else:
        if Output < SatDwn:
           Output = SatDwn 
    return Output, Proportional, Integral, Differential

class FindDist:
    def __init__(self):
        self.color = 'red'
        self.red_fix_range = [(160, 110, 150), (180, 255, 255)]
        self.colors = {
            'red': [(0, 150, 150), (10, 255, 255)],
        }

        self.image_pub_debug = rospy.Publisher('~debug', Image, queue_size=1)
        self.image_pub_HSV = rospy.Publisher('HSV', Image, queue_size=1)

        self.range = None
        self.mDist = None

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
        #print(self.poseOfArucoInBody)

    def led(self):
        while not self.STOP:
            if self.poseOfArucoInBody is not None:
                set_effect(effect='flash', r=0, g=0, b=255)
            rospy.sleep(1)

reboot_fcu()
rospy.sleep(15)
print('rebooted')

for_dist = FindDist()
print(*for_dist.__dict__.items())
thread = threading.Thread(target=for_dist.led)
thread.start()
sub1 = rospy.Subscriber('main_camera/image_raw_throttled', Image, for_dist.image_callback, queue_size=1)
sub3 = rospy.Subscriber('rangefinder/range', Range, for_dist.range_callback, queue_size=1)

navigate_wait(z=1, frame_id='body', yaw=float('nan'), auto_arm=True)
navigate_wait(x=1, y=1.2, z=1, yaw=math.pi / 2, frame_id='aruco_map')


def logicOfFlight(datas):
    lastPoseOfAruco = [0, 0]
    prev_errx = 0
    prev_erry = 0
    dt = 0
    maxDistSnizeniya = 0.03
    minDistNivigate = 0.6
    lastPoseOfAruco = [0, 0]
    VZ = -0.3

    while not rospy.is_shutdown():
        start = time.time()
        rangefinder_range = datas.range
        poseAruco = datas.poseOfArucoInBody # here .x and .y (.pose.position of object)

        # if find mark:
        if poseAruco != None:
            # find, blue idicate
            
            distX = (poseAruco.x)
            distY = (poseAruco.y)

            refx = 0
            refy = 0
            feedbckx = distX
            feedbcky = distY
            errx = refx - feedbckx
            erry = refy - feedbcky

            p, i, d = 0.5, 45, 0.1
            sat = 2
            Outx, Px, Ix, Dx = PID(err = errx, SatUp =sat, SatDwn=-sat, Kp=p, Ki=i, Kd=d, prev_err=prev_errx, dt = dt)
            Outy, Py, Iy, Dy = PID(err = erry, SatUp =sat, SatDwn=-sat, Kp=p, Ki=i, Kd=d, prev_err=prev_erry, dt = dt)

            prev_errx = errx
            prev_erry = erry

            xVelocity = -Outx
            yVelocity = -Outy


            #print(f'{xVelocity}\n{yVelocity}\n')
            if rangefinder_range > 0.8 or abs(distX) > minDistNivigate or abs(distY) > minDistNivigate:
                print('NAVIGATE')
                navigate(frame_id='body', x=distX, y=distY, z=-0.2, yaw=float('nan'), speed=1)

            elif abs(distX) < maxDistSnizeniya and abs(distY) < maxDistSnizeniya:
                print('PID')
                set_velocity(vx=xVelocity, vy=yVelocity, vz=VZ, frame_id='body')

            else:
                print('PID')
                set_velocity(vx=xVelocity, vy=yVelocity, vz=0, frame_id='body')

            lastPoseOfAruco = [distX, distY, xVelocity, yVelocity]
            dt = time.time() - start

        # if don't find mark
        else:
            print('nan')

            # land
            if rangefinder_range < 0.3 and abs(lastPoseOfAruco[0]) < maxDistSnizeniya and abs(lastPoseOfAruco[1]) < maxDistSnizeniya:
                print(f'range: {rangefinder_range}')
                set_velocity(vx=lastPoseOfAruco[2], vy=lastPoseOfAruco[3], vz=-10, frame_id='body')
                while rospy.wait_for_message('rangefinder/range', Range).range > 0.1:
                    pass
                arming(False)
                return 'success'

            if rangefinder_range < 1.1:
                set_velocity(vx=0, vy=0, vz=0.2, yaw=float('nan'), frame_id='body')

            else:
                # поиск метки во всём поле
                return 'unsuccess'
                navigate(x=3.2, y=1.2, z=1.1, yaw=math.pi / 2, frame_id='aruco_map')

def findInZone():
    pointsForFligtToFind = [123]
    while not rospy.is_shutdown():
        for point in pointsForFligtToFind:
            navigate(x=0, y=0, z=1.1, yaw=math.pi / 2, frame_id=f'aruco_{point}')

            while not rospy.is_shutdown():
                if for_dist.poseOfArucoInBody is not None:
                    res = logicOfFlight(for_dist)
                    if res == 'success':
                        # landed, red indicate
                        set_effect(r=255, g=0, b=0)
                        return
                    else:
                        navigate(x=0, y=0, z=1.1, yaw=math.pi / 2, frame_id=f'aruco_{point}')
                        
                telem = get_telemetry(frame_id='navigate_target')
                if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.2:
                    break

findInZone()
print(f'range: {for_dist.range}')

rospy.sleep(1)
for_dist.STOP = True
thread.join()
sub1.unregister()
sub3.unregister()
rospy.sleep(1)