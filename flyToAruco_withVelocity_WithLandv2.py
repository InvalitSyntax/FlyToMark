import rospy
from aruco_pose.msg import MarkerArray
from mavros_msgs.srv import CommandBool
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Range

rospy.init_node('fly_makr')

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

zFlight = 1
lastPoseOfAruco = [0, 0]
ID = 10
VZ = -0.3

navigate_wait(z=1, frame_id='body', yaw=float('nan'), auto_arm=True)
navigate_wait(x=2, y=2, z=zFlight, yaw=math.pi / 2, frame_id='aruco_map')


def logicOfFlight():
    telemetry = get_telemetry(frame_id='aruco_map')
    xStart = telemetry.x
    yStart = telemetry.y

    while not rospy.is_shutdown():
        telemetry = get_telemetry(frame_id='aruco_map')

        msg = rospy.wait_for_message('/aruco_detect/markers', MarkerArray)
        # take only needble marker
        markers = msg.markers
        dataAruco = list(filter(lambda x: x.id == ID, markers))


        # if find mark:
        if len(dataAruco) != 0:
            # pose of marker
            dataAruco = dataAruco[0]
            poseAruco = dataAruco.pose.position

            #print()
            #print(f'x: {poseAruco.x}')
            #print(f'y: {poseAruco.y}')
            lastPoseOfAruco[:] = [(poseAruco.y + 0.07), (poseAruco.x)]

            # +0.07 - поправка на камеру (она не в центре дрона - на 7 сантиметра дальше по y)
            if telemetry.z < 1 and (abs(poseAruco.y) < 1 and abs(poseAruco.x) < 1):
                P = 1.2
            else:
                P = 0.9
            print(f'P = {P}')
            xVelocity = ((poseAruco.y + 0.07) * P)
            yVelocity = (poseAruco.x * P)

            if abs(poseAruco.y) < 0.2 and abs(poseAruco.x) < 0.2:
                set_velocity(vx=xVelocity, vy=yVelocity, vz=VZ, frame_id='body')

            else:
                set_velocity(vx=xVelocity, vy=yVelocity, vz=0, frame_id='body')

            xStart = telemetry.x
            yStart = telemetry.y
        
        # if don't find mark
        else:
            print('nan')

            # land
            if rospy.wait_for_message('rangefinder/range', Range).range < 0.3:
                set_velocity(vx=lastPoseOfAruco[0] * P, vy=lastPoseOfAruco[1] * P, vz=-10, frame_id='body')
                while rospy.wait_for_message('rangefinder/range', Range).range > 0.1:
                    pass
                return

            telemetry = get_telemetry(frame_id='aruco_map')
            if 0.5 < telemetry.x < 3.5 and 0.5 < telemetry.y < 3.5 and telemetry.z < 1.5 and abs(telemetry.x - xStart) < 1 and abs(telemetry.y - yStart) < 1:
                set_velocity(vx=0, vy=0, vz=0.2, yaw=float('nan'), frame_id='body')

            else:
                # поиск метки во всём поле
                set_position(x=xStart, y=yStart, z=1.8, yaw=math.pi / 2, frame_id='aruco_map')


logicOfFlight()
arming(False)

rospy.sleep(1)