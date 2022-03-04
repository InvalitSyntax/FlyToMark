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

P = 0.6
zFlight = 1
lastPoseOfAruco = [0, 0]

def followAruco(ID=10, VZ=-0.3):
    print()
    # take msg
    msg = rospy.wait_for_message('/aruco_detect/markers', MarkerArray)

    # take only needble marker
    markers = msg.markers
    dataAruco = list(filter(lambda x: x.id == ID, markers))

    # fix floating height
    telemetry = get_telemetry(frame_id='aruco_map')

    
    zVelocity = zFlight - telemetry.z


    if len(dataAruco) != 0:
        # pose of marker
        dataAruco = dataAruco[0]
        poseAruco = dataAruco.pose.position

        print(f'x: {poseAruco.x}')
        print(f'y: {poseAruco.y}')
        lastPoseOfAruco[:] = [poseAruco.y + 0.07, poseAruco.x]

        # +0.07 - поправка на камеру (она не в центре дрона - на 7 сантиметра дальше по y)
        xVelocity = ((poseAruco.y + 0.07) * P)
        yVelocity = (poseAruco.x * P)
        if poseAruco.y < 0.15 and poseAruco.x < 0.15:
            set_velocity(vx=xVelocity, vy=yVelocity, vz=VZ, frame_id='body')
        else:
            set_velocity(vx=xVelocity, vy=yVelocity, vz=0, frame_id='body')
    else:
        print('nan')
        #set_velocity(vx=0, vy=0, yaw=float('nan'), vz=zVelocity, frame_id='aruco_map')
        return 'lost'

navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(x=1, y=1, z=zFlight, frame_id='aruco_map')


def logicOfFlight():
    while not rospy.is_shutdown():
        telemetry = get_telemetry(frame_id='aruco_map')

        xStart = telemetry.x
        yStart = telemetry.y

        while followAruco() == 'lost':

            # land
            if rospy.wait_for_message('rangefinder/range', Range).range < 0.3:
                set_velocity(vx=lastPoseOfAruco[0], vy=lastPoseOfAruco[1], vz=-10, frame_id='body')
                while rospy.wait_for_message('rangefinder/range', Range).range > 0.15:
                    pass
                return



            telemetry = get_telemetry(frame_id='aruco_map')
            if 0 < telemetry.x < 4.2 and 0 < telemetry.y < 4.2 and telemetry.z < 1 and abs(telemetry.x - xStart) < 1 and abs(telemetry.y - yStart) < 1:
                set_velocity(vx=lastPoseOfAruco[0] / 4, vy=lastPoseOfAruco[1] / 4, vz=0.2, yaw=float('nan'), frame_id='body')

            else:
                # поиск метки во всём поле
                set_position(x=telemetry.x, y=telemetry.y, z=telemetry.z, yaw=float('nan'), frame_id='aruco_map')


logicOfFlight()
arming(False)

rospy.sleep()