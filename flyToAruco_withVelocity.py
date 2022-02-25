import rospy
from aruco_pose.msg import MarkerArray
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('fly_makr')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
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

P = 2

def followAruco(ID=90):
    # take msg
    msg = rospy.wait_for_message('/aruco_detect/markers', MarkerArray)

    # take only needble marker
    markers = msg.markers
    dataAruco = list(filter(lambda x: x.id == ID, markers))

    # fix floating height
    telemetry = get_telemetry(frame_id='aruco_map')
    zVelocity = 1.5 - telemetry.z

    if len(dataAruco) != 0:
        # pose of marker
        dataAruco = dataAruco[0]
        poseAruco = dataAruco.pose.position

        print(f'x: {poseAruco.x}')
        print(f'y: {poseAruco.y}')

        xVelocity = -(poseAruco.y / P)
        yVelocity = -(poseAruco.x / P)
        
        set_velocity(vx=xVelocity, vy=yVelocity, vz=zVelocity, frame_id='body')
    else:
        print('nan')
        set_velocity(vx=0, vy=0, vz=zVelocity, frame_id='body')

navigate_wait(z=1, frame_id='body', auto_arm=True)
navigate_wait(x=1, y=1, z=1.5, frame_id='aruco_map')

while not rospy.is_shutdown():
    followAruco()
    print()