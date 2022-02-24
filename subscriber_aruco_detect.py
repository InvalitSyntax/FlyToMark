import rospy
from aruco_pose.msg import MarkerArray


rospy.init_node('sub')

def markersCallback(msg):
    markers = msg.markers

    aruco90 = list(filter(lambda x: x.id == 90, markers))
    
    if len(aruco90) != 0:
        poseAruco90 = aruco90[0].pose.position

        print(f'x: {poseAruco90.x}')
        print(f'y: {poseAruco90.y}')

    print()


# name, type of massage, func
rospy.Subscriber('/aruco_detect/markers', MarkerArray, markersCallback)

rospy.spin()