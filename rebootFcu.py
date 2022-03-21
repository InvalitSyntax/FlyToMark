import rospy
from mavros_msgs.srv import CommandLong
from pymavlink.dialects.v20 import common as mavlink
from clover import srv
import os

rospy.init_node('test_mavros_wrapper')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

def reboot_fcu():
    send_command_long(False, mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    os.system("reboot")

reboot_fcu()
rospy.sleep(0.1)
print(get_telemetry(frame_id='body'))
rospy.sleep(10)

print(get_telemetry(frame_id='body'))