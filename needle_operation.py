from utils.attach_needle import attach_needle
from ambf_client import Client
import psm_arm
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import rospy
from PyKDL import Rotation
import time
import keyboard


class RobotData:
    def __init__(self):
        self.measured_js = JointState()
        self.measured_cp = TransformStamped()


robData = RobotData()


def measured_js_cb(msg):
    robData.measured_js = msg


def measured_cp_cb(msg):
    robData.measured_cp = msg


rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm1"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"

measured_js_sub = rospy.Subscriber(
    measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(
    measured_cp_name, TransformStamped, measured_cp_cb, queue_size=1)

servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, TransformStamped, queue_size=1)

rate = rospy.Rate(50)

'''don't change above'''

servo_cp_msg = TransformStamped()
servo_cp_msg.transform.translation.x = 0.5
servo_cp_msg.transform.translation.y = 0.18
servo_cp_msg.transform.translation.z = -1.168

R_7_0 = Rotation.RPY(3.14, -1.5, 1.57079)

servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

# use attach_needle.py
c = Client('attach_needle')
c.connect()
# create psm arm
#arm1 = psm_arm.PSM(c, 'arm1')
needle = c.get_obj_handle('Needle')
link1 = c.get_obj_handle('psm1' + '/toolyawlink')

while not rospy.is_shutdown():
    valid_key = False
    key = None
    while not valid_key:
        key = input("Press: \n")

        try:
            key = int(key)
        except ValueError:
            key = None
        pass

        if key in [1, 2, 3]:
            valid_key = True
        else:
            print("Invalid Entry")
        if key == 1:
            servo_cp_msg.transform.translation.x = input()
            print('changed x')

        if key == 2:
            attach_needle(needle, link1)
            link1.run_grasp_logic(0.1)

        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(10)

# world coordinate position
# [0.559 0.179 -1.168 3.14 -1.5 1.57]

