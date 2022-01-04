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
        self.measured_jaw_jp = JointState()


robData = RobotData()


def measured_js_cb(msg):
    robData.measured_js = msg


def measured_cp_cb(msg):
    robData.measured_cp = msg


def measured_jaw_jp(msg):
    robData.measured_jaw_jp = msg


rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm1"
measured_js_name = namespace + arm_name + "/measured_js"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_jaw_name = namespace + arm_name + '/jaw/servo_jp'
servo_jp_name = namespace + arm_name + "/servo_jp"
servo_cp_name = namespace + arm_name + "/servo_cp"

measured_js_sub = rospy.Subscriber(
    measured_js_name, JointState, measured_js_cb, queue_size=1)
measured_cp_sub = rospy.Subscriber(
    measured_cp_name, TransformStamped, measured_cp_cb, queue_size=1)

servo_jp_pub = rospy.Publisher(servo_jp_name, JointState, queue_size=1)
servo_cp_pub = rospy.Publisher(servo_cp_name, TransformStamped, queue_size=1)
servo_jaw_pub = rospy.Publisher(servo_jaw_name, JointState, queue_size=1)

rate = rospy.Rate(50)

'''don't change above'''

# set a jaw angle
servo_jaw_angle_closed = JointState()
servo_jaw_angle_closed.position = [0.1, 0, 0, 0, 0, 0]
servo_jaw_angle_open = JointState()
servo_jaw_angle_open.position = [0.5, 0, 0, 0, 0, 0]

servo_cp_msg = TransformStamped()
servo_cp_msg.transform.translation.x = 0.27
servo_cp_msg.transform.translation.y = -0.168
servo_cp_msg.transform.translation.z = -1.35

R_7_0 = Rotation.RPY(3.14, 2.16, 0.675)

servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]


while not rospy.is_shutdown():
    valid_key = False
    key = None
    while not valid_key:
        key = input("Press: \n"
                    '1 - change x coordinate \n'
                    '2 - close the jaw \n')

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
            servo_cp_msg.transform.translation.x = float(
                input('type the x value you want \n'))
            print('changed x \n')
            servo_cp_msg.transform.translation.y = float(
                input('type the y value you want \n'))
            print('changed y \n')
            servo_cp_msg.transform.translation.z = float(
                input('type the z value you want \n'))
            print('changed z \n')

        if key == 2:
            # use attach_needle.py
            servo_jaw_pub.publish(servo_jaw_angle_open)
            c = Client('attach_needle')
            c.connect()
            # create psm arm
            # arm1 = psm_arm.PSM(c, 'arm1')
            needle = c.get_obj_handle('Needle')
            link1 = c.get_obj_handle('psm1' + '/toolyawlink')
            attach_needle(needle, link1)
            # arm1.run_grasp_logic(0.1)
            time.sleep(0.5)
            servo_jaw_pub.publish(servo_jaw_angle_closed)
            c.clean_up()

        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(0.1)

# world coordinate position
# [0.559 0.179 -1.168 3.14 -1.5 1.57]
