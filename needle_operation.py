from utils.attach_needle import attach_needle
from ambf_client import Client
import psm_arm
import scene
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import rospy
from PyKDL import Rotation
import time
import keyboard
from launch_crtk_interface import SceneManager, Options

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

def set_servo_cp(state):
    '''  set the position of the arm by a state. The input 'state' is a list of information: x,y,z,roll,pitch,yaw '''
    x,y,z,r,p,yaw = state
    servo_cp_msg.transform.translation.x = x
    servo_cp_msg.transform.translation.y = y
    servo_cp_msg.transform.translation.z = z
    print(servo_cp_msg)

    R_0 = Rotation.RPY(r, p, yaw)
    servo_cp_msg.transform.rotation.x = R_0.GetQuaternion()[0]
    servo_cp_msg.transform.rotation.y = R_0.GetQuaternion()[1]
    servo_cp_msg.transform.rotation.z = R_0.GetQuaternion()[2]
    servo_cp_msg.transform.rotation.w = R_0.GetQuaternion()[3]
    return servo_cp_msg


rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm2"
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
servo_cp_msg.transform.translation.x = -0.26
servo_cp_msg.transform.translation.y = 0.011
servo_cp_msg.transform.translation.z = -1.14

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
                    '2 - close the jaw \n'
                    '3 - pick up the needle and point it towards the entry')

        try:
            key = int(key)
        except ValueError:
            key = None
        pass

        if key in [1, 2, 3, 4]:
            valid_key = True
        else:
            print("Invalid Entry")
        if key == 1:
            servo_cp_msg.transform.translation.x = float(input('type the x value you want \n'))
            print('changed x \n')
            servo_cp_msg.transform.translation.y = float(input('type the y value you want \n'))
            print('changed y \n')
            servo_cp_msg.transform.translation.z = float(input('type the z value you want \n'))
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
        
        if key == 3:
            # grasp needle and point towards the entry

            # here are the hard-code states
            s1 = [-0.26,0.011,-1.14,2.42,1.02,1.57]
            s2 = [-0.268,-0.067,-1.179,2.42,1.164,1.57]
            
            s3 = [-0.4365,0.20149,-1.3246,2.423,1.1194,2.5558]
            s4 = [-0.6,0.167910,-1.358209,3.005672,0.716418,3.809596]
            
            servo_jaw_pub.publish(servo_jaw_angle_open)
            time.sleep(1)
            servo_cp_msg = set_servo_cp(s1)
            servo_cp_pub.publish(servo_cp_msg)
            time.sleep(5)
            set_servo_cp(s2)
            servo_cp_pub.publish(servo_cp_msg)
            time.sleep(6)
            servo_jaw_pub.publish(servo_jaw_angle_closed)
            time.sleep(6)
            set_servo_cp(s3)
            servo_cp_pub.publish(servo_cp_msg)
            time.sleep(6)
            set_servo_cp(s4)
            servo_cp_pub.publish(servo_cp_msg)
            time.sleep(6)

            # get the position of any entry
            c = Client('get_scene')
            c.connect()
            sc_obj = scene.Scene(c)
            # print pose of entry 1
            print('pose of entry 1')
            print(sc_obj.entry1_measured_cp())
            # print pose of entry 2
            print('pose of entry 2')
            print(sc_obj.entry2_measured_cp())
            # print pose of entry 3
            print('pose of entry 3')
            print(sc_obj.entry3_measured_cp())
            # print pose of entry 4
            print('pose of entry 4')
            print(sc_obj.entry4_measured_cp())
            # print pose of exit 1
            print('pose of exit 1')
            print(sc_obj.exit1_measured_cp())
            # print pose of exit 2
            print('pose of exit 2')
            print(sc_obj.exit2_measured_cp())
            # print pose of exit 3
            print('pose of exit 3')
            print(sc_obj.exit3_measured_cp())
            # print pose of exit 4
            print('pose of exit 4')
            print(sc_obj.exit4_measured_cp())

            print('----------------------------------------------------------')
            # call publish_cs() in the scene manager
            options = Options()
            options.run_psm_one = False
            options.run_psm_two = False
            options.run_psm_three = False
            options.run_ecm = False
            options.run_scene = True
            options.rate = 120

            sceneManager = SceneManager(options)
            sceneManager.run()


        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(0.1)

# world coordinate position
# [0.559 0.179 -1.168 3.14 -1.5 1.57]

# [-0.26 0.011 -1.14 2.42 1.02 1.57]
# [-0.268 -0.067 -1.179 2.42 1.164 1.57]
# close gripper
# [-0.4365 0.20149 -1.3246 2.423 1.1194 2.5558]
# [-0.624478 0.167910 -1.358209 3.005672 0.716418 3.809596]


