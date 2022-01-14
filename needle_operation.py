from utils.attach_needle import attach_needle
from ambf_client import Client
import psm_arm
import scene
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import rospy
from PyKDL import Rotation, Vector, Frame
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

def set_servo_cp_2(frame):
    '''  set the position of the arm by a frame.'''
    x,y,z= frame.p
    servo_cp_msg.transform.translation.x = x
    servo_cp_msg.transform.translation.y = y
    servo_cp_msg.transform.translation.z = z

    R = frame.M
    servo_cp_msg.transform.rotation.x = R.GetQuaternion()[0]
    servo_cp_msg.transform.rotation.y = R.GetQuaternion()[1]
    servo_cp_msg.transform.rotation.z = R.GetQuaternion()[2]
    servo_cp_msg.transform.rotation.w = R.GetQuaternion()[3]
    return servo_cp_msg

def transform_to_frame(R, p):
    '''
    R is a one dimensional list representing Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz; p is a list of x, y, z
    see more at http://docs.ros.org/en/diamondback/api/kdl/html/python/geometric_primitives.html#PyKDL.Rotation
    '''
    Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz = R
    x,y,z = p
    frame = Frame(Rotation(Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz), Vector(x,y,z))

    return frame

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

# find transformations between world and base
c=Client('blah')
c.connect()
psm2 = psm_arm.PSM(c, 'psm2')
T_w_b = psm2.get_T_w_b()
c.clean_up()

while not rospy.is_shutdown():
    valid_key = False
    key = None
    while not valid_key:
        key = input("Press: \n"
                    '1 - change x coordinate \n'
                    '2 - supposely move to ... \n'
                    '3 - pick up the needle and point it towards the entry \n'
                    '4 - print locations of entries and exits in world frame \n')

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
                another_key = int(input('1 - ... entry 1 \n'
                                    '2 - ... above needle \n'))

                if another_key == 1:
                    # entry 1 position in blender
                    entry1_state = [-0.0748, 0.4419, 0.7601, 3.14, 0, 1.57]
                    x,y,z,r,p,yaw = entry1_state
                    # covert it from world to base
                    # state object to frame object
                    P = Vector(x,y,z)
                    R = Rotation.RPY(r,p,yaw)
                    entry1_frame = Frame(R, P)
                    target_pose = T_w_b * entry1_frame
                if another_key == 2:
                    above_needle_center = Frame(Rotation(0.99955, 0.000893099,   0.0299795, 1.6103e-05,     0.99954,  -0.0303135, -0.0299928,   0.0303003,    0.999091),
                                                Vector(-0.207883,     0.56198,    0.711725)) # data from HUiyun_script.py
                    target_pose = Frame( Rotation.RPY(0, -3.14, 0.5 * 3.14) * above_needle_center.M, above_needle_center.p + above_needle_center.M * Vector(-0.10253, 0.03, 0.05)) # expression from HUiyun_script.py
                    target_pose = T_w_b * target_pose
                    # target_pose.M = Rotation.RPY(2.42,1.02,1.57)    # data from s1

                # move the arm to it
                set_servo_cp_2(target_pose)
                servo_cp_pub.publish(servo_cp_msg)

        if key == 3:
            # grasp needle and point towards the entry

            # here are the hard-code states
            above_needle_center = Frame(Rotation(0.99955, 0.000893099,   0.0299795, 1.6103e-05,     0.99954,  -0.0303135, -0.0299928,   0.0303003,    0.999091),
                                        Vector(-0.207883,     0.56198,    0.711725)) # data from HUiyun_script.py
            above_needle_tail = Frame( Rotation.RPY(0, -3.14, 0.5 * 3.14) * above_needle_center.M, above_needle_center.p + above_needle_center.M * Vector(-0.10253, 0.03, 0.05)) # expression from HUiyun_script.py
            s1 = T_w_b * above_needle_tail

            needle_tail = above_needle_tail
            needle_tail.p += Vector(0,0, -0.09)
            s2 = T_w_b * needle_tail

            s3 = [-0.4365,0.20149,-1.3246,2.423,1.1194,2.5558]
            s4 = [-0.6,0.167910,-1.358209,3.005672,0.716418,3.809596]
            
            servo_jaw_pub.publish(servo_jaw_angle_open)
            time.sleep(1)
            servo_cp_msg = set_servo_cp_2(s1)
            servo_cp_pub.publish(servo_cp_msg)
            time.sleep(5)
            set_servo_cp_2(s2)
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

        if key == 4:
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

# position: [0.9388150572776794, -0.8624297976493835, -7.284080538738635e-07, -2.1081840991973877, -1.5711891651153564, 0.1948612481355667]
# velocity: [1.9089757188339718e-06, -1.957461608981248e-05, 6.901156552885368e-07, 1.0777608156204224, -4.375037315185182e-05, -1.391890525817871]
# effort: []
# ------------------------------------
# measured_cp:  translation: 
#   x: -0.17486074581098204
#   y: -0.2271024949728268
#   z: -0.09153860265431155
# rotation: 
#   x: -0.43954491763127124
#   y: 0.8729602253467024
#   z: -0.0192643795399843
#   w: -0.2106409125219813

# [[  0.00041677,    0.751203,    0.660072;
#      0.523366,    0.562289,    -0.64025;
#     -0.852108,    0.345726,   -0.392919]
# [   -0.366903, -0.00912285,     -1.2308]]


