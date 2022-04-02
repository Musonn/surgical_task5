from ast import Pass, expr_context
from ctypes.wintypes import MSG
from utils.attach_needle import attach_needle
from ambf_client import Client
import psm_arm
import scene
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import rospy
from PyKDL import Rotation, Vector,Frame
import time
from launch_crtk_interface import SceneManager, Options
import roboticstoolbox as rtb
from spatialmath import SE3
import posemath
import numpy as np

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

# global var
# get transformations from tail to tip. See README for details
r = 0.10253
T1 = Frame(Vector(0, -r, 0))
T2 = Frame(Vector(-r * 0.86602540378, -r * 0.5, 0)) # cos(pi/6) and sin(pi/6)
T3 = Frame(Rotation.RotZ(-3.14/2))
x = -0.02   # This is the adjustment obtained from trials and errors
z = x * 1.03 # tan(46.1 deg)
entry1_frame = Frame(Rotation(0.692862, 0, -0.72107, 0, 1, 0, 0.72107,0 , 0.692862), Vector(-0.0373985 + x, 0.441891, 0.750042 + z)) # data from key == 4
entry1_frame.M = entry1_frame.M.EulerZYX(-3.14/2, 3.14/2, 0)
exit1_frame = Frame(Rotation(0.692862, 0,     0.72107, 0, 1, 0, -0.72107, 0, 0.692862), Vector(0.0407746,     0.44189,    0.748603 )) # data from key == 4
# exit1_frame.M = exit1_frame.M.EulerZYX( 0, -3.14/2, 3.14/2)
exit1_frame.M = exit1_frame.M.RPY( 3.14/2, 0, 0)
CC = Frame(Rotation(0.692862, 0, -0.72107, 0, 1, 0, 0.72107,0 , 0.692862), Vector(0.002, 0.442, 0.791)) # see README. Same orientation as entry 1

# suture function
global suture_r
def suture_r(entry_frame, angle):
    circle_center = entry_frame * (T2 * T3).Inverse()
    # rotate around z. See README for coordinate
    circle_center.M.DoRotZ(float(angle))
    target_pose = circle_center * T1.Inverse()
    return T_w_b * target_pose

# trajectory planning function
def move_cp(T0,T1,s):
    tj = rtb.ctraj(T0, T1, s)
    for _SE3 in tj:
        _array = _SE3.A
        _frame = posemath.fromMatrix(_array)
        set_servo_cp_2(_frame)
        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(0.05)

def move_cp2(F0,F1,s):
    # input frame MUST be under base frame
    T0 = SE3(posemath.toMatrix(F0), check=False)
    T1 = SE3(posemath.toMatrix(F1), check=False)
    T0 = T0.norm()
    T1 = T1.norm()
    tj = rtb.ctraj(T0, T1, s)
    for _SE3 in tj:
        _array = _SE3.A
        _frame = posemath.fromMatrix(_array)
        set_servo_cp_2(_frame)
        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(0.05)

# get current cartesian frame
def get_current_cp_frame(robData):
    data = robData.measured_cp.transform
    x = data.translation.x
    y = data.translation.y
    z = data.translation.z
    v = Vector(x,y,z)

    x,y,z,w = data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w
    r = Rotation.Quaternion(x,y,z,w)
    return Frame(r,v)


while not rospy.is_shutdown():
    valid_key = False
    key = None
    while not valid_key:
        key = input("Press: \n"
                    '1 - change x coordinate \n'
                    '2 - test ... \n'
                    '3 - pick up the needle and point it towards the entry \n'
                    '4 - print locations of entries and exits in world frame \n'
                    '5 - move in random trajectory \n')

        try:
            key = int(key)
        except ValueError:
            key = None
        pass

        if key in [1, 2, 3, 4, 5]:
            valid_key = True
        else:
            print("Invalid Entry")
        if key == 1:
            x = float(input('type the x value you want \n'))
            print('changed x \n')
            y = float(input('type the y value you want \n'))
            print('changed y \n')
            z = float(input('type the z value you want \n'))
            print('changed z \n')
            temp = Frame(Vector(x,y,z))
            temp = T_w_b * temp
            servo_cp_msg.transform.translation.x = temp.p.x()
            servo_cp_msg.transform.translation.y = temp.p.y()
            servo_cp_msg.transform.translation.z = temp.p.z()

        if key == 2:
                another_key = int(input('1 - ... move to exit 1 w/ needle \n'
                                    '2 - ... move to above needle tip \n'
                                    '3 - ... rotate at the entry 1 ONLY \n'
                                    '4 - ... rotate at the exit 1 ONLY \n'
                                    '5 - ... rotate around the center ONLY \n'
                                    '6 - ... move_cp(SE3) \n'
                                    '7 - ... move_cp2(frame)' ))

                if another_key == 1:
                    target_pose = exit1_frame * (T1 * T2 * T3).Inverse()
                    target_pose = T_w_b * target_pose

                if another_key == 2:
                    above_needle_center = Frame(Rotation(0.99955, 0.000893099,   0.0299795, 1.6103e-05,     0.99954,  -0.0303135, -0.0299928,   0.0303003,    0.999091),
                                                Vector(-0.207883,     0.56198,    0.711725)) # data from HUiyun_script.py
                    T_center_tail = Frame(Rotation.RPY(0, -3.14, 0.5 * 3.14), Vector(-0.10253, 0.03, 0.05)) # data from HUiyun_script.py
                    target_pose = above_needle_center * T_center_tail

                    target_pose = target_pose * T1 *T2
                    target_pose.p += Vector(0, 0, -0.09)
                    target_pose = T_w_b * target_pose
                if another_key == 3:
                    circle_center = entry1_frame * (T2 * T3).Inverse()
                    # rotate by deg around z
                    circle_center.M.DoRotZ(float(input()))
                    target_pose = circle_center * T1.Inverse()
                    target_pose = T_w_b * target_pose
                if another_key == 4:
                    circle_center = exit1_frame * (T2 * T3).Inverse()
                    # rotate by deg around z
                    circle_center.M.DoRotZ(float(input()))
                    target_pose = circle_center * T1.Inverse()
                    target_pose = T_w_b * target_pose
                if another_key == 5:
                    T4 = Frame(Vector(-0.037, 0.442, 0.750)-Vector(0.002, 0.442, 0.791))
                    # rotate by deg around z
                    CC.M.DoRotZ(float(input()))
                    target_pose =CC * T4  * (T1 * T2 * T3).Inverse()
                    target_pose = T_w_b * target_pose
                if another_key ==  6:
                    pose1 = SE3(-0.268,-0.067,-1.179)
                    pose2 = SE3(-0.4365,0.20149,-1.3246)
                    move_cp(pose1, pose2, 300)
                if another_key == 7:
                    F0 = entry1_frame
                    F1 = CC
                    print(SE3.isvalid(posemath.toMatrix(F0)), SE3.isvalid(posemath.toMatrix(F1)))
                    try:
                        T0 = SE3(posemath.toMatrix(F0), check=False)
                        T1 = SE3(posemath.toMatrix(F1), check=False)
                        T1 = T1.norm()
                        print(T1)
                    except ValueError:
                        print(T0.det(),np.linalg.det(posemath.toMatrix(F1)))
                    tj = rtb.ctraj(T0, T1, 30)
                    for _SE3 in tj:
                        _array = _SE3.A
                        _frame = posemath.fromMatrix(_array)
                        print(_frame)
                        set_servo_cp_2(T_w_b * _frame)
                        servo_cp_pub.publish(servo_cp_msg)
                        time.sleep(1)

                if another_key == 8:
                    s3 = [-0.4365,0.20149,-1.3246,2.423,1.1194,2.5558]
                    set_servo_cp(s3)
                    time.sleep(5)
                    print("measured_cp: ", robData.measured_cp.transform)

                # move the arm to it
                try: set_servo_cp_2(target_pose)
                except NameError: Pass
                servo_cp_pub.publish(servo_cp_msg)

        if key == 3:
            # grasp needle and point towards the entry
            # See README for details about transformations

            s0 = get_current_cp_frame(robData)

            '''move to above needle tail'''
            above_needle_center = Frame(Rotation(0.99955, 0.000893099,   0.0299795, 1.6103e-05,     0.99954,  -0.0303135, -0.0299928,   0.0303003,    0.999091),
                                        Vector(-0.207883,     0.56198,    0.711725)) # data from HUiyun_script.py
            T_center_tail = Frame(Rotation.RPY(0, -3.14, 0.5 * 3.14), Vector(-0.10253, 0.03, 0.05)) # data from HUiyun_script.py
            above_needle_tail = above_needle_center * T_center_tail
            s1 = T_w_b * above_needle_tail  # FIXME why s1 is not the matrix belongs to SE(3).

            '''pick up the needle'''
            needle_tail = above_needle_tail
            needle_tail.p += Vector(0, 0, -0.09) # move down by 0.09
            s2 = T_w_b * needle_tail
            
            transition_frame = Frame(Vector(-0.4555263280341801, 0.20142959155286047, -1.323356444777203))
            transition_frame.M = Rotation.Quaternion(0.15390270337209705, 0.8288184856382974, 0.011714044065109085, 0.5378072674579993)
            s3 = transition_frame

            '''go to the entry 1'''
            # entry 1 position from key == 4: pose of entry 1
            target_pose = entry1_frame * (T1 * T2 * T3).Inverse()
            s4 = T_w_b * target_pose
            
            servo_jaw_pub.publish(servo_jaw_angle_open)
            time.sleep(3)
            move_cp2(s0, s1, 100)
            time.sleep(3)
            move_cp2(s1, s2, 100)
            time.sleep(3)
            servo_jaw_pub.publish(servo_jaw_angle_closed)
            time.sleep(3)
            move_cp2(s2, s3, 100)
            time.sleep(3)
            move_cp2(s3, s4, 100)
            time.sleep(3)
            
            for i in range(3, 10, 1):
                s = suture_r(entry1_frame, i/10) # angle MUST from 0.3 to 1
                set_servo_cp_2(s)
                servo_cp_pub.publish(servo_cp_msg)
                time.sleep(0.3)
            for i in range(-5, 2, 1):
                s = suture_r(exit1_frame, i/10) # from -0.5 to 0.2
                set_servo_cp_2(s)
                servo_cp_pub.publish(servo_cp_msg)
                time.sleep(0.3)

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

        if key == 5:
            for i in range(100):
                vertice1 = [0.35, 1.0126, 1]
                vertice2 = [-0.35,1.0126, 1]
                vertice3 = [-0.35, -0.7174, 1]
                vertice4 = [0.35, -0.7174, 1]

                roll, pitch, yaw = np.random.uniform(-1,1), np.random.uniform(-1,1), np.random.uniform(-1,1)
                x2 = np.random.uniform(-0.34, 0.34)
                y2 = np.random.uniform(-0.6174, 0.9)
                z2 = np.random.uniform(1, 1.1)

                s0 = get_current_cp_frame(robData)
                pose2 = T_w_b * Frame(Rotation.RPY(roll, pitch, yaw), Vector(x2, y2, z2))
                servo_jaw_angle = JointState()
                servo_jaw_angle.position = [np.random.choice([0.1, 0.5]), 0, 0, 0, 0, 0]
                servo_jaw_pub.publish(servo_jaw_angle)
                print(s0)
                move_cp2(s0, pose2, 10)
                time.sleep(10)


        servo_cp_pub.publish(servo_cp_msg)
        time.sleep(0.1)

# world coordinate position
# [0.559 0.179 -1.168 3.14 -1.5 1.57]

# entry 1
# [[    0.692862, 0,    -0.72107;
#  0,           1, 0;
#       0.72107,0 ,    0.692862]
# [  -0.0373985,    0.441891,    0.750042]]

# pose of exit 1
# [[    0.692862, 0,     0.72107;
#   0,           1, 0;
#      -0.72107, 0,    0.692862]
# [   0.0407746,     0.44189,    0.748603]]
