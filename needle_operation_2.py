# author: Muchen Li
from crtk_package_based_control import *
import crtk
import psm_arm
import time
import rospy
from ambf_client import Client
import numpy
from PyKDL import Vector, Frame, Rotation

def set_servo_cp(state):
    '''  set the position of the arm by a state. The input 'state' is a list of information: x,y,z,roll,pitch,yaw '''
    x,y,z,r,p,yaw = state
    frame = Frame()

    frame.p = Vector(x,y,z)
    frame.M = Rotation.RPY(r, p, yaw)

    return frame

client = Client("crtk_client_node")
client.connect()
arm_name = 'psm2'
namespace = '/CRTK'
# arm = psm_arm.PSM(client, arm_name)


while not rospy.is_shutdown():
    valid_key = False
    key = None
    while not valid_key:
        key = input("Press: \n"
                    '1 - change x coordinate \n'
                    '2 - close the jaw \n'
                    '3 - pick up the needle and point it towards the entry\n')

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
            print('not implemented yet')
        
        if key == 2:
            print('not implemented yet')

        if key == 3:
            example = crtk_move_cp_example(namespace + '/' + arm_name )
            time.sleep(0.1)

            s1 = [-0.26,0.011,-1.14,2.42,1.02,1.57]
            s2 = [-0.268,-0.067,-1.179,2.42,1.164,1.57]

            s3 = [-0.4365,0.20149,-1.3246,2.423,1.1194,2.5558]
            s4 = [-0.6,0.167910,-1.358209,3.005672,0.716418,3.809596]
            example.servo_cp(set_servo_cp(s1))
            time.sleep(2)
            example.servo_cp(set_servo_cp(s2))
            time.sleep(5)
            example.servo_cp(set_servo_cp(s3))
            time.sleep(5)
            example.servo_cp(set_servo_cp(s4))
            time.sleep(5)
