from surgical_robotics_challenge.camera import *
from ambf_client import Client
from PyKDL import Frame, Rotation, Vector, Twist
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
import sys

c = Client('test_cam')
c.connect()
time.sleep(0.5)

cam = Camera(c, 'CamFrame')

T_WF = cam.get_T_c_w()
#T_w_c = cam.get_T_w_c()

s2 = [-0.268,-0.067,-1.179,2.42,1.164,1.57]

x,y,z,r,p,yaw = s2

p = Vector(x,y,z)
R_0 = Rotation.RPY(r, p, yaw)
R = R_0.GetQuaternion()

frame = Frame(R, p)

test_pos = T_WF.dot(frame)
print(test_pos.p)