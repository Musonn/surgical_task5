import attach_needle
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM

'''
copy from attach_needle
'''
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

servo_jp_msg = JointState()
servo_jp_msg.position = [0., 0., 1.0, 0., 0., 0.]

servo_cp_msg = TransformStamped()
servo_cp_msg.transform.translation.z = -1.0
R_7_0 = Rotation.RPY(3.14, 0.0, 1.57079)

servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

c = Client('attach_needle')
c.connect()
time.sleep(0.5)

needle = c.get_obj_handle('Needle')
link1 = c.get_obj_handle('psm1' + '/toolyawlink')
link2 = c.get_obj_handle('psm2' + '/toolyawlink')
link3 = c.get_obj_handle('psm3' + '/toolyawlink')
attach_needle.attach_needle(needle, link1)

test = PSM(c, 'test')
test.run_grasp_logic(0.10)
