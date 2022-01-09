# author: Muchen Li
import crtk
import psm_arm
import time
import rospy
from ambf_client import Client

class crtk_move_cp_example:
    def __init__(self, namespace):


        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_move_cp_example', anonymous = True, log_level = rospy.WARN)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, namespace)
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_move_cp()   # not implemented in launch_crtk_interface.py
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_servo_cp()

