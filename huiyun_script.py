from surgical_robotics_challenge.kinematics.psmIK import *
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
from surgical_robotics_challenge.scene import Scene
from PyKDL import Frame, Rotation, Vector, Twist
import numpy as np


class PSMController:
    def __init__(self, arm, scene):
        self.counter = 0
        self.arm = arm
        self.xyz = [0, 0.0, -1.0]
        self.rpy = [0, 0, 0]
        self.step_go_above_needle = False
        self.scene = scene

    def update_arm_pose(self):
        T_t_b = Frame(
            Rotation.RPY(self.rpy[0], self.rpy[1], self.rpy[2]),
            Vector(self.xyz[0], self.xyz[1], self.xyz[2]),
        )
        # print(T_t_b)
        # print(1)
        self.arm.servo_cp(T_t_b)
        # print(1)
        # self.arm.set_jaw_angle(gui.gr)
        # self.arm.run_grasp_logic(gui.gr)

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            gui = self.GUI
            T_ik_w = self.arm.get_T_b_w() * Frame(
                Rotation.RPY(self.rpy[0], self.rpy[1], self.rpy[2]),
                Vector(self.xyz[0], self.xyz[1], self.xyz[2]),
            )
            self.arm.target_IK.set_pos(T_ik_w.p[0], T_ik_w.p[1], T_ik_w.p[2])
            self.arm.target_IK.set_rpy(
                T_ik_w.M.GetRPY()[0], T_ik_w.M.GetRPY()[1], T_ik_w.M.GetRPY()[2]
            )
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.arm.target_FK.set_rpy(
                T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2]
            )

    def run(self):
        # self.xyz = needle_xyz
        if self.step_go_above_needle == False:
            self.go_above_needle()

            # self.step_go_above_needle = True
        # self.go_down_to_grab()

    def go_above_needle(self):
        # read the needle pose
        desired_pose = self.scene.needle_measured_cp()

        # step 1
        # modify the target position to be above the needle
        # desired_pose.p = Vector(
        #     desired_pose.p.x()-0.1053,
        #     desired_pose.p.y(),
        #     desired_pose.p.z()+0.05,
        # )
        print("init pose:", desired_pose.M)
        desired_pose.p += desired_pose.M * Vector(-0.1053, 0.03, 0.05)

        desired_pose.M = Rotation.RPY(0, -np.pi, 0.5 * np.pi) * desired_pose.M

        # Frame transformation
        target_pose = psm.get_T_w_b() * desired_pose
        print('This is what I want', psm.get_T_w_b().M, psm.get_T_w_b().p)

        self.move_arm_to(target_pose, 0.7)
        print("p1:", desired_pose.p)
        print("t1:", target_pose.p)
        print("actuators1:", self.arm.actuators)
        print("grasped1", self.arm.grasped)

        # step 2
        desired_pose.p -= Vector(
            0,
            0,
            0.09,
        )

        target_pose = psm.get_T_w_b() * desired_pose
        self.move_arm_to(target_pose, 0)
        print("p2:", desired_pose.p)
        print("t2:", target_pose.p)
        print("actuators2:", self.arm.actuators)
        print("grasped2:", self.arm.grasped)

        # step 3

        desired_pose.p += Vector(
            0,
            0,
            0.09,
        )

        target_pose = psm.get_T_w_b() * desired_pose
        self.move_arm_to(target_pose, 0)
        print("p3:", desired_pose.p)
        print("t3:", target_pose.p)
        print("actuators3:", self.arm.actuators)
        print("grasped3:", self.arm.grasped)

        # step 4

        desired_pose.p += Vector(
            0.05,
            0,
            0,
        )
        desired_pose.M = Rotation.RPY(0, 0, 0.5 * np.pi) * desired_pose.M

        target_pose = psm.get_T_w_b() * desired_pose
        self.move_arm_to(target_pose, 0)
        print("p4:", desired_pose.p)
        print("t4:", target_pose.p)
        print("actuators4:", self.arm.actuators)
        print("grasped4:", self.arm.grasped)
        self.arm.set_jaw_angle(0.5)
        self.arm.run_grasp_logic(0.5)

    def move_arm_to(self, target_pose, angle):
        self.xyz = target_pose.p
        self.rpy = target_pose.M.GetRPY()
        self.arm.set_jaw_angle(angle)
        self.arm.run_grasp_logic(angle)
        self.update_arm_pose()
        self.update_visual_markers()
        time.sleep(5)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument(
        "-c",
        action="store",
        dest="client_name",
        help="Client Name",
        default="ambf_client",
    )
    parser.add_argument(
        "--one", action="store", dest="run_psm_one", help="Control PSM1", default=False
    )
    parser.add_argument(
        "--two", action="store", dest="run_psm_two", help="Control PSM2", default=True
    )

    parsed_args = parser.parse_args()
    print("Specified Arguments")
    # print(parsed_args)
    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)

    # print(parsed_args.run_psm_one)
    c = Client(parsed_args.client_name)
    c.connect()
    time.sleep(0.5)
    controllers = []
    scene = Scene(c)

    if parsed_args.run_psm_two is True:
        arm_name = "psm2"
        psm = PSM(c, arm_name)
        if psm.base is not None:
            print("LOADING CONTROLLER FOR ", arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -1.0]
            init_rpy = [3.14, 0, 1.57079]

            controller = PSMController(psm, scene)
            controllers.append(controller)

    # Initialize the scene to subscribe through ambf

    # Start running
    if len(controllers) == 0:
        print("No Valid PSM Arms Specified")
        print("Exiting")

    else:
        while not rospy.is_shutdown():
            for cont in controllers:

                # Need to convert this to Robot frame

                # base_pose = psm.get_T_w_b().p

                # base_orientation = psm.get_T_w_b().M

                # target_orientation = psm.get_T_w_b().M * scene.needle_measured_cp().M

                # print(scene.needle_measured_cp().p[2])
                # print(type(scene.needle_measured_cp().p))

                # # target_pos = (
                # #     psm.get_T_w_b().M * scene.needle_measured_cp().p + psm.get_T_w_b().p
                # # )
                # target_pos = (psm.get_T_w_b() * scene.needle_measured_cp()).p

                cont.run()

            time.sleep(0.005)
