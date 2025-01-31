import numpy as np
import dartpy as dart
import time
from simulation.dart.robot import Robot
from simulation.dart.kinematics import Kinematics
from ismpc import FrameInfo, Reference, State, FootstepPlan, RotationMatrix
from ismpc import (
    FootstepPlanProvider,
    ModelPredictiveController,
    FootTrajectoryGenerator,
    MovingConstraintProvider,
    KalmanFilter,
)
from scipy.spatial.transform import Rotation as R
from simulation.dart.misc import REDUNDANT_DOFS
from simulation.utils import config

from time import sleep
import sys


class Controller(dart.gui.osg.RealTimeWorldNode):

    def __init__(self, world: dart.simulation.World, robot: Robot):
        super(Controller, self).__init__(world)
        self.world = world
        self.robot = robot
        world.setTimeStep(config.delta)
        self.dt = world.getTimeStep()
        self.dart_elapsed = 0
        self.ismpc_elapsed = 0
        self.kin_elapsed = 0

        # Balls for COM and desired COM
        GREEN = np.array([0, 255, 0])
        RED = np.array([255, 0, 0])
        self.com_ball = self.create_ball("com", RED, 0.05)
        self.des_com_ball = self.create_ball("des_com", GREEN, 0.05)
        world.addSimpleFrame(self.com_ball)
        world.addSimpleFrame(self.des_com_ball)
        
        # Balls for ZMP and desired ZMP
        BLUE = np.array([0, 0, 255])
        YELLOW = np.array([255, 255, 0])
        self.zmp_ball = self.create_ball("zmp", BLUE, 0.05)
        self.des_zmp_ball = self.create_ball("des_zmp", YELLOW, 0.05)
        world.addSimpleFrame(self.zmp_ball)
        world.addSimpleFrame(self.des_zmp_ball)
        
        # Balls for the feet
        self.lsole_ball = self.create_ball("lsole", RED, 0.07)
        self.rsole_ball = self.create_ball("rsole", RED, 0.07)
        world.addSimpleFrame(self.lsole_ball)
        world.addSimpleFrame(self.rsole_ball)
        self.lsole_des_ball = self.create_ball("lsole_des", GREEN, 0.07)
        self.rsole_des_ball = self.create_ball("rsole_des", GREEN, 0.07)
        world.addSimpleFrame(self.lsole_des_ball)
        world.addSimpleFrame(self.rsole_des_ball)

        # Representations
        self.frame_info = FrameInfo()
        self.reference = Reference()
        self.state = State()
        self.plan = FootstepPlan()

        # Modules
        self.planner = FootstepPlanProvider(
            self.frame_info, self.reference, self.state, self.plan
        )
        self.mpc = ModelPredictiveController(self.frame_info, self.state, self.plan)
        self.ft_generator = FootTrajectoryGenerator(
            self.frame_info, self.state, self.plan
        )
        self.mc_provider = MovingConstraintProvider(
            self.frame_info, self.state, self.plan
        )
        self.filter = KalmanFilter()

        self.kinematics = Kinematics(self.robot, REDUNDANT_DOFS[config.robot])

        # Filling plan
        self.planner.update(self.plan)

    def customPreStep(self):

        start = time.time()
        self.robot.update(self.state, self.world)
        end = time.time()
        self.dart_elapsed += end - start

        start = time.time()
        if self.frame_info.k == 0:
            self.state.footstep.start_pose.translation[0] = (
                self.state.right_foot.pose.translation[0]
            )
            self.state.footstep.end_pose.translation[0] = (
                self.state.right_foot.pose.translation[0]
            )
            self.state.desired_right_foot.pose.translation[0] = (
                self.state.right_foot.pose.translation[0]
            )

        # TODO: THIS IS A TEST
        self.state.lip.zmp_pos = self.state.desired_lip.zmp_pos
        self.com_ball.setTranslation(self.state.lip.com_pos)
        self.des_com_ball.setTranslation(self.state.desired_lip.com_pos)
        self.zmp_ball.setTranslation(self.state.lip.zmp_pos)
        self.des_zmp_ball.setTranslation(self.state.desired_lip.zmp_pos)
        self.lsole_ball.setTranslation(self.state.left_foot.pose.translation)
        self.rsole_ball.setTranslation(self.state.right_foot.pose.translation)
        self.lsole_des_ball.setTranslation(
            self.state.desired_left_foot.pose.translation
        )
        self.rsole_des_ball.setTranslation(
            self.state.desired_right_foot.pose.translation
        )

        self.filter.update(self.state)
        # print("Filter updated")
        self.mc_provider.update(self.plan)
        # print("mc_provider updated")
        self.mpc.update(self.state)
        # print("mpc updated")
        self.state.lip = self.state.desired_lip
        self.ft_generator.update(self.state)
        # print("ft_generator updated")
        end = time.time()
        self.ismpc_elapsed += end - start

        # print("---------------------------------------------------")
        # print(f"LIP: \n {self.state.lip}")
        # print(f"LEFT FOOT: \n {self.state.left_foot.pose.translation}")
        # print(f"RIGHT FOOT: \n {self.state.right_foot.pose.translation}")
        # print("")
        # print(f"DESIRED LIP: \n {self.state.desired_lip}")
        # print(
        #     f"DESIRED LEFT FOOT: \n {self.state.desired_left_foot.pose.translation}"
        # )
        # print(
        #     f"DESIRED RIGHT FOOT: \n {self.state.desired_right_foot.pose.translation}"
        # )
        # print("")
        # print(f"FOOTSTEP: \n {self.state.footstep}")
        # print("---------------------------------------------------")
        # print("ITERATION NUMBER: ", self.frame_info.k)
        # print(f"TIME: {self.frame_info.tk:.2f}")

        start = time.time()

        lf_rotvec = R.from_matrix(
            self.state.desired_left_foot.pose.rotation.matrix()
        ).as_rotvec()
        rf_rotvec = R.from_matrix(
            self.state.desired_right_foot.pose.rotation.matrix()
        ).as_rotvec()
        self.state.desired_torso.pose.rotation = RotationMatrix(
            R.from_rotvec((lf_rotvec + rf_rotvec) / 2.0).as_matrix()
        )
        self.state.desired_base.pose.rotation = self.state.desired_torso.pose.rotation

        lf_rotvec_dot = self.state.desired_left_foot.ang_vel
        rf_rotvec_dot = self.state.desired_right_foot.ang_vel
        self.state.desired_torso.ang_vel = (lf_rotvec_dot + rf_rotvec_dot) / 2.0
        self.state.desired_base.ang_vel = self.state.desired_torso.ang_vel

        lf_rotvec_ddot = self.state.desired_left_foot.ang_acc
        rf_rotvec_ddot = self.state.desired_right_foot.ang_acc
        self.state.desired_torso.ang_acc = (lf_rotvec_ddot + rf_rotvec_ddot) / 2.0
        self.state.desired_base.ang_acc = self.state.desired_torso.ang_acc

        commands: np.ndarray = self.kinematics.get_joint_accelerations(self.state)
        # print("COMMANDS: \n", commands)

        # If you print the histogram you should remove all the other prints
        self.printCommandsHistogram(self.robot.jointList, commands)

        for i in range(self.kinematics.dofs - 6):
            self.robot.skeleton.setCommand(i + 6, commands[i])
        end = time.time()
        self.kin_elapsed += end - start

        self.frame_info.k += 1
        self.frame_info.tk += self.dt
        # print(
        #     "AVERAGE DART TIME IN MILLISECONDS: ",
        #     (self.dart_elapsed / self.frame_info.k) * 1000,
        # )
        # print(
        #     "AVERAGE ISMPC TIME IN MILLISECONDS: ",
        #     (self.ismpc_elapsed / self.frame_info.k) * 1000,
        # )
        # print(
        #     "AVERAGE KINEMATICS TIME IN MILLISECONDS: ",
        #     (self.kin_elapsed / self.frame_info.k) * 1000,
        # )
        # print("---------------------------------------------------\n\n\n")
        # print("---------------------------------------------------")

        # if self.frame_info.k > config.N:
        #     exit()

        # input("Press enter to apply command...")
        sleep(0.05)

    def create_ball(self, name: str, color: np.ndarray, radius: float = 0.1) -> dart.dynamics.SimpleFrame:
        ball_frame: dart.dynamics.SimpleFrame = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(), name, dart.math.Isometry3.Identity()
        )
        ball_frame.setShape(dart.dynamics.SphereShape(radius))
        ball_frame.createVisualAspect()
        ball_frame.getVisualAspect().setColor(color)
        return ball_frame

    def printCommandsHistogram(self, jointList, commands):
        commands_abs = np.abs(commands)
        max_val = np.max([30, np.max(commands_abs)])
        bar_length = 40  # Reduced from 60
        joint_width = 15  # Fixed width for joint name section
        total_width = bar_length + joint_width

        # Move cursor up
        for _ in range(len(commands) + 1):
            sys.stdout.write("\033[F")
            sys.stdout.flush()

        # Print centered scale
        scale = "-1" + "-" * (bar_length // 4) + "0" + "-" * (bar_length // 4) + "1"
        padding = " " * ((total_width - len(scale)) // 2)
        print(padding + scale)

        # Print histogram bars
        for joint, command in zip(jointList, commands):
            norm_value = command / max_val
            half_space = bar_length // 2

            # Truncate or pad joint name to fixed width
            if len(joint) > joint_width:
                joint = joint[:joint_width]
            else:
                joint = joint.center(joint_width)

            # Calculate bars
            if norm_value < 0:
                left_part = "█" * int(abs(norm_value) * half_space)
                left_space = " " * (half_space - len(left_part))
                right_space = " " * half_space
                output = f"{left_space}{left_part}|{joint}|{right_space}"
            else:
                right_part = "█" * int(norm_value * half_space)
                right_space = " " * (half_space - len(right_part))
                left_space = " " * half_space
                output = f"{left_space}|{joint}|{right_part}{right_space}"

            sys.stdout.write("\r" + output + "\n")
            sys.stdout.flush()
