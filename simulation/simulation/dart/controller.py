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
    KalmanFilter
)
from scipy.spatial.transform import Rotation as R
from simulation.dart.config import REDUNDANT_DOFS, ROBOT, N

class Controller(dart.gui.osg.RealTimeWorldNode):

    def __init__(self, world: dart.simulation.World, robot: Robot):
        super(Controller, self).__init__(world)
        self.world = world
        self.robot = robot
        world.setTimeStep(0.01)
        self.dt = world.getTimeStep()
        self.dart_elapsed = 0
        self.ismpc_elapsed = 0
        self.kin_elapsed = 0

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

        self.kinematics = Kinematics(self.robot, REDUNDANT_DOFS[ROBOT])

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
        self.filter.update(self.state)
        self.mc_provider.update(self.plan)
        self.mpc.update(self.state)
        self.state.lip = self.state.desired_lip
        self.ft_generator.update(self.state)
        end = time.time()
        self.ismpc_elapsed += end - start

        print("---------------------------------------------------")
        print(f"LIP: \n {self.state.lip}")
        print(f"LEFT FOOT: \n {self.state.left_foot.pose.translation}")
        print(f"RIGHT FOOT: \n {self.state.right_foot.pose.translation}")
        print("")
        print(f"DESIRED LIP: \n {self.state.desired_lip}")
        print(
            f"DESIRED LEFT FOOT: \n POS: {self.state.desired_left_foot.pose.translation} \n ROT: {self.state.desired_left_foot.pose.rotation}"
        )
        print(
            f"DESIRED RIGHT FOOT: \n {self.state.desired_right_foot.pose.translation}"
        )
        print("")
        print(f"FOOTSTEP: \n {self.state.footstep}")
        print("---------------------------------------------------")
        print("ITERATION NUMBER: ", self.frame_info.k)
        print(f"TIME: {self.frame_info.tk:.2f}")

        start = time.time()
        
        lf_rotvec = R.from_matrix(self.state.desired_left_foot.pose.rotation.matrix()).as_rotvec()
        rf_rotvec = R.from_matrix(self.state.desired_right_foot.pose.rotation.matrix()).as_rotvec()
        self.state.desired_torso.pose.rotation = RotationMatrix(R.from_rotvec((lf_rotvec + rf_rotvec) / 2.0).as_matrix())
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
        print("COMMANDS: \n", commands)
        print("\n\n")
        for i in range(self.kinematics.dofs - 6):
            self.robot.skeleton.setCommand(i + 6, commands[i])
        end = time.time()
        self.kin_elapsed += end - start

        self.frame_info.k += 1
        self.frame_info.tk += self.dt
        print(
            "AVERAGE DART TIME IN MILLISECONDS: ",
            (self.dart_elapsed / self.frame_info.k) * 1000,
        )
        print(
            "AVERAGE ISMPC TIME IN MILLISECONDS: ",
            (self.ismpc_elapsed / self.frame_info.k) * 1000,
        )
        print(
            "AVERAGE KINEMATICS TIME IN MILLISECONDS: ",
            (self.kin_elapsed / self.frame_info.k) * 1000,
        )

        if self.frame_info.k > N:
            exit()