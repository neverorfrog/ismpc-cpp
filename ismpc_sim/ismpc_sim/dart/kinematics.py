from typing import Dict, List
import dartpy as dart
import numpy as np
from utils import rotation_difference, pose_difference
from qpsolvers import solve_qp
from ismpc import State
from robot import Robot


class Kinematics:
    def __init__(self, robot: Robot, redundant_dofs: List[str]):
        self.robot = robot.skeleton
        self.dofs = self.robot.getNumDofs()
        self.initial_q = self.robot.getPositions()
        self.initial_qdot = self.robot.getVelocities()
        self.q_ddot_des = np.zeros(self.dofs)

        # selection matrix for redundant dofs
        self.joint_selection = np.zeros((self.dofs, self.dofs))
        for i in range(self.dofs):
            joint_name = self.robot.getDof(i).getName()
            if joint_name in redundant_dofs:
                self.joint_selection[i, i] = 1

    def get_joint_accelerations(self, state: State):
        # self.robot parameters
        lsole = self.robot.getBodyNode("l_sole")
        rsole = self.robot.getBodyNode("r_sole")
        torso = self.robot.getBodyNode("torso")
        base = self.robot.getBodyNode("body")

        # weights and gains
        tasks = ["lsole", "rsole", "com", "torso", "base", "joints"]
        weights = {
            "lsole": 1.0,
            "rsole": 1.0,
            "com": 1.0,
            "torso": 1.0,
            "base": 1.0,
            "joints": 1.0e-2,
        }
        pos_gains = {
            "lsole": 5.0,
            "rsole": 5.0,
            "com": 5.0,
            "torso": 1,
            "base": 1,
            "joints": 10.0,
        }
        vel_gains = {
            "lsole": 10.0,
            "rsole": 10.0,
            "com": 10.0,
            "torso": 2,
            "base": 2,
            "joints": 1.0e-1,
        }

        # jacobians
        J: Dict[str, np.ndarray] = {
            "lsole": self.robot.getJacobian(
                lsole, inCoordinatesOf=dart.dynamics.Frame.World()  # [6, dof]
            ),
            "rsole": self.robot.getJacobian(
                rsole, inCoordinatesOf=dart.dynamics.Frame.World()  # [6, dof]
            ),
            "com": self.robot.getCOMLinearJacobian(
                inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "torso": self.robot.getAngularJacobian(
                torso, inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "base": self.robot.getAngularJacobian(
                base, inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "joints": self.joint_selection,  # [dof, dof]
        }

        # jacobians derivatives
        Jdot: Dict[str, np.ndarray] = {
            "lsole": self.robot.getJacobianClassicDeriv(
                lsole, inCoordinatesOf=dart.dynamics.Frame.World()  # [6, dof]
            ),
            "rsole": self.robot.getJacobianClassicDeriv(
                rsole, inCoordinatesOf=dart.dynamics.Frame.World()  # [6, dof]
            ),
            "com": self.robot.getCOMLinearJacobianDeriv(
                inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "torso": self.robot.getAngularJacobianDeriv(
                torso, inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "base": self.robot.getAngularJacobianDeriv(
                base, inCoordinatesOf=dart.dynamics.Frame.World()  # [3, dof]
            ),
            "joints": np.zeros((self.dofs, self.dofs)),  # [dof, dof]
        }

        # feedforward terms
        ff: Dict[str, np.ndarray] = {
            "lsole": np.hstack(
                (
                    state.desired_left_foot.ang_acc,
                    state.desired_left_foot.lin_acc,
                )  # [6]
            ),
            "rsole": np.hstack(
                (
                    state.desired_right_foot.ang_acc,
                    state.desired_right_foot.lin_acc,
                )  # [6]
            ),
            "com": state.desired_lip.com_acc,  # [3]
            "torso": state.desired_torso.ang_acc,  # [3]
            "base": state.desired_base.ang_acc,  # [3]
            "joints": np.zeros(self.dofs),  # [dof]
        }

        # error vectors
        pos_error: Dict[str, np.ndarray] = {
            "lsole": pose_difference(
                state.desired_left_foot.pose,
                state.left_foot.pose,
            ),
            "rsole": pose_difference(
                state.desired_right_foot.pose,
                state.right_foot.pose,
            ),
            "com": state.desired_lip.com_pos - state.lip.com_pos,
            "torso": rotation_difference(
                state.desired_torso.pose,
                state.torso.pose,
            ),
            "base": rotation_difference(
                state.desired_base.pose,
                state.base.pose,
            ),
            "joints": self.initial_q - self.robot.getPositions(),
        }

        # velocity error vectors
        vel_error = {
            "lsole": np.hstack(
                (
                    state.desired_left_foot.ang_vel - state.left_foot.ang_vel,
                    state.desired_left_foot.lin_vel - state.left_foot.lin_vel,
                )
            ),
            "rsole": np.hstack(
                (
                    state.desired_right_foot.ang_vel - state.right_foot.ang_vel,
                    state.desired_right_foot.lin_vel - state.right_foot.lin_vel,
                )
            ),
            "com": state.desired_lip.com_vel - state.lip.com_vel,
            "torso": state.desired_torso.ang_vel - state.torso.ang_vel,
            "base": state.desired_base.ang_vel - state.base.ang_vel,
            "joints": self.initial_qdot - self.robot.getVelocities(),
        }

        # cost function
        cost_matrix = np.zeros((self.dofs, self.dofs))
        cost_vector = np.zeros(self.dofs)
        for task in tasks:
            cost_matrix += weights[task] * J[task].T @ J[task]
            cost_vector += (
                -weights[task]
                * J[task].T
                @ (
                    ff[task]
                    + vel_gains[task] * vel_error[task]
                    + pos_gains[task] * pos_error[task]
                    - Jdot[task] @ self.robot.getVelocities()
                )
            )

        self.q_ddot_des = solve_qp(cost_matrix, cost_vector, solver="proxqp", initvals=self.q_ddot_des)
        
        if (self.q_ddot_des is None):
            exit("Kinematics QP solver failed to find a solution")
        
        return self.q_ddot_des[-(self.dofs - 6) :]
