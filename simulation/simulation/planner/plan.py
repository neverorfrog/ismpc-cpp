from typing import List
import numpy as np
from ismpc import State, Footstep, LipState


class Plan:

    state: State

    history: np.ndarray
    zmp_traj: np.ndarray
    left_foot_traj: np.ndarray
    right_foot_traj: np.ndarray

    xf: np.ndarray
    yf: np.ndarray
    thetaf: np.ndarray
    timestamps: np.ndarray
    footstep_history: np.ndarray

    def __init__(self, state: State):
        self.state = state
        footstep_history: List[Footstep] = state.fs_history
        xf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        yf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        thetaf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        timestamps = np.ndarray(shape=(len(footstep_history)), dtype=float)
        for i, footstep in enumerate(footstep_history):
            pose = footstep.end_pose
            xf[i] = pose.translation[0]
            yf[i] = pose.translation[1]
            thetaf[i] = pose.rotation()
            timestamps[i] = footstep.start
            
        lip_history: List[LipState] = state.lip_history
        zmp_traj = np.ndarray(shape=(len(lip_history), 3), dtype=float)
        for i, lip_state in enumerate(lip_history):
            zmp_traj[i, :] = lip_state.zmp_pos

        self.xf = xf
        self.yf = yf
        self.thetaf = thetaf
        self.timestamps = timestamps
        self.zmp_traj = zmp_traj
        self.footstep_history = footstep_history