import numpy as np
from ismpc_py import State

class Gait:

    state: State

    history: np.ndarray
    com_traj: np.ndarray
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
        self.extract_traj()
        self.extract_footsteps()

    def extract_traj(self) -> None:
        lip_history = self.state.lip_history
        com_traj = np.ndarray(shape=(3, len(lip_history)), dtype=float)
        zmp_traj = np.ndarray(shape=(3, len(lip_history)), dtype=float)
        for i, state in enumerate(lip_history):
            com_traj[:, i] = state.com_pos
            zmp_traj[:, i] = state.zmp_pos
        self.com_traj = com_traj
        self.zmp_traj = zmp_traj

        lf_history = self.state.left_foot_history
        rf_history = self.state.right_foot_history
        left_foot_traj = np.ndarray(shape=(3, len(lf_history)), dtype=float)
        right_foot_traj = np.ndarray(shape=(3, len(rf_history)), dtype=float)
        for i, foot in enumerate(lf_history):
            left_foot_traj[:, i] = foot.pose.translation
        for i, foot in enumerate(rf_history):
            right_foot_traj[:, i] = foot.pose.translation
        self.left_foot_traj = left_foot_traj
        self.right_foot_traj = right_foot_traj

    def extract_footsteps(self) -> None:
        footstep_history = self.state.fs_history
        xf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        yf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        thetaf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        timestamps = np.ndarray(shape=(len(footstep_history)), dtype=float)
        for i, footstep in enumerate(footstep_history):
            pose = footstep.end_pose
            xf[i] = pose.translation[0]
            yf[i] = pose.translation[1]
            thetaf[i] = pose.rotation()
            timestamps[i] = footstep.timestamp

        self.xf = xf
        self.yf = yf
        self.thetaf = thetaf
        self.timestamps = timestamps
        self.footstep_history = footstep_history
