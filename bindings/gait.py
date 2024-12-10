import numpy as np
from ismpc_cpp import LipRobot, WalkEngine

class Gait:

    engine: WalkEngine
    robot: LipRobot

    history: np.ndarray
    walk_history: np.ndarray
    com_traj: np.ndarray
    zmp_traj: np.ndarray
    left_foot_traj: np.ndarray
    right_foot_traj: np.ndarray

    xf: np.ndarray
    yf: np.ndarray
    thetaf: np.ndarray
    timestamps: np.ndarray
    footstep_history: np.ndarray

    def __init__(self, engine: WalkEngine):
        self.engine = engine
        self.robot = engine.get_robot()
        self.extract_traj()
        self.extract_footsteps()

    def extract_traj(self) -> None:
        history = self.engine.get_history()
        com_traj = np.ndarray(shape=(3, len(history)), dtype=float)
        zmp_traj = np.ndarray(shape=(3, len(history)), dtype=float)
        left_foot_traj = np.ndarray(shape=(3, len(history)), dtype=float)
        right_foot_traj = np.ndarray(shape=(3, len(history)), dtype=float)
        for i, state in enumerate(history):
            com_traj[:, i] = state.com.pose.translation
            zmp_traj[:, i] = state.zmp_pos
            left_foot_traj[:, i] = state.left_foot.pose.translation
            right_foot_traj[:, i] = state.right_foot.pose.translation
        self.com_traj = com_traj
        self.zmp_traj = zmp_traj
        self.left_foot_traj = left_foot_traj
        self.right_foot_traj = right_foot_traj
        self.hppistory = history

    def extract_footsteps(self) -> None:
        footstep_history = self.engine.get_footstep_history()
        xf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        yf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        thetaf = np.ndarray(shape=(len(footstep_history)), dtype=float)
        for i, footstep in enumerate(footstep_history):
            xf[i] = footstep.translation[0]
            yf[i] = footstep.translation[1]
            thetaf[i] = footstep.rotation()

        timestamp_history = self.engine.get_timestamp_history()
        timestamps = np.ndarray(shape=(len(timestamp_history)), dtype=float)
        for i, timestamp in enumerate(timestamp_history):
            timestamps[i] = timestamp

        self.xf = xf
        self.yf = yf
        self.thetaf = thetaf
        self.timestamps = timestamps
        self.footstep_history = footstep_history
