from typing import Dict, List
from simulation.utils.misc import repo_root, load_config
import os
from dataclasses import dataclass
from typing import Literal

@dataclass
class Config:
    save_log: bool = False
    delta: float = 0.02
    first_fs_duration: float = 1.0
    N: int = 500
    P: int = 1000
    C: int = 100
    W: int = 100
    robot: str = "hrp4"
    des_vel_x: float = 0.1
    des_vel_y: float = 0.0
    des_omega: float = 0.0
    tail_type: str = "PERIODIC"

config = load_config(os.path.join(repo_root(), "config", "config.yaml"), Config)

N: int = config.N
ROBOT: str = config.robot

LINK_NAMES: Dict[str, List[str]] = {
    "hrp4": ["l_sole", "r_sole", "torso", "body"],
    "nao": ["LSole", "RSole", "Torso", "base_link"],
}

REDUNDANT_DOFS: Dict[str, List[str]] = {
    "hrp4": [
        "NECK_Y",
        "NECK_P",
        "R_SHOULDER_P",
        "R_SHOULDER_R",
        "R_SHOULDER_Y",
        "R_ELBOW_P",
        "L_SHOULDER_P",
        "L_SHOULDER_R",
        "L_SHOULDER_Y",
        "L_ELBOW_P",
    ],
    "nao": [
        "HeadYaw",
        "HeadPitch",
        "LWristYaw",
        "LHand",
        "RWristYaw",
        "RHand",
    ],
}

INITIAL_CONFIG: Dict[str, Dict[str, float]] = {
    "hrp4": {
        "CHEST_P": 0.0,
        "CHEST_Y": 0.0,
        "NECK_P": 0.0,
        "NECK_Y": 0.0,
        "R_HIP_Y": 0.0,
        "R_HIP_R": -3.0,
        "R_HIP_P": -25.0,
        "R_KNEE_P": 50.0,
        "R_ANKLE_P": -25.0,
        "R_ANKLE_R": 3.0,
        "L_HIP_Y": 0.0,
        "L_HIP_R": 3.0,
        "L_HIP_P": -25.0,
        "L_KNEE_P": 50.0,
        "L_ANKLE_P": -25.0,
        "L_ANKLE_R": -3.0,
        "R_SHOULDER_P": 4.0,
        "R_SHOULDER_R": -8.0,
        "R_SHOULDER_Y": 0.0,
        "R_ELBOW_P": -25.0,
        "L_SHOULDER_P": 4.0,
        "L_SHOULDER_R": 8.0,
        "L_SHOULDER_Y": 0.0,
        "L_ELBOW_P": -25.0,
    },
    "nao": {
        "HeadYaw": 0.0,
        "HeadPitch": 0.0,
        "LHipYawPitch": 0.0,
        "LHipRoll": -3,
        "LHipPitch": -15,
        "LKneePitch": 30,
        "LAnklePitch": -15,
        "LAnkleRoll": 0.0,
        "LShoulderPitch": 90,
        "LShoulderRoll": 0.0,
        "LElbowYaw": 0.0,
        "LElbowRoll": 0.0,
        "LWristYaw": 0.0,
        "LHand": 0.0,
        "RHipYawPitch": 0.0,
        "RHipRoll": -3,
        "RHipPitch": -15,
        "RKneePitch": 30,
        "RAnklePitch": -15,
        "RAnkleRoll": 0.0,
        "RShoulderPitch": 90.0,
        "RShoulderRoll": 0.0,
        "RElbowYaw": 0.0,
        "RElbowRoll": 0.0,
        "RWristYaw": 0.0,
        "RHand": 0.0,
    },
}
