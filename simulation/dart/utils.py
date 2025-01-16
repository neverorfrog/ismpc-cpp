from scipy.spatial.transform import Rotation as R
import numpy as np
from ismpc_py import Pose3


def position_difference(pose_a: Pose3, pose_b: Pose3):
    return pose_b.translation - pose_a.translation


def rotation_difference(pose_a: Pose3, pose_b: Pose3):
    rot_a: R = R.from_matrix(pose_a.rotation.matrix())
    rot_b: R = R.from_matrix(pose_b.rotation.matrix())
    R_diff: R = rot_b.inv() * rot_a
    return R_diff.as_rotvec()


def pose_difference(pose_a: Pose3, pose_b: Pose3):
    pos_diff = pose_b.translation - pose_a.translation
    rot_diff = rotation_difference(pose_a, pose_b)
    return np.hstack((rot_diff, pos_diff))


# converts a rotation matrix to a rotation vector
def get_rotvec(rot_matrix):
    rotation = R.from_matrix(rot_matrix)
    return rotation.as_rotvec()
