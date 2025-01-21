import numpy as np
import dartpy as dart
from ismpc import State, RotationMatrix


initial_configuration = {
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
}

initial_configuration = {
    "HeadYaw": 0.0,
    "HeadPitch": 0.0,
    "LHipYawPitch": 0.0,
    "LHipRoll": 0.0,
    "LHipPitch": 0.0,
    "LKneePitch": 0.0,
    "LAnklePitch": 0.0,
    "LAnkleRoll": 0.0,
    "LShoulderPitch": 90,
    "LShoulderRoll": 0.0,
    "LElbowYaw": 0.0,
    "LElbowRoll": 0.0,
    "LWristYaw": 0.0,
    "LHand": 0.0,
    "RHipYawPitch": 0.0,
    "RHipRoll": 0.0,
    "RHipPitch": 0.0,
    "RKneePitch": 0.0,
    "RAnklePitch": 0.0,
    "RAnkleRoll": 0.0,
    "RShoulderPitch": 90.0,
    "RShoulderRoll": 0.0,
    "RElbowYaw": 0.0,
    "RElbowRoll": 0.0,
    "RWristYaw": 0.0,
    "RHand": 0.0,
}

worldFrame: dart.dynamics.Frame = dart.dynamics.Frame.World()

class Robot:

    def __init__(self, skeleton: dart.dynamics.Skeleton):

        # robot links
        self.lsole = skeleton.getBodyNode("LSole")
        self.rsole = skeleton.getBodyNode("RSole")
        self.torso = skeleton.getBodyNode("Torso")
        self.base = skeleton.getBodyNode("base_link")

        # set joint types
        for i in range(skeleton.getNumJoints()):
            joint = skeleton.getJoint(i)
            dim = joint.getNumDofs()

            # this sets the root joint to passive
            if dim == 6:
                joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)

            # this sets the remaining joints as position-controlled
            elif dim == 1:
                joint.setActuatorType(dart.dynamics.ActuatorType.ACCELERATION)

        # set initial configuration
        for joint_name, value in initial_configuration.items():
            skeleton.setPosition(
                skeleton.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.0
            )

        # set position the robot on the ground
        lsole_pos = self.lsole.getTransform(
            withRespectTo=dart.dynamics.Frame.World(),
            inCoordinatesOf=dart.dynamics.Frame.World(),
        ).translation()
        rsole_pos = self.rsole.getTransform(
            withRespectTo=dart.dynamics.Frame.World(),
            inCoordinatesOf=dart.dynamics.Frame.World(),
        ).translation()
        skeleton.setPosition(5, -(lsole_pos[2] + rsole_pos[2]) / 2.0)

        # set mass and inertia for zero-mass bodies
        default_inertia = dart.dynamics.Inertia(
            1e-8, np.zeros(3), 1e-10 * np.identity(3)
        )
        for body in skeleton.getBodyNodes():
            if body.getMass() == 0.0:
                body.setMass(1e-8)
                body.setInertia(default_inertia)

        self.skeleton = skeleton

    def update(self, state: State, world: dart.simulation.World):

        # COM
        state.lip.com_pos = self.skeleton.getCOM()
        state.lip.com_vel = self.skeleton.getCOMLinearVelocity(
            relativeTo=worldFrame, inCoordinatesOf=worldFrame
        )

        # Torso
        torso_transform: dart.math.Isometry3 = self.torso.getTransform(
            withRespectTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.torso.pose.rotation = RotationMatrix(torso_transform.rotation())
        state.torso.ang_vel = self.torso.getAngularVelocity(
            relativeTo=worldFrame, inCoordinatesOf=worldFrame
        )

        # Base
        base_transform: dart.math.Isometry3 = self.base.getTransform(
            withRespectTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.base.pose.rotation = RotationMatrix(base_transform.rotation())
        state.base.ang_vel = self.base.getAngularVelocity(
            relativeTo=worldFrame, inCoordinatesOf=worldFrame
        )

        # Left Foot
        lsole_transform: dart.math.Isometry3 = self.lsole.getTransform(
            withRespectTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.left_foot.pose.translation = lsole_transform.translation()
        state.left_foot.pose.rotation = RotationMatrix(lsole_transform.rotation())
        vel: np.ndarray = self.lsole.getSpatialVelocity(
            relativeTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.left_foot.ang_vel = vel[:3]
        state.left_foot.ang_acc = np.zeros((3,))
        state.left_foot.lin_vel = vel[3:]
        state.left_foot.lin_acc = np.zeros((3,))

        # Right Foot
        rsole_transform: dart.math.Isometry3 = self.rsole.getTransform(
            withRespectTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.right_foot.pose.translation = rsole_transform.translation()
        state.right_foot.pose.rotation = RotationMatrix(rsole_transform.rotation())
        vel: np.ndarray = self.rsole.getSpatialVelocity(
            relativeTo=worldFrame, inCoordinatesOf=worldFrame
        )
        state.right_foot.ang_vel = vel[:3]
        state.right_foot.ang_acc = np.zeros((3,))
        state.right_foot.lin_vel = vel[3:]
        state.right_foot.lin_acc = np.zeros((3,))

        # ZMP
        total_vertical_force = 0.0
        zmp = np.zeros(3)
        for contact in world.getLastCollisionResult().getContacts():
            total_vertical_force += contact.force[2]
            zmp[0] += contact.point[0] * contact.force[2]
            zmp[1] += contact.point[1] * contact.force[2]
        if total_vertical_force > 0.1:  # threshold for when we lose contact
            print("FEET ARE ON THE GROUND")
            zmp /= total_vertical_force
            # sometimes we get contact points that dont make sense, so we clip the ZMP close to the robot
            midpoint = (
                state.left_foot.pose.translation + state.right_foot.pose.translation
            ) / 2.0
            zmp[0] = np.clip(zmp[0], midpoint[0] - 0.3, midpoint[0] + 0.3)
            zmp[1] = np.clip(zmp[1], midpoint[1] - 0.3, midpoint[1] + 0.3)
            state.lip.zmp_pos = zmp

        return state
