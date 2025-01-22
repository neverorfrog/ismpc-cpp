import numpy as np
import dartpy as dart
from ismpc import State, RotationMatrix
from simulation.dart.config import INITIAL_CONFIG, ROBOT, LINK_NAMES

worldFrame: dart.dynamics.Frame = dart.dynamics.Frame.World()

class Robot:

    def __init__(self, skeleton: dart.dynamics.Skeleton):

        # robot links
        self.lsole = skeleton.getBodyNode(LINK_NAMES[ROBOT][0])
        self.rsole = skeleton.getBodyNode(LINK_NAMES[ROBOT][1])
        self.torso = skeleton.getBodyNode(LINK_NAMES[ROBOT][2])
        self.base = skeleton.getBodyNode(LINK_NAMES[ROBOT][3])

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
        for joint_name, value in INITIAL_CONFIG[ROBOT].items():
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