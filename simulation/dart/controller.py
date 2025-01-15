import numpy as np
import dartpy as dart
import time
from ismpc_py import FrameInfo, Reference, State, FootstepPlan
from ismpc_py import (
    FootstepPlanProvider,
    ModelPredictiveController,
    FootTrajectoryGenerator,
    MovingConstraintProvider,
)


class Hrp4Controller(dart.gui.osg.RealTimeWorldNode):

    def __init__(self, world: dart.simulation.World, hrp4: dart.dynamics.Skeleton):
        super(Hrp4Controller, self).__init__(world)
        self.world = world
        self.hrp4 = hrp4
        self.dofs = self.hrp4.getNumDofs()
        world.setTimeStep(0.01)
        self.world_time_step = world.getTimeStep()
        self.time = 0
        self.elapsed = 0

        # Representations
        frame_info = FrameInfo()
        reference = Reference()
        state = State()
        plan = FootstepPlan()

        # Modules
        planner = FootstepPlanProvider(frame_info, reference, state, plan)
        mpc = ModelPredictiveController(frame_info, state, plan)
        ft_generator = FootTrajectoryGenerator(frame_info, state, plan)
        mc_provider = MovingConstraintProvider(frame_info, state, plan)

        # robot links
        self.lsole = hrp4.getBodyNode("l_sole")
        self.rsole = hrp4.getBodyNode("r_sole")
        self.torso = hrp4.getBodyNode("torso")
        self.base = hrp4.getBodyNode("body")

        for i in range(hrp4.getNumJoints()):
            joint = hrp4.getJoint(i)
            dim = joint.getNumDofs()

            # this sets the root joint to passive
            if dim == 6:
                joint.setActuatorType(dart.dynamics.ActuatorType.PASSIVE)

            # this sets the remaining joints as acceleration-controlled
            elif dim == 1:
                joint.setActuatorType(dart.dynamics.ActuatorType.ACCELERATION)

        # set initial configuration
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

        for joint_name, value in initial_configuration.items():
            self.hrp4.setPosition(
                self.hrp4.getDof(joint_name).getIndexInSkeleton(), value * np.pi / 180.0
            )

        # position the robot on the ground
        lsole_pos = self.lsole.getTransform(
            withRespectTo=dart.dynamics.Frame.World(),
            inCoordinatesOf=dart.dynamics.Frame.World(),
        ).translation()
        rsole_pos = self.rsole.getTransform(
            withRespectTo=dart.dynamics.Frame.World(),
            inCoordinatesOf=dart.dynamics.Frame.World(),
        ).translation()
        self.hrp4.setPosition(5, -(lsole_pos[2] + rsole_pos[2]) / 2.0)

    def customPreStep(self):
        start = time.time()
        self.time += 1
        end = time.time()
        self.elapsed += end - start
        print("AVERAGE TIME IN MILLISECONDS: ", (self.elapsed / self.time) * 1000)
