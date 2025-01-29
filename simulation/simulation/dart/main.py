import dartpy as dart
import os
import numpy as np
from simulation.dart.controller import Controller
from simulation.dart.robot import Robot
from simulation.dart.config import REDUNDANT_DOFS, ROBOT
from simulation.utils.misc import project_root
from ismpc import State

if __name__ == "__main__":
    world = dart.simulation.World()
    urdfParser = dart.utils.DartLoader()
    models_root = os.path.join(project_root(), "models")
    
    skeleton = urdfParser.parseSkeleton(
        os.path.join(models_root, ROBOT, "urdf", f"{ROBOT}.urdf")
    )
    print("Loading urdf file: " + os.path.join(models_root, ROBOT, "urdf", f"{ROBOT}.urdf"))
    ground = urdfParser.parseSkeleton(
        os.path.join(models_root, ROBOT, "urdf", "ground.urdf")
    )
    world.addSkeleton(skeleton)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(1.0/60)
    
    for i in range(skeleton.getNumDofs()):
        joint_name = skeleton.getDof(i).getName()
        
    robot = Robot(skeleton)
    node = Controller(world, robot)
    node.setTargetRealTimeFactor(10)

    viewer = dart.gui.osg.Viewer()
    viewer.setUpViewInWindow(0, 0, 1280, 720)
    viewer.setCameraHomePosition([5.0, -1.0, 1.5], [1.0, 0.0, 0.5], [0.0, 0.0, 1.0])
    viewer.addWorldNode(node)
    viewer.run()