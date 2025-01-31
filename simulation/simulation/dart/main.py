import dartpy as dart
import os
import numpy as np
from simulation.dart.controller import Controller
from simulation.dart.robot import Robot
from simulation.dart.misc import REDUNDANT_DOFS
from simulation.utils import project_root
from simulation.utils import config



if __name__ == "__main__":
    world = dart.simulation.World()
    urdfParser = dart.utils.DartLoader()
    models_root = os.path.join(project_root(), "models")
    
    skeleton = urdfParser.parseSkeleton(
        os.path.join(models_root, config.robot, "urdf", f"{config.robot}.urdf")
    )
    ground = urdfParser.parseSkeleton(
        os.path.join(models_root, config.robot, "urdf", "ground.urdf")
    )
    world.addSkeleton(skeleton)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    
    for i in range(skeleton.getNumDofs()):
        joint_name = skeleton.getDof(i).getName()
        
    robot = Robot(skeleton)
    node = Controller(world, robot)
    node.setTargetRealTimeFactor(2)

    viewer = dart.gui.osg.Viewer()
    viewer.setUpViewInWindow(0, 0, 1920, 1080)
    viewer.setCameraHomePosition([5.0, -1.0, 1.5], [1.0, 0.0, 0.5], [0.0, 0.0, 1.0])
    viewer.addWorldNode(node)
    viewer.run()