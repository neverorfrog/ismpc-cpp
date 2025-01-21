import dartpy as dart
import os
import numpy as np
from controller import Controller
from robot import Robot
from ismpc_sim.dart.config import REDUNDANT_DOFS

if __name__ == "__main__":
    print(REDUNDANT_DOFS)
    # world = dart.simulation.World()
    # urdfParser = dart.utils.DartLoader()
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # skeleton = urdfParser.parseSkeleton(
    #     os.path.join(current_dir, "naov4", "urdf", "nao.urdf")
    # )
    # ground = urdfParser.parseSkeleton(
    #     os.path.join(current_dir, "naov4", "urdf", "ground.urdf")
    # )
    # world.addSkeleton(skeleton)
    # world.addSkeleton(ground)
    # world.setGravity([0, 0, -9.81])
    # world.setTimeStep(1.0/60)
    
    # for i in range(skeleton.getNumDofs()):
    #     joint_name = skeleton.getDof(i).getName()
    #     print(joint_name)
        
    # robot = Robot(skeleton)
    # node = Controller(world, robot)
    # node.setTargetRealTimeFactor(10)

    # viewer = dart.gui.osg.Viewer()
    # viewer.setUpViewInWindow(0, 0, 1280, 720)
    # viewer.setCameraHomePosition([5.0, -1.0, 1.5], [1.0, 0.0, 0.5], [0.0, 0.0, 1.0])
    # viewer.addWorldNode(node)
    # viewer.run()
