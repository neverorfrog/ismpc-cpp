import dartpy as dart
import os
import numpy as np
from controller import Hrp4Controller
from robot import Robot

if __name__ == "__main__":
    world = dart.simulation.World()
    urdfParser = dart.utils.DartLoader()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    skeleton = urdfParser.parseSkeleton(
        os.path.join(current_dir, "hrp4", "urdf", "hrp4.urdf")
    )
    ground = urdfParser.parseSkeleton(
        os.path.join(current_dir, "hrp4", "urdf", "ground.urdf")
    )
    world.addSkeleton(skeleton)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(1.0/60)
    
    # robot = Robot(skeleton)
    # node = Hrp4Controller(world, robot)
    # node.setTargetRealTimeFactor(10)

    # viewer = dart.gui.osg.Viewer()
    # viewer.setUpViewInWindow(0, 0, 1280, 720)
    # viewer.setCameraHomePosition([5.0, -1.0, 1.5], [1.0, 0.0, 0.5], [0.0, 0.0, 1.0])
    # viewer.addWorldNode(node)
    # viewer.run()
