import dartpy as dart
import os
import numpy as np
from controller import Hrp4Controller

if __name__ == "__main__":
    world = dart.simulation.World()
    urdfParser = dart.utils.DartLoader()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    hrp4 = urdfParser.parseSkeleton(
        os.path.join(current_dir, "hrp4", "urdf", "hrp4.urdf")
    )
    ground = urdfParser.parseSkeleton(
        os.path.join(current_dir, "hrp4", "urdf", "ground.urdf")
    )
    world.addSkeleton(hrp4)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    default_inertia = dart.dynamics.Inertia(1e-8, np.zeros(3), 1e-10 * np.identity(3))

    for body in hrp4.getBodyNodes():
        if body.getMass() == 0.0:
            body.setMass(1e-8)
            body.setInertia(default_inertia)
    node = Hrp4Controller(world, hrp4)
    node.setTargetRealTimeFactor(10)

    viewer = dart.gui.osg.Viewer()
    viewer.setUpViewInWindow(0, 0, 1280, 720)
    viewer.setCameraHomePosition([5.0, -1.0, 1.5], [1.0, 0.0, 0.5], [0.0, 0.0, 1.0])
    viewer.addWorldNode(node)
    viewer.run()
