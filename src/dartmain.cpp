#include <chrono>
#include <dart/dart.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "ismpc_cpp/dart/controller.h"
#include "ismpc_cpp/dart/simulated_robot.h"
#include "ismpc_cpp/tools/config/config.h"

int main() {
    dart::simulation::WorldPtr world(new dart::simulation::World);

    dart::utils::DartLoader::Options options;
    dart::utils::DartLoader urdfLoader(options);
    options.mDefaultInertia =
        dart::dynamics::Inertia(1e-8, Eigen::Vector3d::Zero(), 1e-10 * Eigen::Matrix3d::Identity());
    auto robot = urdfLoader.parseSkeleton(realpath("simulation/urdf/hrp4/urdf/hrp4.urdf", NULL));
    auto ground = urdfLoader.parseSkeleton(realpath("simulation/urdf/ground.urdf", NULL));
    world->addSkeleton(ground);
    world->addSkeleton(robot);
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(ismpc::Config::delta);

    for (auto* bn : robot->getBodyNodes()) {
        if (bn->getMass() == 0.0) {
            bn->setMass(1e-8);
            bn->setInertia(options.mDefaultInertia);
        }
    }

    // set joint actuator type
    for (size_t i = 0; i < robot->getNumJoints(); i++) {
        size_t dim = robot->getJoint(i)->getNumDofs();
        if (dim == 6) {
            robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
        }
        if (dim == 1) {
            robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ACCELERATION);
        }
    }

    // Wrap a WorldNode around the world and robot
    osg::ref_ptr<ismpc::Controller> node = new ismpc::Controller(world, robot);
    node->setNumStepsPerCycle(1);

    // Create a viewer and set it up with the WorldNode
    dart::gui::osg::ImGuiViewer viewer;
    viewer.addWorldNode(node);

    // Set the dimensions for the window
    viewer.setUpViewInWindow(0, 0, 1280, 720);
    if (ismpc::Config::save_log) {
        std::filesystem::create_directories("videos/dart");
        viewer.record("videos/dart", "frame");
    }

    // Set the window name
    viewer.realize();
    osgViewer::Viewer::Windows windows;
    viewer.getWindows(windows);
    windows.front()->setWindowName("Intrinsincally Stable Model Predictive Control");

    // Adjust the viewpoint of the Viewer
    viewer.getCameraManipulator()->setHomePosition(::osg::Vec3d(5, -1, 1.5), ::osg::Vec3d(1, 0, 0.5),
                                                   ::osg::Vec3d(0, 0, 1));

    // We need to re-dirty the CameraManipulator by passing it into the viewer
    // again, so that the viewer knows to update its HomePosition setting
    viewer.setCameraManipulator(viewer.getCameraManipulator());

    // Run the simulation!
    viewer.run();
}
