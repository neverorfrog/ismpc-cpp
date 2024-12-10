#include <chrono>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "tools/config/config.h"
#include "simulation/simulated_robot.h"
#include "simulation/world_node.h"

int main() {
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and HRP4 robot and add them to the world
  dart::utils::DartLoader::Options options;
  options.mDefaultInertia =
      dart::dynamics::Inertia(1e-8, Eigen::Vector3d::Zero(), 1e-10 * Eigen::Matrix3d::Identity());
  dart::utils::DartLoader urdfLoader(options);
  auto ground = urdfLoader.parseSkeleton(realpath("external/hrp4/urdf/ground.urdf", NULL));
  auto robot = urdfLoader.parseSkeleton(realpath("external/hrp4/urdf/hrp4.urdf", NULL));
  world->addSkeleton(ground);
  world->addSkeleton(robot);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() += Eigen::Vector3d(3.0, 0.0, 0.0);
  ground->getJoint(0)->setTransformFromParentBodyNode(tf);

  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(ismpc::Config::delta);

  // set joint actuator type and force limits
  double forceLimit = 100;
  for (size_t i = 0; i < robot->getNumJoints(); i++) {
    size_t dim = robot->getJoint(i)->getNumDofs();
    if (dim == 6) {
      robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
    }
    if (dim == 1) {
      robot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ACCELERATION);
      robot->getJoint(i)->setForceUpperLimit(0, forceLimit);
      robot->getJoint(i)->setForceLowerLimit(0, -forceLimit);
    }
  }

  // Wrap a WorldNode around the world and robot
  ismpc::WalkEngine walk_engine = ismpc::WalkEngine();
  osg::ref_ptr<ismpc::WorldNode> node = new ismpc::WorldNode(world, robot, walk_engine);
  node->setNumStepsPerCycle(1);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);
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
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3d(0.8, -8.2 * 3.28 * 0.2, 3.3 * 0.155) * 1.0,
                                                 ::osg::Vec3d(-0.10, 2.5, 0.35), ::osg::Vec3d(0.00, 0.2, 2.1));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Run the simulation!
  viewer.run();
}
