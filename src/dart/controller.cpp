#include "ismpc_cpp/dart/controller.h"

namespace ismpc {

Controller::Controller(const dart::simulation::WorldPtr world, const dart::dynamics::SkeletonPtr skeleton)
    : dart::gui::osg::WorldNode(world), world(world) {
    robot = SimulatedRobot(skeleton);
    com = registerBall("com", ismpc::Config::RED);
    zmp = registerBall("zmp", ismpc::Config::GREEN);
}

void Controller::customPreStep() {
    auto start = std::chrono::high_resolution_clock::now();

    if (world->getSimFrames() == Config::W + Config::N) {
        std::cout << "ENDING SIMULATION" << std::endl;
        return;
    }

    if (world->getSimFrames() > Config::W + Config::N) {
        return;
    }

    auto end = std::chrono::high_resolution_clock::now();
    total_elapsed_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

dart::simulation::WorldPtr Controller::get_world() {
    return world;
}

dart::dynamics::SimpleFramePtr Controller::registerBall(const std::string& name, const Eigen::Vector3d& color) {
    dart::dynamics::SimpleFramePtr ball_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), name, Eigen::Isometry3d::Identity());

    ball_frame->setShape(std::make_shared<dart::dynamics::EllipsoidShape>(.2 * Eigen::Vector3d::Ones()));
    ball_frame->createVisualAspect();
    ball_frame->getVisualAspect()->setColor(color);
    world->addSimpleFrame(ball_frame);

    return ball_frame;
}

}  // namespace ismpc
