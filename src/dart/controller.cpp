#include "ismpc_cpp/dart/controller.h"

namespace ismpc {

Controller::Controller(const dart::simulation::WorldPtr world, const dart::dynamics::SkeletonPtr skeleton)
    : dart::gui::osg::WorldNode(world), world(world), walk_engine(state), state_provider(), kalman_filter(robot) {
    robot = SimulatedRobot(skeleton);
    world->setTimeStep(Config::delta);
    com = registerBall("com", ismpc::Config::RED);
    zmp = registerBall("zmp", ismpc::Config::GREEN);
}

void Controller::customPreStep() {
    auto start = std::chrono::high_resolution_clock::now();

    // Update the state of the robot
    state_provider.update(robot);  // Reading sensors
    kalman_filter.update(state);   // Filtering the state

    std::cout << std::endl << "--------------------------------" << std::endl;
    std::cout << "CURRENT STATE" << std::endl;
    std::cout << "COM POS: " << state.lip.com_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM VEL: " << state.lip.com_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM ACC: " << state.lip.com_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP POS: " << state.lip.zmp_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP VEL: " << state.lip.zmp_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "LFoot Pose: " << state.left_foot.pose.getVector().transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "RFoot Pose: " << state.right_foot.pose.getVector().transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Left Foot Vel: " << state.left_foot.lin_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Right Foot Vel: " << state.right_foot.lin_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Acc: " << state.left_foot.lin_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Right Foot Acc: " << state.right_foot.lin_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "" << std::endl;

    // Step of MPC
    walk_engine.update(state);  // This is the main function of the walk engine

    std::cout << "DESIRED STATE" << std::endl;
    std::cout << "COM POS: " << state.desired_lip.com_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM VEL: " << state.desired_lip.com_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM ACC: " << state.desired_lip.com_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP POS: " << state.desired_lip.zmp_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP VEL: " << state.desired_lip.zmp_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Pose: "
              << state.desired_left_foot.pose.getVector().transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Right Foot Pose: "
              << state.desired_right_foot.pose.getVector().transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Vel: " << state.desired_left_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Vel: " << state.desired_right_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Left Foot Acc: " << state.desired_left_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Acc: " << state.desired_right_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Get and send the joint request
    VectorX joint_request = robot.getJointRequest(state);
    for (size_t i = 0; i < robot.skeleton->getNumDofs() - 6; i++) {
        robot.skeleton->setCommand(i + 6, joint_request(i));
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
