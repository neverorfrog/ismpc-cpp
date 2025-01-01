#include "ismpc_cpp/dart/controller.h"

namespace ismpc {

Controller::Controller(const dart::simulation::WorldPtr world, const dart::dynamics::SkeletonPtr skeleton)
    : dart::gui::osg::WorldNode(world), world(world), walk_engine(), state_provider(), kalman_filter(robot) {
    robot = SimulatedRobot(skeleton);
    world->setTimeStep(Config::delta);
    com = registerBall("com", ismpc::Config::RED);
    zmp = registerBall("zmp", ismpc::Config::GREEN);
}

void Controller::customPreStep() {
    auto start = std::chrono::high_resolution_clock::now();

    // Update the state of the robot
    state_provider.update(robot);        // Reading sensors
    kalman_filter.update(robot.state);   // Filtering the state
    walk_engine.set_state(robot.state);  // Setting the state of the walk engine

    std::cout << std::endl << "--------------------------------" << std::endl;
    std::cout << "CURRENT STATE" << std::endl;
    std::cout << "COM POS: " << robot.state.com_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM VEL: " << robot.state.com_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM ACC: " << robot.state.com_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP POS: " << robot.state.zmp_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP VEL: " << robot.state.zmp_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Pose: " << robot.state.left_foot.pose.getVector().transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Pose: "
              << robot.state.right_foot.pose.getVector().transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Vel: " << robot.state.left_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Vel: " << robot.state.right_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Left Foot Acc: " << robot.state.left_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Acc: " << robot.state.right_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "" << std::endl;

    // Step of MPC
    State& desired_state = walk_engine.get_desired_state();
    desired_state = robot.state;
    walk_engine.update(desired_state);  // This is the main function of the walk engine

    std::cout << "DESIRED STATE" << std::endl;
    std::cout << "COM POS: " << desired_state.com_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM VEL: " << desired_state.com_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "COM ACC: " << desired_state.com_acc.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP POS: " << desired_state.zmp_pos.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "ZMP VEL: " << desired_state.zmp_vel.transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Pose: "
              << desired_state.left_foot.pose.getVector().transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Right Foot Pose: "
              << desired_state.right_foot.pose.getVector().transpose().format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "Left Foot Vel: " << desired_state.left_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Vel: " << desired_state.right_foot.lin_vel.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Left Foot Acc: " << desired_state.left_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "Right Foot Acc: " << desired_state.right_foot.lin_acc.transpose().format(Eigen::IOFormat(2))
              << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Get and send the joint request
    VectorX joint_request = robot.getJointRequest(desired_state);
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
