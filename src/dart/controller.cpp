#include "ismpc_cpp/dart/controller.h"

namespace ismpc {

Controller::Controller(const dart::simulation::WorldPtr world, const dart::dynamics::SkeletonPtr skeleton)
    : dart::gui::osg::WorldNode(world),
      state(),
      plan(),
      frame_info(),
      reference(),
      world(world),
      planner(frame_info, reference, state, plan),
      mpc(frame_info, state, plan),
      mc_provider(frame_info, state, plan),
      ft_generator(frame_info, state, plan),
      casadi_mpc(frame_info, state, plan),
      state_provider(robot),
      kalman_filter(robot) {
    robot = SimulatedRobot(skeleton);
    world->setTimeStep(Config::delta);
    com = registerBall("com", ismpc::Config::RED);
    zmp = registerBall("zmp", ismpc::Config::GREEN);
    planner.update(plan);
}

void Controller::customPreStep() {
    auto start = std::chrono::high_resolution_clock::now();

    std::cout << std::endl << "--------------------------------" << std::endl;
    // Update the state of the robot
    state_provider.update(state);  // Reading sensors
    kalman_filter.update(state);   // Filtering the state

    state.lip.zmp_pos = state.desired_lip.zmp_pos;

    if (frame_info.k == 0) {
        state.footstep.start_pose.translation(0) = state.right_foot.pose.translation(0);
        state.footstep.end_pose.translation(0) = state.right_foot.pose.translation(0);
        state.desired_right_foot.pose.translation(0) = state.right_foot.pose.translation(0);
    }

    std::cout << "CURRENT STATE" << std::endl;
    std::cout << "COM POS: " << state.lip.com_pos.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "COM VEL: " << state.lip.com_vel.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "COM ACC: " << state.lip.com_acc.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "ZMP POS: " << state.lip.zmp_pos.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "ZMP VEL: " << state.lip.zmp_vel.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "LFoot Pose: " << state.left_foot.pose.getVector().transpose().format(Config::CleanFmt)
              << std::endl;
    std::cout << "RFoot Pose: " << state.right_foot.pose.getVector().transpose().format(Config::CleanFmt)
              << std::endl;
    std::cout << "" << std::endl;

    std::cout << "CURRENT TIME: " << frame_info.tk << std::endl;
    std::cout << "CURRENT PLANNED FOOTSTEP: \n" << state.footstep.toString() << std::endl;

    // Step of MPC
    mc_provider.update(plan);
    casadi_mpc.update(state);
    ft_generator.update(state);

    std::cout << "DESIRED STATE" << std::endl;
    std::cout << "COM POS: " << state.desired_lip.com_pos.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "COM VEL: " << state.desired_lip.com_vel.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "COM ACC: " << state.desired_lip.com_acc.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "ZMP POS: " << state.desired_lip.zmp_pos.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "ZMP VEL: " << state.desired_lip.zmp_vel.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "Left Foot Pose: " << state.desired_left_foot.pose.getVector().transpose().format(Config::CleanFmt)
              << std::endl;
    std::cout << "Right Foot Pose: "
              << state.desired_right_foot.pose.getVector().transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Setting Desired Torso based on the feet
    RotationMatrix desired_torso_orientation =
        RotationMatrix(state.desired_left_foot.pose.rotation.getQuaternion().slerp(
            0.5, state.desired_right_foot.pose.rotation.getQuaternion()));
    Vector3 desired_torso_ang_vel = (state.desired_left_foot.ang_vel + state.desired_right_foot.ang_vel) / 2;
    Vector3 desired_torso_ang_acc = (state.desired_left_foot.ang_acc + state.desired_right_foot.ang_acc) / 2;
    state.desired_torso.pose.rotation = desired_torso_orientation;
    state.desired_torso.ang_vel = desired_torso_ang_vel;
    state.desired_torso.ang_acc = desired_torso_ang_acc;

    // Get and send the joint request
    VectorX joint_request = robot.getJointRequest(state);
    for (size_t i = 0; i < robot.skeleton->getNumDofs() - 6; i++) {
        robot.skeleton->setCommand(i + 6, joint_request(i));
    }

    // Update frame info
    frame_info.k += 1;
    frame_info.tk += Config::delta;

    if (frame_info.k > Config::N)
        exit(0);

    auto end = std::chrono::high_resolution_clock::now();
    total_elapsed_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Average execution time: " << total_elapsed_time / frame_info.k << " microseconds" << std::endl;
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
