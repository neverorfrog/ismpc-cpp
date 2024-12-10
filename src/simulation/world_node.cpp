#include "simulation/world_node.h"

namespace ismpc {

WorldNode::WorldNode(const dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr dart_robot,
                     ismpc::WalkEngine& engine)
    : dart::gui::osg::WorldNode(world),
      world(world),
      simrobot(dart_robot),
      engine(engine),
      robot(engine.get_robot()) {
  com = registerBall("com", ismpc::Config::RED);
  zmp = registerBall("zmp", ismpc::Config::GREEN);
}

void WorldNode::customPreStep() {
  auto start = std::chrono::high_resolution_clock::now();

  if (world->getSimFrames() == Config::W + Config::N) {
    std::cout << "ENDING SIMULATION" << std::endl;
    std::cout << "Average elapsed time: " << total_elapsed_time / (Config::W + Config::N) << " microseconds"
              << std::endl;
    return;
  }

  if (world->getSimFrames() > Config::W + Config::N) {
    return;
  }

  if (world->getSimFrames() == Config::W) {
    std::cout << "STARTING WALKING" << std::endl;
    simrobot.update_state(robot, engine.get_frame_info());
    engine.update();
  }

  if (world->getSimFrames() >= Config::W) {
    PRINT("");
    PRINT("------- k: " << engine.get_frame_info().k << "  tk: " << engine.get_frame_info().tk << " -------");

    simrobot.update_state(robot, engine.get_frame_info());
    engine.update();

    PRINT("CURRENT LIP: \n" << robot.state);
    PRINT("NEXT FOOTSTEP: " << engine.get_footsteps().x(0) << " " << engine.get_footsteps().y(0) << " "
                            << engine.get_footsteps().theta(0));
    PRINT("WALK STATE: \n" << robot.walk);

    simrobot.control(robot, world);
    draw();
    PRINT("---------------------------------");
  }

  auto end = std::chrono::high_resolution_clock::now();
  total_elapsed_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

dart::simulation::WorldPtr WorldNode::get_world() {
  return world;
}

ismpc::WalkEngine& WorldNode::get_walk_engine() {
  return engine;
}

dart::dynamics::SimpleFramePtr WorldNode::registerBall(const std::string& name, const Eigen::Vector3d& color) {
  dart::dynamics::SimpleFramePtr ball_frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), name, Eigen::Isometry3d::Identity());

  ball_frame->setShape(std::make_shared<dart::dynamics::EllipsoidShape>(.2 * Eigen::Vector3d::Ones()));
  ball_frame->createVisualAspect();
  ball_frame->getVisualAspect()->setColor(color);
  world->addSimpleFrame(ball_frame);

  return ball_frame;
}

void WorldNode::draw() {
  com->setTranslation(simrobot.state.com.pose.translation);
  zmp->setTranslation(simrobot.state.zmp_pos);
}

}  // namespace ismpc
