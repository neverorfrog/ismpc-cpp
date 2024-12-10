#pragma once

#include <chrono>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "tools/config/config.h"
#include "modules/walk_engine.h"
#include "representations/lip_robot.h"
#include "simulation/simulated_robot.h"
#include "tools/debug.h"
#include "types/math_types.h"

namespace ismpc {

class WorldNode : public dart::gui::osg::WorldNode {
 private:
  dart::dynamics::SimpleFramePtr com, desired_com, zmp;
  WalkEngine& engine;
  SimulatedRobot simrobot;
  LipRobot& robot;

  /**
   * @brief Get a pointer to a ball in world (change its coordinates in draw method)
   *
   * @param name
   * @param color
   * @return dart::dynamics::SimpleFramePtr
   */
  dart::dynamics::SimpleFramePtr registerBall(const std::string& name, const Eigen::Vector3d& color);

  /**
   * @brief draw the com, desired_com and zmp
   *
   */
  void draw();

 public:
  WorldNode(dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr robot, ismpc::WalkEngine& engine);

  void customPreStep() override;

  dart::simulation::WorldPtr get_world();
  ismpc::WalkEngine& get_walk_engine();

  Scalar total_elapsed_time = 0.0;

 protected:
  dart::simulation::WorldPtr world;
};

}  // namespace ismpc
