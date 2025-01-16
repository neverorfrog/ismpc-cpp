#pragma once

#include <chrono>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "ismpc_cpp/modules/kalman_filter.h"
#include "ismpc_cpp/dart/simulated_robot.h"
#include "ismpc_cpp/dart/state_provider.h"
#include "ismpc_cpp/modules/casadi_mpc.h"
#include "ismpc_cpp/modules/foot_trajectory_generator.h"
#include "ismpc_cpp/modules/footstep_plan_provider.h"
#include "ismpc_cpp/modules/model_predictive_controller.h"
#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/debug.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class Controller : public dart::gui::osg::WorldNode {
   private:
    // walkengine representations
    State state;
    FootstepPlan plan;
    FrameInfo frame_info;
    Reference reference;

    // dart representations
    SimulatedRobot robot;
    dart::simulation::WorldPtr world;

    // modules
    FootstepPlanProvider planner;
    ModelPredictiveController mpc;
    MovingConstraintProvider mc_provider;
    FootTrajectoryGenerator ft_generator;
    CasadiMPC casadi_mpc;
    StateProvider state_provider;
    KalmanFilter kalman_filter;

    // Stuff for drawing
    dart::dynamics::SimpleFramePtr com, desired_com, zmp;

    /**
     * @brief Get a pointer to a ball in world (change its coordinates in draw method)
     *
     * @param name
     * @param color
     * @return dart::dynamics::SimpleFramePtr
     */
    dart::dynamics::SimpleFramePtr registerBall(const std::string& name, const Vector3& color);

   public:
    Controller(dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr robot);

    /**
     * @brief Custom pre step function for the simulation. This is where
     * the magic happens. This function is called at every simulation step.
     */
    void customPreStep() override;

    /**
     * @brief Get the world object
     *
     * @return dart::simulation::WorldPtr
     */
    dart::simulation::WorldPtr get_world();

    // Time related stuff
    Scalar total_elapsed_time = 0;
};

}  // namespace ismpc
