#pragma once

#include <chrono>

#include "ismpc_cpp/modules/foot_trajectory_generator.h"
#include "ismpc_cpp/modules/footstep_plan_provider.h"
#include "ismpc_cpp/modules/model_predictive_controller.h"
#include "ismpc_cpp/modules/reference_provider.h"
#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/debug.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class WalkEngine {
   private:
    const Scalar delta = Config::delta;

    // Shared representations (not owned by me)
    const State& state;

    // My representations
    FrameInfo frame_info;
    Reference reference;
    FootstepPlan plan;
    std::vector<State> history;

    // Modules
    FootstepPlanProvider planner;
    FootTrajectoryGenerator generator;
    ModelPredictiveController mpc;

   public:
    WalkEngine(const State& state);

    /**
     * @brief Updates the whole walk engine every delta seconds. Plans the footsteps and computes the
     * desired zmp velocity to be tracked by the com through the intrinsically stable mpc algorithm
     *
     */
    void update(State& state);

    /**
     * @brief Updates time information (needed for the python binding)
     *
     */
    void update_time();

    // Getters
    const FootstepPlan& get_footsteps() const;
    const Reference& get_reference() const;
    const FrameInfo& get_frame_info() const;
    const State& get_state() const;
    const std::vector<State>& get_history() const;

    // Timing
    Scalar total_planner_duration = 0.0;
    Scalar total_mpc_duration = 0.0;
};

}  // namespace ismpc
