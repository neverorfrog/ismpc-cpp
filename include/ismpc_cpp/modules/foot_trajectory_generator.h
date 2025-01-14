#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/math/arithmetic.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

using namespace ismpc::Arithmetic;
namespace ismpc {

class FootTrajectoryGenerator {
   private:
    const FrameInfo& frame_info;
    const State& state;
    const FootstepPlan& plan;

    const Scalar step_height = RobotConfig::step_height;
    const Scalar ds_percentage = RobotConfig::ds_percentage;
    const Scalar ss_percentage = RobotConfig::ss_percentage;

   public:
    FootTrajectoryGenerator(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan);

    /**
     * @brief
     *
     * @param robot
     */
    void update(State& state);
};

}  // namespace ismpc
