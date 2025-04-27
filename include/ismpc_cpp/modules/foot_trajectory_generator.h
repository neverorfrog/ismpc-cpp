#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/math/arithmetic.h"
#include "ismpc_cpp/types/configs.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

using namespace ismpc::Arithmetic;
namespace ismpc {

class FootTrajectoryGenerator {
   private:
    const FrameInfo& frame_info;
    const State& state;
    const FootstepPlan& plan;
    const Params& params;

    // Parameters
    Scalar step_height;
    Scalar ds_percentage;
    Scalar ss_percentage;

   public:
    FootTrajectoryGenerator(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan,
                            const Params& params)
        : frame_info(frame_info),
          state(state),
          plan(plan),
          params(params),
          step_height(params.gait.step_height),
          ds_percentage(params.gait.ds_percentage),
          ss_percentage(params.gait.ss_percentage) {}

    /**
     * @brief
     *
     * @param robot
     */
    void update(State& state);
};

}  // namespace ismpc
