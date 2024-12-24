#pragma once

#include <cmath>

#include "ismpc_cpp/libraries/feet_lib.h"
#include "ismpc_cpp/representations/footsteps.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/representations/walk_state.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/math/arithmetic.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

using namespace ismpc::Arithmetic;
namespace ismpc {

class SwingFootProvider {
   private:
    const FrameInfo& frame_info;
    const WalkState& walk;
    const FeetLib& feet;
    const FootstepsPlan& footsteps;

    const Scalar step_height = RobotConfig::step_height;
    const Scalar ds_percentage = RobotConfig::ds_percentage;
    const Scalar ss_percentage = RobotConfig::ss_percentage;

   public:
    SwingFootProvider(const FrameInfo& frame_info, const WalkState& walk, const FeetLib& feet,
                      const FootstepsPlan& footsteps);

    /**
     * @brief
     *
     * @param robot
     */
    void update(State& state);
};

}  // namespace ismpc
