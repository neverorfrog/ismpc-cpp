#pragma once

#include <cmath>

#include "ismpc_cpp/libraries/feet_lib.h"
#include "ismpc_cpp/representations/footsteps.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/representations/walk_state.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class WalkStateProvider {
   private:
    const FrameInfo& frame_info;
    const State& state;
    const WalkState& walk;
    const FeetLib& feet;
    const FootstepsPlan& footsteps;

    const Scalar ds_percentage = RobotConfig::ds_percentage;

    /**
     * @brief Switch to a new support foot at the start of a new double support phase
     *
     * @param robot
     */
    void switchSupportFoot(WalkState& walk);

   public:
    WalkStateProvider(const FrameInfo& frame_info, const State& state, const WalkState& walk, const FeetLib& feet,
                      const FootstepsPlan& footsteps);

    /**
     * @brief
     *
     * @param robot
     */
    void update(WalkState& walk);
};

}  // namespace ismpc
