#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footsteps.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/lip_robot.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/walk_state.h"

namespace ismpc {

class WalkStateProvider {
   private:
    const FrameInfo& frame_info;
    const LipRobot& robot;
    const FootstepsPlan& footsteps;

    const Scalar ds_percentage = RobotConfig::ds_percentage;

    /**
     * @brief Switch to a new support foot at the start of a new double support phase
     *
     * @param robot
     */
    void switchSupportFoot(LipRobot& robot);

   public:
    WalkStateProvider(const FrameInfo& frame_info, const LipRobot& robot, const FootstepsPlan& footsteps);

    /**
     * @brief
     *
     * @param robot
     */
    void update(LipRobot& robot);
};

}  // namespace ismpc
