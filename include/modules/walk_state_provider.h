#pragma once

#include <cmath>

#include "representations/footsteps.h"
#include "representations/frame_info.h"
#include "representations/lip_robot.h"
#include "tools/config/robot_config.h"
#include "types/body_parts.h"
#include "types/math_types.h"
#include "types/walk_state.h"

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
