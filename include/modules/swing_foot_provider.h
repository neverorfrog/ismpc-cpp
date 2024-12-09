#pragma once

#include <cmath>

#include "representations/footsteps.h"
#include "representations/frame_info.h"
#include "representations/lip_robot.h"
#include "tools/config/robot_config.h"
#include "tools/math/arithmetic.h"
#include "types/math_types.h"
#include "types/support_phase.h"
#include "types/walk_state.h"

using namespace ismpc::Arithmetic;
namespace ismpc {

class SwingFootProvider {
   private:
    const FrameInfo& frame_info;
    const LipRobot& robot;
    const FootstepsPlan& footsteps;

    const Scalar step_height = RobotConfig::step_height;
    const Scalar ds_percentage = RobotConfig::ds_percentage;
    const Scalar ss_percentage = RobotConfig::ss_percentage;

   public:
    SwingFootProvider(const FrameInfo& frame_info, const LipRobot& robot, const FootstepsPlan& footsteps);

    /**
     * @brief
     *
     * @param robot
     */
    void update(LipRobot& robot);
};

}  // namespace ismpc
