#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footsteps.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/lip_robot.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/math/arithmetic.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"
#include "ismpc_cpp/types/walk_state.h"

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
