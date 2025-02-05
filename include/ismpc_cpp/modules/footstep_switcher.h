/**
 * @file FootstepPlanProvider.hpp
 * @brief Header file for the FootstepPlanProvider class.
 *
 * This file contains the declaration of the FootstepPlanProvider module. This module uses
 * trajectory optimization to generate the footstep candidates. These in turn
 * are sent to the Footsteps representation. This representation is then
 * accessed by the ISMPC module.
 */

#pragma once

#include <cmath>
#include <iostream>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/end_effector.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/optimization.h"

namespace ismpc {

/**
 * @class FootstepSwitcher
 */
class FootstepSwitcher {
   private:
    const FrameInfo& frame_info;
    const State& state;
    const FootstepPlan& plan;

    // Internal Stuff
    Scalar last_switch_timestamp = 0.0;
    bool phase_switched = false;

   public:
    FootstepSwitcher(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan);

    /**
     * Switches the footstep
     */
    void update(State& state);
};

}  // namespace ismpc
