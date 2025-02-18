#include "ismpc_cpp/modules/footstep_switcher.h"

#include "ismpc_cpp/types/support_phase.h"

namespace ismpc {

FootstepSwitcher::FootstepSwitcher(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void FootstepSwitcher::update(State& state) {
    // Update support phase
    if (frame_info.tk >= state.footstep.ds_start && frame_info.tk <= state.footstep.end && !phase_switched) {
        phase_switched = true;
        state.support_phase = SupportPhase::DOUBLE;
    }

    // Switch support foot when the double support phase ends
    if (frame_info.tk >= state.footstep.end && state.support_phase == SupportPhase::DOUBLE) {
        phase_switched = false;
        state.support_phase = SupportPhase::SINGLE;
        state.footstep = plan.footsteps[1];
        state.footstep.start_pose = state.getSwingFoot().getPose2();
        state.fs_history.push_back(state.footstep);
    }

    // For debugging purposes
    state.mc_x_history.push_back(plan.zmp_midpoints_x);
    state.mc_y_history.push_back(plan.zmp_midpoints_y);
    state.mc_theta_history.push_back(plan.zmp_midpoints_theta);
}

}  // namespace ismpc
