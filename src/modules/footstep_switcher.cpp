#include "ismpc_cpp/modules/footstep_switcher.h"

#include "ismpc_cpp/types/support_phase.h"

namespace ismpc {

FootstepSwitcher::FootstepSwitcher(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void FootstepSwitcher::update(State& state) {
    // Update support phase
    if (frame_info.tk >= state.footstep.ds_start && frame_info.tk <= state.footstep.end && !phase_switched) {
        std::cout << "SWITCHING TO DOUBLE SUPPORT AT TIME " << frame_info.tk << std::endl;
        phase_switched = true;
        state.support_phase = SupportPhase::DOUBLE;
    }

    // Switch support foot when the double support phase ends
    if (frame_info.tk >= state.footstep.end && state.support_phase == SupportPhase::DOUBLE) {
        std::cout << "SWITCHING FOOT AT TIME " << frame_info.tk << std::endl;

        phase_switched = false;
        state.support_phase = SupportPhase::SINGLE;
        state.footstep = plan.footsteps[1];
        state.footstep.start_pose = state.getSwingFoot().getPose2();
        state.fs_history.push_back(state.footstep);

        // TODO TEST FORCED FOOTSTEP
        // state.setSwingFootPose(state.footstep.end_pose);

        // std::cout << "New Footstep: " << state.footstep << std::endl;
        // std::cout << "New Swingfoot Pos: " << state.getSwingFoot().pose.translation.transpose() << std::endl;
        // std::cout << "New Swingfoot Ang: " << state.getSwingFoot().pose.rotation << std::endl;
        // std::cout << "\n\n\n\n" << std::endl;
    }

    // TODO TEST
    // state.lip.zmp_pos = Vector3(plan.zmp_midpoints_x(0), plan.zmp_midpoints_y(0), plan.zmp_midpoints_theta(0));
    // state.lip_history.push_back(state.lip);
}

}  // namespace ismpc
