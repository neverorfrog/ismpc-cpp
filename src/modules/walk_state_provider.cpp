#include "ismpc_cpp/modules/walk_state_provider.h"

namespace ismpc {

WalkStateProvider::WalkStateProvider(const FrameInfo& frame_info, const State& state, const WalkState& walk,
                                     const FeetLib& feet, const FootstepsPlan& footsteps)
    : frame_info(frame_info), state(state), walk(walk), feet(feet), footsteps(footsteps) {}

void WalkStateProvider::update(WalkState& walk) {
    walk.next_footstep_timestamp = footsteps.timestamps[0];
    walk.next_support_foot_pose = Pose2(footsteps.theta(0), footsteps.x(0), footsteps.y(0));

    // Switch support foot at the start of a new double support phase
    if (frame_info.tk >= footsteps.timestamps[0] && walk.support_phase == SupportPhase::SINGLE) {
        switchSupportFoot(walk);
        walk.footstep_history.push_back(feet.getSupportFootPose().getPose2());
        walk.timestamp_history.push_back(frame_info.tk);
    }

    // Update the support phase info
    Scalar ds_duration = (walk.next_footstep_timestamp - walk.current_footstep_timestamp) * ds_percentage;
    if (frame_info.tk >= walk.current_footstep_timestamp + ds_duration)
        walk.support_phase = SupportPhase::SINGLE;
    else
        walk.support_phase = SupportPhase::DOUBLE;
}

void WalkStateProvider::switchSupportFoot(WalkState& walk) {
    walk.previous_support_foot_pose = feet.getSupportFootPose().getPose2();
    walk.next_support_foot_pose = Pose2(footsteps.theta(1), footsteps.x(1), footsteps.y(1));

    walk.support_foot_type = walk.support_foot_type == Foot::right ? Foot::left : Foot::right;

    walk.current_footstep_timestamp = frame_info.tk;
    walk.next_footstep_timestamp = footsteps.timestamps[1];
}

}  // namespace ismpc
