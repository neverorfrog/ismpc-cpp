#include "ismpc_cpp/libraries/feet_lib.h"

#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/types/end_effector.h"

namespace ismpc {

FeetLib::FeetLib(const State& state, const WalkState& walk) : state(state), walk(walk) {}

int FeetLib::getFootstepSign(int j) const {
    int starting_sign = walk.support_foot_type == Foot::right ? 1 : -1;
    return starting_sign * pow(-1, j);
}

const EndEffector& FeetLib::getSupportFoot() const {
    return walk.support_foot_type == Foot::right ? state.right_foot : state.left_foot;
}

const EndEffector& FeetLib::getSwingFoot() const {
    return walk.support_foot_type == Foot::right ? state.left_foot : state.right_foot;
}

EndEffector FeetLib::getSwingFoot() {
    return walk.support_foot_type == Foot::right ? state.left_foot : state.right_foot;
}

void FeetLib::setSwingFoot(const EndEffector& swing_foot, State& state) const {
    if (walk.support_foot_type == Foot::right) {
        state.left_foot = swing_foot;
    } else {
        state.right_foot = swing_foot;
    }
}

const Pose3& FeetLib::getSupportFootPose() const {
    return getSupportFoot().pose;
}

const Pose3& FeetLib::getSwingFootPose() const {
    return getSwingFoot().pose;
}

Pose3 FeetLib::getRelComPose() const {
    return state.com.pose.relativeTo(getSupportFootPose());
}

Pose3 FeetLib::getRelSwingFootPose() const {
    return getSwingFootPose().relativeTo(getSupportFootPose());
}

}  // namespace ismpc
