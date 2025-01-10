#include "ismpc_cpp/representations/state.h"

namespace ismpc {

State::State() {
    left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    right_foot.pose.translation << right_foot_x, right_foot_y, 0;

    // Walk Initialization
    footstep.start_pose = right_foot.getPose2();
    footstep.end_pose = right_foot.getPose2();
    footstep.walk_phase = WalkPhase::STARTING;
    footstep.support_foot = Foot::right;
    footstep.start = 0;
    footstep.ds_start = 0;
    footstep.end = 0.5;  // DEFAULT VALUE

    previous_sf_pose = left_foot.getPose2();
    previous_support_foot = Foot::left;
    support_phase = SupportPhase::DOUBLE;
}

int State::getFootstepSign(int j) const {
    int starting_sign = footstep.support_foot == Foot::right ? 1 : -1;
    return starting_sign * pow(-1, j);
}

const EndEffector& State::getSupportFoot() const {
    return footstep.support_foot == Foot::right ? right_foot : left_foot;
}

const EndEffector& State::getSwingFoot() const {
    return footstep.support_foot == Foot::right ? left_foot : right_foot;
}

void State::setDesiredSwingFoot(const EndEffector& foot) {
    if (footstep.support_foot == Foot::right) {
        desired_right_foot = foot;
    } else {
        desired_left_foot = foot;
    }
}

std::string State::toString() const {
    std::ostringstream oss;
    oss << "Left Foot: \n" << left_foot.toString() << std::endl;
    oss << "Right Foot: \n" << right_foot.toString() << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const State& state) {
    os << state.toString();
    return os;
}

}  // namespace ismpc
