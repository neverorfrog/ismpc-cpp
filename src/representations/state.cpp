#include "ismpc_cpp/representations/state.h"

namespace ismpc {

State::State(const Params& params)
    : lip(LipState(params)),
      desired_lip(LipState(params)),
      left_foot_x(params.initial_feet.lf_x),
      left_foot_y(params.initial_feet.lf_y),
      right_foot_x(params.initial_feet.rf_x),
      right_foot_y(params.initial_feet.rf_y) {
    left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    right_foot.pose.translation << right_foot_x, right_foot_y, 0;
    desired_left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    desired_right_foot.pose.translation << right_foot_x, right_foot_y, 0;
}

State::State(const State& state)
    : lip(state.lip),
      left_foot(state.left_foot),
      right_foot(state.right_foot),
      torso(state.torso),
      base(state.base),
      desired_lip(state.desired_lip),
      desired_left_foot(state.desired_left_foot),
      desired_right_foot(state.desired_right_foot),
      desired_torso(state.desired_torso),
      desired_base(state.desired_base),
      left_foot_x(state.left_foot_x),
      left_foot_y(state.left_foot_y),
      right_foot_x(state.right_foot_x),
      right_foot_y(state.right_foot_y) {}

State& State::operator=(const State& state) {
    if (this == &state) {  // Protect against self-assignment
        return *this;
    }
    lip = state.lip;
    left_foot = state.left_foot;
    right_foot = state.right_foot;
    torso = state.torso;
    base = state.base;
    desired_lip = state.desired_lip;
    desired_left_foot = state.desired_left_foot;
    desired_right_foot = state.desired_right_foot;
    desired_torso = state.desired_torso;
    desired_base = state.desired_base;
    left_foot_x = state.left_foot_x;
    left_foot_y = state.left_foot_y;
    right_foot_x = state.right_foot_x;
    right_foot_y = state.right_foot_y;

    return *this;
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
