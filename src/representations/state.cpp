#include "ismpc_cpp/representations/state.h"

#include "ismpc_cpp/tools/math/rotation_matrix.h"

namespace ismpc {

State::State() {
    left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    right_foot.pose.translation << right_foot_x, right_foot_y, 0;
    desired_left_foot.pose.translation << left_foot_x, left_foot_y, 0;
    desired_right_foot.pose.translation << right_foot_x, right_foot_y, 0;
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
