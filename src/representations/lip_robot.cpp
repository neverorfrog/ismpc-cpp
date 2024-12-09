#include "representations/lip_robot.h"

namespace ismpc {

int LipRobot::getFootstepSign(int j) const {
    int starting_sign = walk.support_foot_type == Foot::right ? 1 : -1;
    return starting_sign * pow(-1, j);
}

EndEffector& LipRobot::getSupportFoot() {
    return walk.support_foot_type == Foot::right ? state.right_foot : state.left_foot;
}

const EndEffector& LipRobot::getSupportFoot() const {
    return walk.support_foot_type == Foot::right ? state.right_foot : state.left_foot;
}

EndEffector& LipRobot::getSwingFoot() {
    return walk.support_foot_type == Foot::right ? state.left_foot : state.right_foot;
}

const EndEffector& LipRobot::getSwingFoot() const {
    return walk.support_foot_type == Foot::right ? state.left_foot : state.right_foot;
}

const Pose3& LipRobot::getSupportFootPose() const {
    return getSupportFoot().pose;
}

const Pose3& LipRobot::getSwingFootPose() const {
    return getSwingFoot().pose;
}

Pose3 LipRobot::getRelComPose() const {
    return state.com.pose.relativeTo(getSupportFootPose());
}

Pose3 LipRobot::getRelSwingFootPose() const {
    return getSwingFootPose().relativeTo(getSupportFootPose());
}

std::string LipRobot::toString() const {
    std::ostringstream oss;
    oss << "State: \n" << state << std::endl;
    oss << "Walk State: \n" << walk << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const LipRobot& robot) {
    os << robot.toString();
    return os;
}

}  // namespace ismpc
