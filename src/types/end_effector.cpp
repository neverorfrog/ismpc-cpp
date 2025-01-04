#include "ismpc_cpp/types/end_effector.h"

namespace ismpc {

EndEffector::EndEffector(const Vector3& translation) : pose(translation) {}

Vector6 EndEffector::getVelocity() const {
    Vector6 vel{};
    vel << lin_vel, ang_vel;
    return vel;
}

Vector6 EndEffector::getAcceleration() const {
    Vector6 acc{};
    acc << lin_acc, ang_acc;
    return acc;
}

Pose2 EndEffector::getPose2() const {
    return pose.getPose2();
}

std::string EndEffector::toString() const {
    std::ostringstream oss;
    oss << "\n" << pose << std::endl;
    oss << "Linear Velocity: " << lin_vel.transpose() << std::endl;
    oss << "Angular Velocity: " << ang_vel.transpose() << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const EndEffector& ee) {
    os << ee.toString();
    return os;
}

}  // namespace ismpc
