#include "ismpc_cpp/types/end_effector.h"

namespace ismpc {

EndEffector::EndEffector(const Vector3& translation) : pose(translation) {}

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
