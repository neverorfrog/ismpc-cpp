#include "types/end_effector.h"

namespace ismpc {

EndEffector::EndEffector(const Vector3& translation) : pose(translation) {}

std::string EndEffector::toString() const {
    std::ostringstream oss;
    oss << "Pose: \n" << pose << std::endl;
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const EndEffector& ee) {
    os << ee.toString();
    return os;
}

}  // namespace ismpc
