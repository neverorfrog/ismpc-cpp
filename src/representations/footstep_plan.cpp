#include "ismpc_cpp/representations/footstep_plan.h"

namespace ismpc {

std::string FootstepPlan::toString() const {
    std::ostringstream os;
    os << "FootstepPlan: " << std::endl;
    return os.str();
}

std::ostream& operator<<(std::ostream& os, const FootstepPlan& footsteps) {
    os << footsteps.toString();
    return os;
}

}  // namespace ismpc
