#include "ismpc_cpp/representations/footstep_plan.h"

#include "ismpc_cpp/types/body_parts.h"

namespace ismpc {

FootstepPlan::FootstepPlan(const Params& params) : numP(params.mpc.P), numC(params.mpc.C) {
    zmp_midpoints_x = VectorX::Zero(numC);
    zmp_midpoints_y = VectorX::Zero(numC);
    zmp_midpoints_theta = VectorX::Zero(numC);
}

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
