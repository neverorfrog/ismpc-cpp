#pragma once

#include <string>
#include <vector>

#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"
#include "ismpc_cpp/types/footstep.h"

namespace ismpc {

class FootstepPlan {
   public:
    std::vector<Footstep> footsteps;
    

    std::vector<int> footstep_indices;
    std::vector<SupportPhase> support_phases;
    Matrix zmp_midpoints_x = VectorX::Zero(Config::P);
    Matrix zmp_midpoints_y = VectorX::Zero(Config::P);

    Scalar total_planner_qp_duration;

    std::string toString() const;
    friend std::ostream& operator<<(std::ostream& os, const FootstepPlan& footsteps);
};

}  // namespace ismpc
