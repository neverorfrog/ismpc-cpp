#pragma once

#include <string>
#include <vector>

#include "ismpc_cpp/types/configs.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

namespace ismpc {

class FootstepPlan {
   public:
    FootstepPlan(const Params& params);

    // Stuff update only when replanning
    std::vector<Footstep> footsteps{};
    bool need_to_replan = true;

    // Stuff updated every instant
    SupportPhase support_phase = SupportPhase::DOUBLE;
    Matrix zmp_midpoints_x;
    Matrix zmp_midpoints_y;
    Matrix zmp_midpoints_theta;

    // Debugging stuff
    Scalar total_planner_qp_duration;
    std::vector<VectorX> mc_x_history;
    std::vector<VectorX> mc_y_history;
    std::vector<VectorX> mc_theta_history;
    std::vector<Footstep> fs_history;
    std::vector<std::vector<Footstep>> fs_plan_history;

    std::string toString() const;
    friend std::ostream& operator<<(std::ostream& os, const FootstepPlan& footsteps);

   private:
    // Parameters
    int numP;    // number of planning points
    int numC;    // number of control points
    Scalar T_p;  // planning time
    Scalar T_c;  // control time
};

}  // namespace ismpc
