#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/optimization.h"

namespace ismpc {
/**
 * @brief class
 */
class MovingConstraintProvider {
   private:
    const FrameInfo& frame_info;
    const State& state;
    const FootstepPlan& plan;

    // Parameters
    const int numP = Config::P;  // number of planning points
    const int numC = Config::C;  // number of control points

    Vector3 initial_lf_pos = Vector3::Zero();
    Vector3 initial_rf_pos = Vector3::Zero();

    VectorX sigmaFunction(VectorX time, Scalar t0, Scalar t1) const;

   public:
    MovingConstraintProvider(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan);

    /**
     * @brief Compute the ZMP midpoints for the moving constraint. The goal
     * is to have the ZMP midpoints passing from one foot to another in the
     * double support phase and to be on the support foot in the single
     * support phase. Thus,
     */
    void update(FootstepPlan& plan);
};

}  // namespace ismpc
