#pragma once

#include <cmath>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/configs.h"
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
    const int numP;  // number of planning points
    const int numC;  // number of control points
    const Scalar T_c;

    Vector3 initial_lf_pos = Vector3::Zero();
    Vector3 initial_rf_pos = Vector3::Zero();

    VectorX sigmaFunction(VectorX time, Scalar t0, Scalar t1) const;

    Scalar ds_start_time;
    Scalar fs_end_time;
    Scalar start_x, start_y, start_theta;
    Scalar end_x, end_y, end_theta;
    VectorX sigma;

   public:
    MovingConstraintProvider(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan,
                             const Params& params);

    /**
     * @brief Compute the ZMP midpoints for the moving constraint. The goal
     * is to have the ZMP midpoints passing from one foot to another in the
     * double support phase and to be on the support foot in the single
     * support phase. Thus,
     */
    void update(FootstepPlan& plan);
};

}  // namespace ismpc
