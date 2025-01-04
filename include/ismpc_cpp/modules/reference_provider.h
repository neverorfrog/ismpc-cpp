#pragma once

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/math/rotation_matrix.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class ReferenceProvider {
   private:
    const State& state;
    const FrameInfo& frame_info;

   public:
    ReferenceProvider(const FrameInfo& frame_info, const State& state);

    /**
     * @brief Get the reference trajectory starting from the current time tk
     * and the current robot state
     *
     * @param tk The current time at step k
     */
    void update(Reference& reference);

    /**
     * @brief Template Model for reference trajectory generation
     *
     * @param x Vector of variables [x, y, theta]
     * @param u Desired velocity [vx, vy, omega]
     *
     * @return Matrix of reference trajectory
     */
    Matrix f(const Matrix& x, const Matrix& u);
};

}  // namespace ismpc
