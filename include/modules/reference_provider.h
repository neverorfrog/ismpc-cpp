#pragma once

#include "representations/footsteps.h"
#include "representations/frame_info.h"
#include "representations/lip_robot.h"
#include "representations/reference.h"
#include "tools/config/config.h"
#include "tools/math/rotation_matrix.h"
#include "types/math_types.h"

namespace ismpc {

class ReferenceProvider {
   private:
    const LipRobot& robot;
    const FrameInfo& frame_info;

   public:
    ReferenceProvider(const FrameInfo& frame_info, const LipRobot& robot);

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
