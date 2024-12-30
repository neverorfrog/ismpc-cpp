#pragma once

#include <vector>

#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/representations/walk_state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/types/end_effector.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class FeetLib {
   private:
    const State& state;
    const WalkState& walk;

   public:
    FeetLib(const State& state, const WalkState& walk);

    /**
     * @brief Compute the sign corresponding to the footstep index
     * @param j
     * @return int
     */
    int getFootstepSign(int j) const;

    /**
     * @brief Get the Support Foot Pose object
     */
    const EndEffector& getSupportFoot() const;

    /**
     * @brief Get the Swing Foot Pose object
     */
    const EndEffector& getSwingFoot() const;

    /**
     * @brief Get the Swing Foot object
     * @return EndEffector
     */
    EndEffector getSwingFoot();

    /**
     * @brief Set the Swing Foot object
     * @param swing_foot
     */
    void setSwingFoot(const EndEffector& swing_foot, State& state) const;

    /**
     * @brief Get the Support Foot Pose object
     * @return const Pose3&
     */
    const Pose3& getSupportFootPose() const;

    /**
     * @brief Get the Swing Foot Pose object
     * @return const Pose3&
     */
    const Pose3& getSwingFootPose() const;

};

}  // namespace ismpc
