#pragma once

#include <cmath>
#include <sstream>

#include "tools/math/pose3.h"
#include "types/body_parts.h"
#include "types/math_types.h"
#include "types/state.h"
#include "types/walk_state.h"

namespace ismpc {

struct LipRobot {
    State state{};
    WalkState walk{};

    Scalar total_mpc_qp_duration = 0.0;
    Scalar total_mpc_preprocessing_duration = 0.0;
    Scalar total_mpc_postprocessing_duration = 0.0;

    LipRobot() = default;

    /**
     * @brief Compute the sign corresponding to the footstep index
     * @param j
     * @return int
     */
    int getFootstepSign(int j) const;

    EndEffector& getSupportFoot();
    const EndEffector& getSupportFoot() const;

    EndEffector& getSwingFoot();
    const EndEffector& getSwingFoot() const;

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

    /**
     * @brief Get the Rel Com Pose
     * @return Pose3
     */
    Pose3 getRelComPose() const;

    /**
     * @brief Get the Rel Swing Foot Pose
     * @return Pose3
     */
    Pose3 getRelSwingFootPose() const;

    /**
     * @brief Convert the object to a string representation
     * @return std::string
     */
    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const LipRobot& robot);
};

}  // namespace ismpc
