#pragma once

#include <iostream>
#include <sstream>

#include "ismpc_cpp/tools/math/pose3.h"

namespace ismpc {

/**
 * This struct is designed to encapsulate key kinematic information and orientation
 * data for a specific part of a robot, such as a foot or the torso. Positional information
 * pertains to the center of mass (CoM) of the body part.
 */
class EndEffector {
   public:
    Pose3 pose{};
    Vector3 lin_vel{0, 0, 0};
    Vector3 ang_vel{0, 0, 0};
    Vector3 lin_acc{0, 0, 0};
    Vector3 ang_acc{0, 0, 0};

    EndEffector() = default;
    EndEffector(const Vector3& translation);

    /**
     * @brief Get linear and angular velocity stacked in a Vector6
     */
    Vector6 getVelocity() const;

    /**
     * @brief Get linear and angular acceleration stacked in a Vector6
     */
    Vector6 getAcceleration() const;

    std::string toString() const;

    /**
     * @brief
     *
     * @param os The output stream.
     * @param ee The EndEffector object to be printed.
     * @return std::ostream& The output stream.
     */
    friend std::ostream& operator<<(std::ostream& os, const EndEffector& ee);
};

}  // namespace ismpc
