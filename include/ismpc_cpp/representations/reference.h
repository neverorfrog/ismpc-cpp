#pragma once

#include <cmath>

#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

struct Velocity {
    Scalar vx;
    Scalar vy;
    Scalar omega;
    Vector3 vector{};
};

struct Reference {
    Velocity velocity;

    Reference();

    /**
     * @brief Get the reference input velocity
     * @return Reference velocity
     */
    Velocity get_velocity() const;

    /**
     * @brief Get the current reference velocity in module
     * @return Scalar
     */
    Scalar getVelocityModule() const;

    /**
     * @brief Set the reference input velocity
     */
    void set_velocity(Scalar vx, Scalar vy, Scalar omega);

    /**
     * @brief Integrate the reference angular velocity over a time interval [start, end]
     * @param start
     * @param end
     * @return Scalar
     */
    Scalar integrateOmega(Scalar start, Scalar end) const;

    /**
     * @brief Computes the displacement of the reference velocity
     * by integrating on a time interval [start, end] over timesteps of size delta
     * @param start
     * @param end
     * @param current_theta
     * @return Pose
     */
    Pose2 integrateVelocity(Scalar start, Scalar end, Scalar current_theta) const;
};

}  // namespace ismpc
