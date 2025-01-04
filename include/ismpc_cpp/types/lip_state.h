#pragma once

#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

struct LipState {
    Vector3 com_pos{};
    Vector3 com_vel{};
    Vector3 com_acc{};
    Vector3 zmp_pos{};
    Vector3 zmp_vel{};

    LipState();

    /**
     * @brief Get the LIP state in the x-direction
     *
     * @return Vector3
     */
    Vector3 getX() const;

    /**
     * @brief Get the LIP state in the y-direction
     *
     * @return Vector3
     */
    Vector3 getY() const;

    Vector6 getState() const;

    /**
     * @brief Integrate the LIP state in the x-direction
     *
     * @param xdz The velocity in the x-direction
     * @return Vector3
     */
    Vector3 integrateX(Scalar xdz) const;

    /**
     * @brief Integrate the LIP state in the y-direction
     *
     * @param ydz The velocity in the y-direction
     * @return Vector3
     */
    Vector3 integrateY(Scalar ydz) const;

    /**
     * @brief Integrate the LIP state in both x and y directions
     */
    Vector6 integrate(Scalar xdz, Scalar ydz) const;

    std::string toString() const;

    friend std::ostream& operator<<(std::ostream& os, const LipState& lip_state);

    // Lip parameters
    static Scalar cosh;
    static Scalar sinh;
    static Matrix3 A;
    static Vector3 B;
};

}  // namespace ismpc