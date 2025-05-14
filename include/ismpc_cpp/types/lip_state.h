#pragma once

#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/configs.h"

namespace ismpc {

struct LipState {
    Vector3 com_pos{};
    Vector3 com_vel{};
    Vector3 zmp_pos{};

    Vector3 com_acc{};
    Vector3 zmp_vel{};

    LipState(const Params& params);

    LipState(const LipState& other):
        com_pos(other.com_pos),
        com_vel(other.com_vel),
        zmp_pos(other.zmp_pos),
        com_acc(other.com_acc),
        zmp_vel(other.zmp_vel) {}

    LipState& operator=(const LipState& other) {
        if (this == &other) { // Protect against self-assignment
            return *this;
        }
        com_pos = other.com_pos;
        com_vel = other.com_vel;
        zmp_pos = other.zmp_pos;
        com_acc = other.com_acc;
        zmp_vel = other.zmp_vel;
        return *this;
    }

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
    private:
        Scalar eta = 0.0;
        Scalar delta = 0.0;
        Scalar cosh = 0.0;
        Scalar sinh = 0.0;
        Matrix3 A = Matrix3::Zero();
        Vector3 B = Vector3::Zero();
};

}  // namespace ismpc
