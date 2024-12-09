#pragma once

#include "tools/config/config.h"
#include "tools/config/robot_config.h"
#include "tools/math/pose2.h"
#include "types/end_effector.h"
#include "types/math_types.h"
#include "types/support_phase.h"
#include "types/walk_state.h"

namespace ismpc {

struct State {
    EndEffector left_foot{};
    EndEffector right_foot{};
    EndEffector com{};
    Vector3 zmp_pos{};
    Vector3 zmp_vel{};

    // LIP Stuff
    Scalar cosh;
    Scalar sinh;
    Matrix A{3, 3};
    Matrix B{3, 1};

    // Variables from config to save permanently
    Scalar eta = RobotConfig::eta;
    Scalar delta = Config::delta;
    Scalar left_foot_x = RobotConfig::left_foot_x;
    Scalar left_foot_y = RobotConfig::left_foot_y;
    Scalar right_foot_x = RobotConfig::right_foot_x;
    Scalar right_foot_y = RobotConfig::right_foot_y;
    Scalar h = RobotConfig::h;

    State();

    /**
     * @brief Get the LIP state in the x-direction
     *
     * @return const Vector3
     */
    const Vector3 getLipx() const;

    /**
     * @brief Get the LIP state in the y-direction
     *
     * @return const Vector3
     */
    const Vector3 getLipy() const;

    /**
     * @brief Update the state with new velocities
     *
     * @param xdz The velocity in the x-direction
     * @param ydz The velocity in the y-direction
     */
    void update(Scalar xdz, Scalar ydz);

    /**
     * @brief Convert the state to a string representation
     *
     * @return std::string The string representation of the state
     */
    std::string toString() const;

    /**
     * @brief Output stream operator for the State class
     *
     * @param os The output stream
     * @param state The State object to be printed
     * @return std::ostream& The output stream
     */
    friend std::ostream& operator<<(std::ostream& os, const State& state);
};

}  // namespace ismpc
