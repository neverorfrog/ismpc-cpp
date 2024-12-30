#pragma once

#include "ismpc_cpp/representations/walk_state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/end_effector.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"

namespace ismpc {

struct State {
    EndEffector left_foot{};
    EndEffector right_foot{};
    Vector3 com_pos{};
    Vector3 com_vel{};
    Vector3 com_acc{};
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

    Scalar total_mpc_qp_duration = 0.0;
    Scalar total_mpc_preprocessing_duration = 0.0;
    Scalar total_mpc_postprocessing_duration = 0.0;

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
     * @brief Get the next LIP state in the x-direction
     *
     * @param xdz The velocity in the x-direction
     * @return const Vector3
     */
    const Vector3 getNextLipx(Scalar xdz) const;

    /**
     * @brief Get the next LIP state in the y-direction
     *
     * @param ydz The velocity in the y-direction
     * @return const Vector3
     */
    const Vector3 getNextLipy(Scalar ydz) const;

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
