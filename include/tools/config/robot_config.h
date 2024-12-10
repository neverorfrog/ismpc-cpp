#pragma once

#include <yaml-cpp/yaml.h>

#include <iostream>

#include "tools/math/angle.h"
#include "tools/systemvars.h"
#include "types/math_types.h"

namespace ismpc {

struct RobotConfig {
    struct Initializer {
        Initializer() {
            if (!constructed) {
                init_params();
                constructed = true;
            }
        }
    };

    static inline bool constructed = false;
    static inline Initializer initializer{};

    static inline Scalar g = 9.81;  // Gravity
    static inline Scalar eta{};     // sqrt(g/h)

    static inline Scalar h{};            // Height of the CoM of the robot
    static inline Scalar theta_max{};    // Maximum angle variation between consecutive footsteps
    static inline Scalar step_height{};  // Height of the step
    static inline Scalar foot_com_height{};

    static inline Scalar l{}, dax{}, day{};  // Footstep length
    static inline Scalar beta{};             // Weight on cost function

    static inline Scalar dxz{}, dyz{};                // ZMP constraints
    static inline Scalar zmp_vx_max{}, zmp_vy_max{};  // ZMP velocity constraints

    static inline Scalar T_bar{}, L_bar{}, v_bar{}, alpha{};  // Cruise parameters
    static inline Scalar ds_percentage{}, ss_percentage{};    // Percentage of double support and single support

    // Initial configuration
    static inline Scalar left_foot_x{}, left_foot_y{}, right_foot_x{}, right_foot_y{};  // Initial feet
    static inline Scalar right_hip_roll, right_hip_pitch, right_ankle_roll, right_ankle_pitch;
    static inline Scalar left_hip_roll, left_hip_pitch, left_ankle_roll, left_ankle_pitch;
    static inline Scalar base_pitch;
    static inline Scalar chest_pitch, chest_yaw;

    // Task Gains
    static inline Eigen::Matrix<Scalar, 12, 12> task_gain = Eigen::Matrix<Scalar, 12, 12>::Identity();
    static inline Scalar ik_gain{};

    static void inline init_params();
};

}  // namespace ismpc
