#pragma once

#include "ismpc_cpp/types/math_types.h" // For Scalar type
#include "ismpc_cpp/types/tail_type.h"
#include <cmath> // Include for std::sqrt

namespace ismpc {


struct MpcParams {
    Scalar delta = 0.012; // Sampling interval
    int N = 100; // Simulation steps (Updated from YAML)
    int P = 300; // Preview horizon steps (Updated from YAML)
    int C = 100; // Control horizon steps (Updated from YAML)
    Scalar T_p = P * delta; // Preview horizon time length
    Scalar T_c = C * delta; // Control horizon time length
    Scalar beta = 200.0; // Cost weight
    TailType tail_type = TailType::PERIODIC; // (Matches YAML)
    int nl = 1; // Number of lip variables
};

struct LipParams {
    Scalar h = 0.75; // Height of the CoM of the robot
    Scalar g = 9.81; // Gravity
    Scalar eta = std::sqrt(g / h); // Natural frequency (Recalculated based on updated h)
    Scalar dxz = 0.2; 
    Scalar dyz = 0.2; 
    Scalar zmp_vx_max = 10.0; 
    Scalar zmp_vy_max = 10.0; 
};

struct InitialFeetParams {
    Scalar lf_x = 0.0; 
    Scalar lf_y = 0.1; 
    Scalar rf_x = 0.0; 
    Scalar rf_y = -0.1; 
};

struct GaitParams {
    Scalar ds_percentage = 0.3; 
    Scalar ss_percentage = 1.0 - ds_percentage; 
    Scalar step_height = 0.01;

    Scalar l = 0.2; // Footstep length
    Scalar dax = 0.2; // Footstep length in x direction
    Scalar day = 0.2; // Footstep length in y direction

    Scalar theta_max = 0.3927; // Maximum angle variation between consecutive footsteps
    Scalar T_bar = 0.3; // Cruise parameter
    Scalar L_bar = 0.3; // Cruise parameter
    Scalar v_bar = L_bar / T_bar; // Cruise parameter (Default, not in hrp4.yaml)
    Scalar alpha = 0.5; // Cruise parameter

    Scalar fs_duration = 1.0; // Footstep duration
};

struct ReferenceParams {
    Scalar des_vel_x = 0.05; // Desired velocity in x direction (Updated from YAML)
    Scalar des_vel_y = 0.0; // Desired velocity in y direction (Matches YAML)
    Scalar des_omega = 0.0; // Desired angular velocity (Matches YAML)
};

struct Params {
    MpcParams mpc{};
    LipParams lip{};
    InitialFeetParams initial_feet{};
    GaitParams gait{};
    ReferenceParams reference{};
};


} // namespace ismpc