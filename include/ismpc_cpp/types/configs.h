#include "ismpc_cpp/types/math_types.h" // For Scalar type
#include "ismpc_cpp/types/tail_type.h"

namespace ismpc {


struct TimeParams {
    Scalar delta = 0.01; // Sampling interval
    int N = 100; // Simulation steps
    int P = 10; // Preview horizon steps
    int C = 5; // Control horizon steps
    Scalar T_p = P * delta; // Preview horizon time length
    Scalar T_c = C * delta; // Control horizon time length
};

struct LipParams {
    Scalar h = 0.8; // Height of the CoM of the robot
    Scalar g = 9.81; // Gravity
    Scalar eta = std::sqrt(g / h); // Natural frequency
    Scalar dxz = 0.05;
    Scalar dyz = 0.05;
    Scalar zmp_vx_max = 0.5;
    Scalar zmp_vy_max = 0.5;
};

struct RobotPhysicsParams {
    Scalar initial_lf_x = 0.0;
    Scalar initial_lf_y = 0.0;
    Scalar initial_rf_x = 0.0;
    Scalar initial_rf_y = 0.0;
};

struct GaitParams {
    Scalar ds_percentage = 0.1;
    Scalar ss_percentage = 1 - ds_percentage;
    Scalar step_height = 0.05;

    Scalar l = 0.15; // Footstep length
    Scalar dax = 0.1; // Footstep length in x direction
    Scalar day = 0.05; // Footstep length in y direction

    Scalar theta_max = 0.5; // Maximum angle variation between consecutive footsteps
    Scalar T_bar = 0.8; // Cruise parameter
    Scalar L_bar = 0.5; // Cruise parameter
    Scalar v_bar = 0.5; // Cruise parameter
    Scalar alpha = 10.0; // Cruise parameter
};

struct ReferenceParams {
    Scalar des_vel_x = 0.0; // Desired velocity in x direction
    Scalar des_vel_y = 0.0; // Desired velocity in y direction
    Scalar des_omega = 0.0; // Desired angular velocity
};

struct MPCParams {
    Scalar beta = 1.0; // Cost weight
    TailType tail_type = TailType::PERIODIC;
};


} // namespace ismpc