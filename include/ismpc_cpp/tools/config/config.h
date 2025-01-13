#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

#include "ismpc_cpp/tools/systemvars.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/tail_type.h"

namespace ismpc {

struct Config {
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

    static inline Eigen::Vector3d RED, PURPLE, GREEN;
    static inline bool save_log;  // Save log file

    static inline Scalar delta{};  // Sampling interval
    static inline Scalar first_fs_duration{};

    static inline int N{};       // Simulation steps
    static inline int P{};       // Preview horizon steps
    static inline int C{};       // Control horizon steps
    static inline int W{};       // Iterations to wait before starting
    static inline Scalar T_p{};  // Preview horizon time length
    static inline Scalar T_c{};  // Control horizon time length

    static inline Scalar des_vel_x{}, des_vel_y{}, des_omega{};  // Reference velocity
    static inline TailType tail_type{};                          // Tail type

    static inline Eigen::IOFormat CleanFmt{2, 0, ", ", "\n", "[", "]"};

    static void init_params();
};

}  // namespace ismpc
