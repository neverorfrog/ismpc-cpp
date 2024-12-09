#pragma once

#include <yaml-cpp/yaml.h>

#include <string>

#include "tools/systemvars.h"
#include "types/math_types.h"
#include "types/tail_type.h"

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

    static inline bool save_log;  // Save log file

    static inline Scalar delta{};  // Sampling interval

    static inline int N{};       // Simulation steps
    static inline int P{};       // Preview horizon steps
    static inline int C{};       // Control horizon steps
    static inline int W{};       // Iterations to wait before starting
    static inline Scalar T_p{};  // Preview horizon time length
    static inline Scalar T_c{};  // Control horizon time length

    static inline Scalar des_vel_x{}, des_vel_y{}, des_omega{};  // Reference velocity
    static inline TailType tail_type{};                          // Tail type

    static void init_params();
};

}  // namespace ismpc
