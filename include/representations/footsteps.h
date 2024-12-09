#pragma once

#include <vector>
#include "tools/config/config.h"
#include "representations/lip_robot.h"
#include "types/support_phase.h"
#include "types/math_types.h"

namespace ismpc {

class FootstepsPlan {
public:
    std::vector<Scalar> timestamps;                     // timestamps of the next footsteps
    std::vector<Scalar> timestamps_for_zmp_midpoints;  // with one extra timestamp beyond Tp for the zmp midpoints

    VectorX theta;
    VectorX x;
    VectorX y;

    int num_predicted_footsteps;
    int num_controlled_footsteps;

    std::vector<int> footstep_indices;
    std::vector<SupportPhase> support_phases;
    Matrix zmp_midpoints = Matrix::Zero(3, Config::P);

    Scalar total_planner_qp_duration;

    VectorX get_zmp_midpoints_x() const;
    VectorX get_zmp_midpoints_y() const;
    VectorX get_zmp_midpoints_theta() const;

    Scalar getNextTimestamp() const;
};

}  // namespace ismpc
