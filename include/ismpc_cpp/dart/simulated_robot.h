#pragma once

#include <Eigen/Core>
#include <iostream>

#include "dart/dart.hpp"
#include "ismpc_cpp/dart/qp_solver.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/debug.h"
#include "ismpc_cpp/tools/math/rotation_matrix.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/support_phase.h"
#include "ismpc_cpp/types/tail_type.h"

namespace ismpc {

struct SimulatedRobot {
    VectorX initial_configuration;
    Matrix joint_selection;

    // State inside dart simulator (d stands for dart)
    dart::dynamics::SkeletonPtr skeleton;
    dart::dynamics::BodyNode* d_base;
    dart::dynamics::BodyNode* d_torso;
    dart::dynamics::BodyNode* d_left_foot;
    dart::dynamics::BodyNode* d_right_foot;

    // Utility functions
    VectorX getJointRequest(const State& desired);
    void setInitialConfiguration();

    SimulatedRobot() = default;
    ~SimulatedRobot() = default;
    SimulatedRobot(dart::dynamics::SkeletonPtr skeleton);

    QPSolver qp;

    Scalar total_ik_duration = 0.0;
};

}  // namespace ismpc
