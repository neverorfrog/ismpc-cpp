#pragma once

#include "tools/config/config.h"
#include "tools/config/robot_config.h"
#include "tools/math/pose2.h"
#include "types/body_parts.h"
#include "types/end_effector.h"
#include "types/math_types.h"
#include "types/support_phase.h"

namespace ismpc {

struct WalkState {
    Pose2 previous_support_foot_pose{};
    Pose2 next_support_foot_pose{};

    Foot support_foot_type;
    Scalar current_footstep_timestamp;
    Scalar next_footstep_timestamp;
    SupportPhase support_phase;

    std::vector<Pose2> footstep_history;
    std::vector<Scalar> timestamp_history;

    WalkState()
        : previous_support_foot_pose(RobotConfig::left_foot_x, RobotConfig::left_foot_y),
          next_support_foot_pose(RobotConfig::left_foot_x + 0.1, RobotConfig::left_foot_y) {
        support_foot_type = Foot::right;  // starting with right foot as support foot
        current_footstep_timestamp = 0.0;
        support_phase = SupportPhase::DOUBLE;
    }

    inline std::string toString() const {
        std::ostringstream oss;
        oss << "Current Footstep Timestamp: " << current_footstep_timestamp << std::endl;
        oss << "Support Foot Type: " << support_foot_type << std::endl;
        oss << "Support Phase: " << support_phase << std::endl;
        oss << "Previous Support Foot: " << previous_support_foot_pose << std::endl;
        oss << "Next Support Foot: " << next_support_foot_pose << std::endl;
        return oss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const WalkState& walkState) {
        os << walkState.toString();
        return os;
    }
};

}  // namespace ismpc
