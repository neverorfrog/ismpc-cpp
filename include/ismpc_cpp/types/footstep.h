#pragma once

#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/walk_phase.h"

namespace ismpc {
struct Footstep {
    Pose2 start_pose;  // the pose of the footstep when it lifts off the ground and the single support phase starts
    Pose2 end_pose;    // the pose of the footstep when it hits the ground and the double support phase starts
    WalkPhase walk_phase;  // the walk phase when the footstep is taken (starting, walking, falling)
    Foot support_foot;     // the foot that is supporting the robot when the footstep is taken (left, right)
    Scalar start;          // the time instant in which the foot lifts off the ground
    Scalar ds_start;       // the time instant in which the foot hits the ground
    Scalar end;            // the time instant in which the double support phase ends and a new footstep starts

    Footstep() = default;

    Footstep(const Pose2& start_pose, const Pose2& end_pose, WalkPhase walk_phase, Foot support_foot, Scalar start,
             Scalar ds_start, Scalar end)
        : start_pose(start_pose),
          end_pose(end_pose),
          walk_phase(walk_phase),
          support_foot(support_foot),
          start(start),
          ds_start(ds_start),
          end(end) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "Start Pose: " << start_pose.translation.transpose() << std::endl;
        oss << "End Pose: " << end_pose.translation.transpose() << std::endl;
        oss << "Support Foot: " << support_foot << std::endl;
        oss << "FS Start: " << start << std::endl;
        oss << "DS Start: " << ds_start << std::endl;
        oss << "FS End: " << end << std::endl;
        return oss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Footstep& footstep) {
        os << footstep.toString();
        return os;
    }
};
}  // namespace ismpc
