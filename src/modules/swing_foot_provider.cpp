#include "modules/swing_foot_provider.h"

namespace ismpc {

SwingFootProvider::SwingFootProvider(const FrameInfo& frame_info, const LipRobot& robot,
                                     const FootstepsPlan& footsteps)
    : frame_info(frame_info), robot(robot), footsteps(footsteps) {}

void SwingFootProvider::update(LipRobot& robot) {
    const WalkState& walk = robot.walk;
    EndEffector& swing_foot = robot.getSwingFoot();

    if (walk.support_phase == SupportPhase::DOUBLE) {
        swing_foot.vel << 0, 0, 0;
        swing_foot.acc << 0, 0, 0;
    } else if (walk.support_phase == SupportPhase::SINGLE) {
        const Pose2& start_pose = walk.previous_support_foot_pose;
        const Pose2& end_pose = walk.next_support_foot_pose;
        const Vector2& start_pos = start_pose.translation;
        const Vector2& end_pos = end_pose.translation;
        const Scalar& start_theta = start_pose.rotation;
        const Scalar& end_theta = end_pose.rotation;

        Scalar ss_duration = (walk.next_footstep_timestamp - walk.current_footstep_timestamp) * ss_percentage;
        Scalar ds_duration = (walk.next_footstep_timestamp - walk.current_footstep_timestamp) * ds_percentage;
        Scalar step_completion = (frame_info.tk - (walk.current_footstep_timestamp + ds_duration)) / (ss_duration);

        Vector2 desired_pos = start_pos + (end_pos - start_pos) * cubic(step_completion);
        Scalar desired_theta = start_theta + (end_theta - start_theta) * cubic(step_completion);
        swing_foot.pose = Pose3(RotationMatrix::aroundZ(desired_theta), Vector3(desired_pos(0), desired_pos(1), 0));
        swing_foot.vel.segment(0, 2) = (end_pos - start_pos) * cubic_dot(step_completion) / ss_duration;
        swing_foot.acc.segment(0, 2) =
            (end_pos - start_pos) * cubic_ddot(step_completion) / (ss_duration * ss_duration);

        // Compute the desired swing foot position (z)
        if (step_completion <= 0.5) {
            swing_foot.pose.translation(2) += cubic(2 * step_completion) * step_height;
            swing_foot.vel(2) += cubic_dot(step_completion * 2) * step_height * 2 / ss_duration;
            swing_foot.acc(2) += cubic_ddot(step_completion * 2) * step_height * 4 / (ss_duration * ss_duration);
        } else if (step_completion > 0.5) {
            swing_foot.pose.translation(2) += step_height - cubic(2 * (step_completion - 0.5)) * step_height;
            swing_foot.vel(2) += -cubic_dot((step_completion - 0.5) * 2.0) * step_height * 2 / ss_duration;
            swing_foot.acc(2) +=
                -cubic_ddot((step_completion - 0.5) * 2.0) * step_height * 4 / (ss_duration * ss_duration);
        }
    }
}

}  // namespace ismpc
