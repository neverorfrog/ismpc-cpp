#include "ismpc_cpp/modules/foot_trajectory_generator.h"

#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/RotationBase.h>

namespace ismpc {

FootTrajectoryGenerator::FootTrajectoryGenerator(const FrameInfo& frame_info, const State& state,
                                                 const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void FootTrajectoryGenerator::update(State& state) {
    // EndEffector swing_foot = feet.getSwingFoot();

    // if (walk.support_phase == SupportPhase::DOUBLE) {
    //     swing_foot.lin_vel << 0, 0, 0;
    //     swing_foot.lin_acc << 0, 0, 0;
    //     swing_foot.ang_vel << 0, 0, 0;
    //     swing_foot.ang_acc << 0, 0, 0;
    // } else if (walk.support_phase == SupportPhase::SINGLE) {
    //     Vector2 start_pos = walk.previous_support_foot_pose.translation;
    //     Vector2 end_pos = walk.next_support_foot_pose.translation;
    //     Vector2 step = end_pos - start_pos;
    //     Scalar start_theta = walk.previous_support_foot_pose.rotation;
    //     Scalar end_theta = walk.next_support_foot_pose.rotation;

    //     Scalar ss_duration = (walk.next_footstep_timestamp - walk.current_footstep_timestamp) * ss_percentage;
    //     Scalar ds_duration = (walk.next_footstep_timestamp - walk.current_footstep_timestamp) * ds_percentage;
    //     Scalar time_in_step = (frame_info.tk - (walk.current_footstep_timestamp + ds_duration)) / (ss_duration);

    //     // 2D Pose with cubic polynomial interpolation
    //     Vector2 desired_pos = start_pos + step * cubic(time_in_step);
    //     Scalar desired_theta = start_theta + (end_theta - start_theta) * cubic(time_in_step);
    //     swing_foot.pose = Pose3(RotationMatrix::aroundZ(desired_theta), Vector3(desired_pos(0), desired_pos(1),
    //     0)); swing_foot.pose.euler = Vector3(0, 0, desired_theta);

    //     // Linear Velocity with cubic polynomial interpolation
    //     swing_foot.lin_vel.segment(0, 2) = step * cubic_dot(time_in_step) / ss_duration;
    //     swing_foot.lin_acc.segment(0, 2) = step * cubic_ddot(time_in_step) / (std::pow(ss_duration, 2));

    //     // Angular Velocity with cubic polynomial interpolation
    //     swing_foot.ang_vel(2) = (end_theta - start_theta) * cubic_dot(time_in_step) / ss_duration;
    //     swing_foot.ang_acc(2) = (end_theta - start_theta) * cubic_ddot(time_in_step) / (ss_duration *
    //     ss_duration);

    //     // Height with quartic polynomial interpolation
    //     swing_foot.pose.translation(2) = step_height * quartic(time_in_step);
    //     swing_foot.lin_vel(2) = step_height * quartic_dot(time_in_step) / ss_duration;
    //     swing_foot.lin_acc(2) = step_height * quartic_ddot(time_in_step) / (ss_duration * ss_duration);
    // }

    // feet.setSwingFoot(swing_foot, desired_state);

    // // Set desired torso and base
    // const EndEffector& support_foot = feet.getSupportFoot();
    // desired_state.torso.pose.rotation = RotationMatrix(
    //     support_foot.pose.rotation.getQuaternion().slerp(0.5, swing_foot.pose.rotation.getQuaternion()));

    // desired_state.torso.ang_vel = (support_foot.ang_vel + swing_foot.ang_vel) / 2;
    // desired_state.torso.ang_acc = (support_foot.ang_acc + swing_foot.ang_acc) / 2;
    // desired_state.base.pose.rotation = desired_state.torso.pose.rotation;
    // desired_state.base.ang_vel = desired_state.torso.ang_vel;
    // desired_state.base.ang_acc = desired_state.torso.ang_acc;
}

}  // namespace ismpc
