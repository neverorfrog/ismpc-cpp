#include "ismpc_cpp/modules/foot_trajectory_generator.h"

#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/RotationBase.h>

namespace ismpc {

FootTrajectoryGenerator::FootTrajectoryGenerator(const FrameInfo& frame_info, const State& state,
                                                 const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void FootTrajectoryGenerator::update(State& state) {
    EndEffector swing_foot = state.getSwingFoot();

    if (state.support_phase == SupportPhase::DOUBLE) {
        swing_foot.lin_vel << 0, 0, 0;
        swing_foot.lin_acc << 0, 0, 0;
        swing_foot.ang_vel << 0, 0, 0;
        swing_foot.ang_acc << 0, 0, 0;
    } else if (state.support_phase == SupportPhase::SINGLE) {
        Vector2 start_pos = state.footstep.start_pose.translation;
        Vector2 end_pos = state.footstep.end_pose.translation;
        Scalar start_theta = state.footstep.start_pose.rotation;
        Scalar end_theta = state.footstep.end_pose.rotation;

        Scalar ss_duration = (state.footstep.ds_start - state.footstep.start);
        Scalar time_in_step = (frame_info.tk - state.footstep.start) / (ss_duration);

        // 2D Pose with cubic polynomial interpolation
        Vector2 desired_pos = start_pos + (end_pos - start_pos) * quintic(time_in_step);
        Scalar desired_theta = start_theta + (end_theta - start_theta) * quintic(time_in_step);
        swing_foot.pose = Pose3(RotationMatrix::aroundZ(desired_theta), Vector3(desired_pos(0), desired_pos(1), 0));
        swing_foot.pose.euler = Vector3(0, 0, desired_theta);

        // Linear Velocity with cubic polynomial interpolation
        // std::cout << "QUINTIC: " << quintic(time_in_step) << std::endl;
        // std::cout << "QUINTIC_DOT: " << quintic_dot(time_in_step) << std::endl;
        // std::cout << "QUINTIC_DDOT: " << quintic_ddot(time_in_step) << std::endl;
        swing_foot.lin_vel.segment(0, 2) = (end_pos - start_pos) * quintic_dot(time_in_step);
        swing_foot.lin_acc.segment(0, 2) = (end_pos - start_pos) * quintic_ddot(time_in_step);

        // Angular Velocity with cubic polynomial interpolation
        swing_foot.ang_vel = Vector3(0, 0, (end_theta - start_theta) * quintic_dot(time_in_step));
        swing_foot.ang_acc = Vector3(0, 0, (end_theta - start_theta) * quintic_ddot(time_in_step));

        // Height with quartic polynomial interpolation
        // std::cout << "SEXTIC: " << sextic(time_in_step) << std::endl;
        // std::cout << "SEXTIC_DOT: " << sextic_dot(time_in_step) << std::endl;
        // std::cout << "SEXTIC_DDOT: " << sextic_ddot(time_in_step) << std::endl;
        swing_foot.pose.translation(2) = step_height * sextic(time_in_step);
        swing_foot.lin_vel(2) = step_height * sextic_dot(time_in_step);
        swing_foot.lin_acc(2) = step_height * sextic_ddot(time_in_step);
    }

    state.setDesiredSwingFoot(swing_foot);
}

}  // namespace ismpc
