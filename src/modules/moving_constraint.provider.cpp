#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/tools/math/pose2.h"

namespace ismpc {

MovingConstraintProvider::MovingConstraintProvider(const FrameInfo& frame_info, const State& state,
                                                   const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void MovingConstraintProvider::update(FootstepPlan& plan) {
    if (frame_info.k == 0) {
        initial_lf_pos = state.left_foot.pose.getPose2().getVector();
        initial_rf_pos = state.right_foot.pose.getPose2().getVector();
    }

    // Initialize the zmp_midpoints to the average of the current feet pose
    Vector3 midpoint = (initial_lf_pos + initial_rf_pos) / 2;
    plan.zmp_midpoints_x = VectorX::Constant(numC, midpoint(0));
    plan.zmp_midpoints_y = VectorX::Constant(numC, midpoint(1));
    plan.zmp_midpoints_theta = VectorX::Constant(numC, midpoint(2));

    // Initialize time vector and sigma
    VectorX time = VectorX::LinSpaced(Config::C, frame_info.tk, frame_info.tk + Config::T_c);
    Scalar ds_start_time = 0.0;                      // state.footstep.ds_start;
    Scalar fs_end_time = Config::first_fs_duration;  // state.footstep.end;
    Pose2 start_pose = Pose2(plan.zmp_midpoints_x(0), plan.zmp_midpoints_y(0));
    Pose2 end_pose = Pose2(initial_rf_pos(0), initial_rf_pos(1));
    VectorX sigma = sigmaFunction(time, ds_start_time, fs_end_time);

    for (size_t j = 0; j < plan.footsteps.size() - 1; ++j) {
        plan.zmp_midpoints_x = plan.zmp_midpoints_x + sigma * (end_pose.translation(0) - start_pose.translation(0));
        plan.zmp_midpoints_y = plan.zmp_midpoints_y + sigma * (end_pose.translation(1) - start_pose.translation(1));
        plan.zmp_midpoints_theta = plan.zmp_midpoints_theta + sigma * (end_pose.rotation - start_pose.rotation);

        Footstep& footstep = plan.footsteps[j];
        ds_start_time = footstep.ds_start;
        fs_end_time = footstep.end;
        start_pose = end_pose;
        end_pose = footstep.end_pose;
        sigma = sigmaFunction(time, ds_start_time, fs_end_time);
    }

    plan.zmp_midpoints_x = plan.zmp_midpoints_x + sigma * (end_pose.translation(0) - start_pose.translation(0));
    plan.zmp_midpoints_y = plan.zmp_midpoints_y + sigma * (end_pose.translation(1) - start_pose.translation(1));
    plan.zmp_midpoints_theta = plan.zmp_midpoints_theta + sigma * (end_pose.rotation - start_pose.rotation);
}

VectorX MovingConstraintProvider::sigmaFunction(VectorX time, Scalar t0, Scalar t1) const {
    VectorX start = VectorX::Constant(time.size(), t0);
    VectorX end = VectorX::Constant(time.size(), t1);

    VectorX diff = time - start;
    VectorX duration = end - start;

    VectorX sigma = diff.cwiseQuotient(duration);

    sigma = (sigma.array() < 0).select(0.0, sigma);
    sigma = (sigma.array() > 1).select(1.0, sigma);

    return sigma;
}

}  // namespace ismpc
