#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/tools/math/pose2.h"

namespace ismpc {

MovingConstraintProvider::MovingConstraintProvider(const FrameInfo& frame_info, const State& state,
                                                   const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void MovingConstraintProvider::update(FootstepPlan& plan) {
    // Initialize the zmp_midpoints to the average of the current feet pose
    // Vector3 midpoint = (state.getSupportFoot().getPose2().getVector() + state.footstep.start_pose.getVector()) /
    // 2;
    Vector3 midpoint = Vector3::Zero();
    plan.zmp_midpoints_x = VectorX::Constant(numP, midpoint(0));
    plan.zmp_midpoints_y = VectorX::Constant(numP, midpoint(1));
    plan.zmp_midpoints_theta = VectorX::Constant(numP, midpoint(2));

    // std::cout << "CURRENT FOOTSTEP: " << state.footstep.toString() << std::endl;
    // std::cout << "CURRENT SUPPORT PHASE: " << state.support_phase << std::endl;
    VectorX time = VectorX::LinSpaced(Config::C, frame_info.tk, frame_info.tk + Config::T_c);
    Scalar ds_start_time = 0.0;  // state.footstep.ds_start;
    Scalar fs_end_time = 0.5;    // state.footstep.end;
    Pose2 start_pose = Pose2(plan.zmp_midpoints_x(0), plan.zmp_midpoints_y(0));
    Pose2 end_pose = Pose2(0.0, 0.0, -0.1);  // state.footstep.end_pose;
    VectorX sigma = sigmaFunction(time, ds_start_time, fs_end_time);

    for (size_t j = 0; j < plan.footsteps.size() - 1; ++j) {
        plan.zmp_midpoints_x = plan.zmp_midpoints_x + sigma * (end_pose.translation(0) - start_pose.translation(0));
        plan.zmp_midpoints_y = plan.zmp_midpoints_y + sigma * (end_pose.translation(1) - start_pose.translation(1));
        plan.zmp_midpoints_theta = plan.zmp_midpoints_theta + sigma * (end_pose.rotation - start_pose.rotation);

        // std::cout << "SIGMA: " << sigma.transpose().format(Config::CleanFmt) << std::endl;

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

    // Regularize ZMP midpoints by setting very small numbers to zero
    const double epsilon = 1e-10;
    plan.zmp_midpoints_x = (plan.zmp_midpoints_x.array().abs() < epsilon).select(0.0, plan.zmp_midpoints_x);
    plan.zmp_midpoints_y = (plan.zmp_midpoints_y.array().abs() < epsilon).select(0.0, plan.zmp_midpoints_y);
    plan.zmp_midpoints_theta =
        (plan.zmp_midpoints_theta.array().abs() < epsilon).select(0.0, plan.zmp_midpoints_theta);

    // std::cout << "ZMP Midpoints X: " << plan.zmp_midpoints_x.transpose().format(Config::CleanFmt) << std::endl;
    // std::cout << "ZMP Midpoints Y: " << plan.zmp_midpoints_y.transpose().format(Config::CleanFmt) << std::endl;
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
