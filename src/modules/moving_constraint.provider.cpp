#include "ismpc_cpp/modules/moving_constraint_provider.h"
#include "ismpc_cpp/tools/math/pose2.h"

namespace ismpc {

MovingConstraintProvider::MovingConstraintProvider(const FrameInfo& frame_info, const State& state,
                                                   const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void MovingConstraintProvider::update(FootstepPlan& plan) {
    /*
     * TODO: this check should be done differently. The condition for which
     * we should enter this if is that we have never computed the moving constraints before,
     * or we are starting from an equilibrium (standing) position.
     */
    if (frame_info.k == 0) {
        initial_lf_pos = state.left_foot.pose.getPose2().getVector();
        initial_rf_pos = state.right_foot.pose.getPose2().getVector();
        // Initialize the zmp_midpoints to the average of the current feet pose
        Vector3 midpoint = (initial_lf_pos + initial_rf_pos) / 2;
        plan.zmp_midpoints_x = VectorX::Constant(numC, midpoint(0));
        plan.zmp_midpoints_y = VectorX::Constant(numC, midpoint(1));
        plan.zmp_midpoints_theta = VectorX::Constant(numC, midpoint(2));
    } else {
        plan.zmp_midpoints_x = VectorX::Constant(numC, plan.zmp_midpoints_x(0));
        plan.zmp_midpoints_y = VectorX::Constant(numC, plan.zmp_midpoints_y(0));
        plan.zmp_midpoints_theta = VectorX::Constant(numC, plan.zmp_midpoints_theta(0));
    }

    // Time vector over which the moving constraint spans
    VectorX time = VectorX::LinSpaced(Config::C, frame_info.tk, frame_info.tk + Config::T_c);

    // Pose2 start_pose = Pose2(plan.zmp_midpoints_x(0), plan.zmp_midpoints_y(0));
    // Pose2 end_pose = Pose2(initial_rf_pos(0), initial_rf_pos(1));
    // VectorX sigma = sigmaFunction(time, ds_start_time, fs_end_time);

    // The footstep currently being executed is always included in the plan
    for (size_t j = 0; j < plan.footsteps.size(); ++j) {
        Footstep& footstep = plan.footsteps[j];
        ds_start_time = footstep.ds_start;
        fs_end_time = footstep.end;
        if (j == 0) {
            start_x = plan.zmp_midpoints_x(0);
            start_y = plan.zmp_midpoints_y(0);
            start_theta = plan.zmp_midpoints_theta(0);
        } else {
            start_x = end_x;
            start_y = end_y;
            start_theta = end_theta;
        }

        end_x = footstep.end_pose.translation(0);
        end_y = footstep.end_pose.translation(1);
        end_theta = footstep.end_pose.rotation;

        sigma = sigmaFunction(time, ds_start_time, fs_end_time);

        // std::cout << "SIGMA: " << sigma.transpose() << std::endl;
        plan.zmp_midpoints_x = plan.zmp_midpoints_x + sigma * (end_x - start_x);
        plan.zmp_midpoints_y = plan.zmp_midpoints_y + sigma * (end_y - start_y);
        plan.zmp_midpoints_theta = plan.zmp_midpoints_theta + sigma * (end_theta - start_theta);

        // std::cout << "START Y: " << start_y << std::endl;
        // std::cout << "ZMP Y INTERM: " << plan.zmp_midpoints_y.transpose().format(Config::CleanFmt) << std::endl;
        // plan.zmp_midpoints_theta = plan.zmp_midpoints_theta + sigma * (end_pose.rotation - start_pose.rotation);
    }
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
