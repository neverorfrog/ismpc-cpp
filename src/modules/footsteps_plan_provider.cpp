#include "ismpc_cpp/modules/footsteps_plan_provider.h"

namespace ismpc {

FootstepsPlanProvider::FootstepsPlanProvider(const FrameInfo& frame_info, const State& state, const WalkState& walk,
                                             const Reference& reference, const FeetLib& feet, const CostLib& cost,
                                             const ConstraintLib& constraint)
    : frame_info(frame_info),
      state(state),
      walk(walk),
      reference(reference),
      feet(feet),
      cost(cost),
      constraint(constraint) {}

void FootstepsPlanProvider::update(FootstepsPlan& footsteps) {
    computeTiming(footsteps);
    computeThetaSequence(footsteps);
    computePositionSequence(footsteps);
    computeZmpMidpoints(footsteps);
}

void FootstepsPlanProvider::computeTiming(FootstepsPlan& footsteps) {
    std::vector<Scalar> timestamps;
    std::vector<Scalar> timestamps_for_zmp_midpoints;

    Scalar V = reference.getVelocityModule();
    Scalar current_footstep_timestamp = walk.current_footstep_timestamp;
    Scalar expected_duration = T_bar * (alpha + v_bar) / (alpha + V);
    Scalar time_of_next_step = current_footstep_timestamp + expected_duration;
    while (time_of_next_step <= frame_info.tk + T_p) {
        timestamps.push_back(truncateToDecimalPlaces(time_of_next_step, 2));
        time_of_next_step += expected_duration;
    }

    footsteps.timestamps = timestamps;
    footsteps.num_predicted_footsteps = timestamps.size();

    timestamps_for_zmp_midpoints = timestamps;
    timestamps_for_zmp_midpoints.push_back(time_of_next_step);
    footsteps.timestamps_for_zmp_midpoints = timestamps_for_zmp_midpoints;

    int num_controlled_footsteps = 0;
    while (timestamps[num_controlled_footsteps] <= frame_info.tk + T_c &&
           num_controlled_footsteps < footsteps.num_predicted_footsteps) {
        num_controlled_footsteps += 1;
    }
    footsteps.num_controlled_footsteps = num_controlled_footsteps;
}

void FootstepsPlanProvider::computeThetaSequence(FootstepsPlan& footsteps) {
    isize d = footsteps.num_predicted_footsteps;  // number of primal variables
    isize n_eq = 0;                               // number of equality constraints
    isize n_in = d;                               // number of inequality constraints
    QP<Scalar> theta_qp = QP<Scalar>(d, n_eq, n_in);
    theta_qp.settings.eps_abs = 1e-4;
    theta_qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    theta_qp.settings.verbose = false;

    // Cost
    Cost theta_cost = cost.getThetaCost();

    // Inequality constraint matrix
    InequalityConstraint theta_constraint = constraint.getThetaConstraint();

    // Solving the optimization problem
    theta_qp.work.timer.start();
    theta_qp.init(theta_cost.H, theta_cost.g, nullopt, nullopt, theta_constraint.C, theta_constraint.l,
                  theta_constraint.u);
    theta_qp.solve();
    theta_qp.work.timer.stop();
    footsteps.total_planner_qp_duration += theta_qp.work.timer.elapsed().user;

    footsteps.theta = theta_qp.results.x;
}

void FootstepsPlanProvider::computePositionSequence(FootstepsPlan& footsteps) {
    isize F = footsteps.num_predicted_footsteps;  // number of footsteps
    isize d = 2 * F;                              // number of primal variables (x and y)
    isize n_eq = 0;                               // number of equality constraints
    isize n_in = d;                               // number of inequality constraints
    QP<Scalar> position_qp = QP<Scalar>(d, n_eq, n_in);
    position_qp.settings.eps_abs = 1e-4;
    position_qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    position_qp.settings.verbose = false;

    // Cost
    Cost pos_cost = cost.getPositionCost();

    // Inequality constraint matrix
    InequalityConstraint kinematic_constraint = constraint.getKinematicConstraint(F);

    // Solving the optimization problem
    position_qp.work.timer.start();
    position_qp.init(pos_cost.H, pos_cost.g, nullopt, nullopt, kinematic_constraint.C, kinematic_constraint.l,
                     kinematic_constraint.u);
    position_qp.solve();
    position_qp.work.timer.stop();
    footsteps.total_planner_qp_duration += position_qp.work.timer.elapsed().user;

    footsteps.x = position_qp.results.x.block(0, 0, F, 1);
    footsteps.y = position_qp.results.x.block(F, 0, F, 1);
}

// TODO: Maybe this could be optimized? In the end it looks just like a sliding window.
void FootstepsPlanProvider::computeZmpMidpoints(FootstepsPlan& footsteps) {
    int j = 0;
    Scalar t = frame_info.tk;
    Scalar t_start = walk.current_footstep_timestamp;
    Scalar t_end = footsteps.timestamps_for_zmp_midpoints[0];

    Vector3 current_support_foot_pose = feet.getSupportFootPose().getPose2().getVector();
    Vector3 previous_support_foot_pose = walk.previous_support_foot_pose.getVector();

    for (int i = 0; i < numP; ++i) {
        footsteps.footstep_indices.push_back(j);
        Scalar double_support_duration = (t_end - t_start) * ds_percentage;
        Scalar t_switch = t_start + double_support_duration;

        if (t_start <= t && t < t_switch) {
            footsteps.support_phases.push_back(SupportPhase::DOUBLE);
            Scalar a = (t - t_start) / (double_support_duration);
            footsteps.zmp_midpoints.col(i) = (1 - a) * previous_support_foot_pose + a * current_support_foot_pose;
        } else if (t >= t_switch) {
            footsteps.support_phases.push_back(SupportPhase::SINGLE);
            footsteps.zmp_midpoints.col(i) = current_support_foot_pose;
            if (t >= t_end) {
                previous_support_foot_pose = current_support_foot_pose;
                Vector3 predicted_pose = Vector3(footsteps.x(j), footsteps.y(j), footsteps.theta(j));
                current_support_foot_pose = predicted_pose;
                t_start = t_end;
                j += 1;
                t_end = footsteps.timestamps_for_zmp_midpoints[j];
            }
        }
        t += delta;
    }
}

}  // namespace ismpc
