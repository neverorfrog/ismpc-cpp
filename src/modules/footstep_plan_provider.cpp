#include "ismpc_cpp/modules/footstep_plan_provider.h"

namespace ismpc {

FootstepPlanProvider::FootstepPlanProvider(const FrameInfo& frame_info, const Reference& reference,
                                           const State& state, const FootstepPlan& plan)
    : frame_info(frame_info), reference(reference), state(state), plan(plan) {}

void FootstepPlanProvider::update(FootstepPlan& plan) {
    Scalar time_since_last_plan = frame_info.tk - last_plan_timestamp;
    Scalar ss_duration = state.footstep.ds_start - state.footstep.start;

    // TODO: planning frequency is hardcoded to the ss_duration. Maybe not the best choice? Should actually check
    // double support etc...
    if ((time_since_last_plan >= ss_duration && frame_info.tk >= Config::fs_duration) || frame_info.tk == 0) {
        last_plan_timestamp = frame_info.tk;
        theta_sequence.clear();
        x_sequence.clear();
        y_sequence.clear();
        timestamps.clear();
        plan.footsteps.clear();

        computeTiming();
        computeThetaSequence();
        computePositionSequence();

        // Need to insert the current footstep pose in the plan
        const Pose2& sf_pose = state.getSwingFoot().getPose2();
        theta_sequence.insert(theta_sequence.begin(), sf_pose.rotation);
        x_sequence.insert(x_sequence.begin(), sf_pose.translation(0));
        y_sequence.insert(y_sequence.begin(), sf_pose.translation(1));
        timestamps.insert(timestamps.begin(), state.footstep.start);

        plan.footsteps.push_back(state.footstep);
        for (int j = 1; j < num_predicted_footsteps + 1; ++j) {
            Footstep footstep{};
            footstep.support_foot = plan.footsteps[j - 1].support_foot == Foot::right ? Foot::left : Foot::right;
            footstep.end_pose = Pose2(theta_sequence[j], x_sequence[j], y_sequence[j]);
            footstep.start = timestamps[j];
            footstep.end = timestamps[j + 1];
            footstep.ds_start = footstep.start + (1 - ds_percentage) * (footstep.end - footstep.start);
            footstep.walk_phase = WalkPhase::WALKING;
            plan.footsteps.push_back(footstep);
        }

    }
}

void FootstepPlanProvider::computeTiming() {
    // Scalar V = reference.getVelocityModule();
    Scalar current_footstep_timestamp = state.footstep.start;
    Scalar expected_duration = Config::fs_duration;  // TODO formula: T_bar * (alpha + v_bar) / (alpha + V);
    Scalar time_of_next_step = current_footstep_timestamp + expected_duration;

    while (time_of_next_step <= frame_info.tk + T_p) {
        timestamps.push_back(truncateToDecimalPlaces(time_of_next_step, 2));
        time_of_next_step += expected_duration;
    }
    num_predicted_footsteps = timestamps.size();
    timestamps.push_back(time_of_next_step);
}

void FootstepPlanProvider::computeThetaSequence() {
    isize d = num_predicted_footsteps;  // number of primal variables
    isize n_eq = 0;                     // number of equality constraints
    isize n_in = d;                     // number of inequality constraints
    QP<Scalar> theta_qp = QP<Scalar>(d, n_eq, n_in);
    theta_qp.settings.eps_abs = 1e-4;

    // Cost
    Cost th_cost = getThetaCost();

    // Inequality constraint matrix
    InequalityConstraint th_constraint = getThetaConstraint();

    // Solving the optimization problem
    theta_qp.work.timer.start();
    theta_qp.init(th_cost.H, th_cost.g, nullopt, nullopt, th_constraint.C, th_constraint.l, th_constraint.u);
    theta_qp.solve();
    theta_qp.work.timer.stop();
    total_planner_qp_duration += theta_qp.work.timer.elapsed().user;

    if (theta_qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
        throw std::runtime_error("Theta QP solver failed to find a solution.");
    }

    VectorX result = theta_qp.results.x;
    theta_sequence = std::vector<Scalar>(result.data(), result.data() + result.size());
}

void FootstepPlanProvider::computePositionSequence() {
    isize F = num_predicted_footsteps;  // number of footsteps
    isize d = 2 * F;                    // number of primal variables (x and y)
    isize n_eq = 0;                     // number of equality constraints
    isize n_in = d;                     // number of inequality constraints
    QP<Scalar> position_qp = QP<Scalar>(d, n_eq, n_in);
    position_qp.settings.eps_abs = 1e-4;

    // Cost
    Cost pos_cost = getPositionCost();

    // Inequality constraint matrix
    InequalityConstraint kin_constraint = getKinematicConstraint(F);

    // Solving the optimization problem
    position_qp.work.timer.start();
    position_qp.init(pos_cost.H, pos_cost.g, nullopt, nullopt, kin_constraint.C, kin_constraint.l, kin_constraint.u);
    position_qp.solve();
    position_qp.work.timer.stop();
    total_planner_qp_duration += position_qp.work.timer.elapsed().user;

    if (position_qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
        throw std::runtime_error("Theta QP solver failed to find a solution.");
    }

    VectorX solution = position_qp.results.x;
    solution = (solution.array().abs() < 1e-9).select(0.0, solution);
    x_sequence = std::vector<Scalar>(solution.data(), solution.data() + F);
    y_sequence = std::vector<Scalar>(solution.data() + F, solution.data() + 2 * F);
}

InequalityConstraint FootstepPlanProvider::getKinematicConstraint(int F) const {
    VectorX cos_theta = VectorX::Map(theta_sequence.data(), theta_sequence.size()).array().cos();
    VectorX sin_theta = VectorX::Map(theta_sequence.data(), theta_sequence.size()).array().sin();

    Matrix C = Matrix::Zero(2 * F, 2 * F);
    Matrix Cxj = Matrix::Zero(2, 2);
    Matrix Cyj = Matrix::Zero(2, 2);
    VectorX lb = VectorX::Zero(2 * F);
    VectorX ub = VectorX::Zero(2 * F);
    VectorX lbj = VectorX::Zero(2);
    VectorX ubj = VectorX::Zero(2);

    Pose2 sf_pose = state.getSwingFoot().getPose2();

    Scalar current_x = sf_pose.translation(0);
    Scalar current_y = sf_pose.translation(1);

    for (int j = 0; j < F; ++j) {
        int sign = state.getFootstepSign(j);
        if (j == 0) {
            C.block(0, 0, 2, 1) << cos_theta(j), -sin_theta(j);
            C.block(0, F, 2, 1) << sin_theta(j), cos_theta(j);

            Scalar oriented_current_x = cos_theta(j) * current_x + sin_theta(j) * current_y;
            Scalar oriented_current_y = -sin_theta(j) * current_x + cos_theta(j) * current_y;
            lbj << oriented_current_x - 0.5 * dax, oriented_current_y + sign * l - 0.5 * day;
            ubj << oriented_current_x + 0.5 * dax, oriented_current_y + sign * l + 0.5 * day;
        } else {
            Cxj << -cos_theta(j), cos_theta(j), sin_theta(j), -sin_theta(j);
            Cyj << -sin_theta(j), sin_theta(j), -cos_theta(j), cos_theta(j);

            C.block(2 * j, j - 1, 2, 2) = Cxj;
            C.block(2 * j, (j - 1) + F, 2, 2) = Cyj;

            lbj << -0.5 * dax, sign * l - 0.5 * day;
            ubj << 0.5 * dax, sign * l + 0.5 * day;
        }

        lb.segment(2 * j, 2) = lbj;
        ub.segment(2 * j, 2) = ubj;
    }

    return InequalityConstraint(C, lb, ub);
}

InequalityConstraint FootstepPlanProvider::getThetaConstraint() const {
    int F = num_predicted_footsteps;
    // Inequality constraint matrix
    Matrix C = Matrix::Zero(F, F);
    C.block(0, 0, F - 1, F - 1).diagonal(0).setConstant(-1);
    C.diagonal(1).setConstant(1);

    // Upper and lower bounds
    VectorX ub = VectorX::Zero(F);
    VectorX lb = VectorX::Zero(F);
    ub.setConstant(theta_max);
    lb.setConstant(-theta_max);

    return InequalityConstraint(C, lb, ub);
}

Cost FootstepPlanProvider::getThetaCost() const {
    int F = num_predicted_footsteps;
    VectorX delta_theta = VectorX::Zero(F);
    Scalar t_start = state.footstep.start;
    Scalar t_end;
    for (int j = 0; j < F; ++j) {
        t_end = timestamps[j];
        delta_theta(j) = reference.integrateOmega(t_start, t_end);
        t_start = t_end;
    }
    Scalar current_theta = state.getSwingFoot().getPose2().rotation;

    // Cost Matrix
    Matrix H = Matrix::Identity(F, F);
    H.diagonal(0) = 4 * Matrix::Ones(F, 1);
    H.diagonal(1).setConstant(-2);
    H.diagonal(-1).setConstant(-2);
    H(F - 1, F - 1) = 2;

    // Cost vector
    VectorX g = VectorX::Zero(F);
    for (int j = 0; j < F; ++j) {
        if (j == 0) {
            g(j) = 2 * (delta_theta(1) - delta_theta(0) - current_theta);
        } else if (j == F - 1) {
            g(j) = -2 * delta_theta(j);
        } else if (j < F - 1) {
            g(j) = 2 * (delta_theta(j + 1) - delta_theta(j));
        }
    }

    return Cost(H, g);
}

Cost FootstepPlanProvider::getPositionCost() const {
    int F = num_predicted_footsteps;

    // Oriented Displacements and Integrated Theta
    VectorX delta_x = VectorX::Zero(F);
    VectorX delta_y = VectorX::Zero(F);
    Scalar integrated_theta = state.getSwingFoot().getPose2().rotation;
    Scalar t_start = state.footstep.start;
    Scalar t_end;
    Pose2 displacement;
    for (int j = 0; j < F; ++j) {
        t_end = timestamps[j];
        displacement = reference.integrateVelocity(t_start, t_end, integrated_theta);
        t_start = t_end;
        Scalar optimal_theta = theta_sequence[j];
        int sign = state.getFootstepSign(j);
        delta_x(j) = displacement.translation(0) + sign * (-sin(optimal_theta)) * l;
        delta_y(j) = displacement.translation(1) + sign * (cos(optimal_theta)) * l;
        integrated_theta = displacement.rotation;
    }

    // Cost Matrix
    Matrix H = Matrix::Zero(2 * F, 2 * F);
    Matrix Hx = Matrix::Identity(F, F);
    Hx.diagonal(0) = 4 * Matrix::Ones(F, 1);
    Hx.diagonal(1).setConstant(-2);
    Hx.diagonal(-1).setConstant(-2);
    Hx(F - 1, F - 1) = 2;
    H.block(0, 0, F, F) = Hx;
    H.block(F, F, F, F) = Hx;

    Pose2 sf_pose = state.getSwingFoot().getPose2();
    Scalar current_x = sf_pose.translation(0);
    Scalar current_y = sf_pose.translation(1);

    // Cost vector
    VectorX g = VectorX::Zero(2 * F);
    for (int j = 0; j < F; ++j) {
        if (j == 0) {
            g(j) = 2 * (delta_x(1) - delta_x(0) - current_x);
            g(j + F) = 2 * (delta_y(1) - delta_y(0) - current_y);
        } else if (j == F - 1) {
            g(j) = -2 * delta_x(j);
            g(j + F) = -2 * delta_y(j);
        } else if (j < F - 1) {
            g(j) = 2 * (delta_x(j + 1) - delta_x(j));
            g(j + F) = 2 * (delta_y(j + 1) - delta_y(j));
        }
    }

    return Cost(H, g);
}

}  // namespace ismpc
