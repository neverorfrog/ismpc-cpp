#include "ismpc_cpp/modules/model_predictive_controller.h"

namespace ismpc {

ModelPredictiveController::ModelPredictiveController(const FrameInfo& frame_info, const State& state,
                                                     const WalkState& walk, const FootstepsPlan& footsteps,
                                                     const FeetLib& feet, const CostLib& cost,
                                                     const ConstraintLib& constraint)
    : frame_info(frame_info),
      state(state),
      walk(walk),
      footsteps(footsteps),
      feet(feet),
      cost(cost),
      constraint(constraint) {}

void ModelPredictiveController::update(State& desired_state) {
    // ================== PREPROCESSING ==================
    start = std::chrono::high_resolution_clock::now();
    // State related stuff
    isize Fprime = footsteps.num_controlled_footsteps;
    isize d = 2 * numC + 2 * Fprime;  // number of primal variables (xdz, ydz, xf and yf)
    Vector3 lipx = state.getLipx();
    Vector3 lipy = state.getLipy();

    // Cost Function
    Cost mpc_cost = cost.getMpcCost();

    // Kinematic Constraint (2 * Fprime inequalities)
    isize dimk = 2 * Fprime;
    kinematic_constraint = constraint.getKinematicConstraint(Fprime);

    // ZMP Constraint (2 * numC inequalities)
    zmp_constraint = constraint.getZmpConstraint(lipx, lipy);
    zmp_velocity_constraint = constraint.getZmpVelocityConstraint();

    // Combine inequality constraints
    isize n_in = 2 * dimz + dimk;
    C = Matrix::Zero(n_in, d);
    C.block(0, 0, dimz, dimz) = zmp_constraint.C;
    C.block(dimz, dimz, dimk, dimk) = kinematic_constraint.C;
    C.block(dimz + dimk, 0, dimz, dimz) = zmp_velocity_constraint.C;
    l = VectorX::Zero(n_in);
    l.segment(0, dimz) << zmp_constraint.l;
    l.segment(dimz, dimk) << kinematic_constraint.l;
    l.segment(dimz + dimk, dimz) << zmp_velocity_constraint.l;
    u = VectorX::Zero(n_in);
    u.segment(0, dimz) << zmp_constraint.u;
    u.segment(dimz, dimk) << kinematic_constraint.u;
    u.segment(dimz + dimk, dimz) << zmp_velocity_constraint.u;

    // Stability Constraint
    EqualityConstraint stability_constraint = constraint.getStabilityConstraint(lipx, lipy);
    end = std::chrono::high_resolution_clock::now();
    total_mpc_preprocessing_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // ==================================================

    // ================== SOLVE QP ==================
    QP<Scalar> qp = QP<Scalar>(d, n_eq, n_in);
    start = std::chrono::high_resolution_clock::now();
    qp = QP<Scalar>(d, n_eq, n_in);
    qp.settings.max_iter = 50;
    qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;
    qp.init(mpc_cost.H, mpc_cost.g, stability_constraint.A, stability_constraint.b, C, l, u);
    qp.solve();

    if (qp.results.info.status != PROXQP_SOLVED) {
        throw std::runtime_error("QP solver failed to find a solution.");
    }
    end = std::chrono::high_resolution_clock::now();
    total_mpc_qp_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // =============================================

    // ================== POSTPROCESSING ==================
    // Extract the solution
    Xdz = qp.results.x.segment(0, numC);
    Ydz = qp.results.x.segment(numC, numC);
    Xf = qp.results.x.segment(2 * numC, Fprime);
    Yf = qp.results.x.segment(2 * numC + Fprime, Fprime);

    // Integrate the lip velocities
    Vector3 predicted_x = state.getNextLipx(Xdz(0));
    Vector3 predicted_y = state.getNextLipy(Ydz(0));

    // Set desired state
    desired_state.com_pos << predicted_x(0), predicted_y(0), RobotConfig::h;
    desired_state.com_vel << predicted_x(1), predicted_y(1), 0.0;
    desired_state.zmp_pos << predicted_x(2), predicted_y(2), 0.0;
    desired_state.zmp_vel << Xdz(0), Ydz(0), 0.0;
    desired_state.com_acc = (RobotConfig::eta * RobotConfig::eta) * (state.com_pos - state.zmp_pos);
    desired_state.com_acc(2) = 0.0;
    // ==================================================
}

}  // namespace ismpc
