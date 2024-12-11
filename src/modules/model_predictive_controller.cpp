#include "ismpc_cpp/modules/model_predictive_controller.h"

namespace ismpc {

ModelPredictiveController::ModelPredictiveController(const FrameInfo& frame_info, const LipRobot& robot,
                                                     const FootstepsPlan& footsteps, const CostLib& cost_lib,
                                                     const ConstraintLib& constraint_lib)
    : frame_info(frame_info),
      robot(robot),
      footsteps(footsteps),
      cost_lib(cost_lib),
      constraint_lib(constraint_lib) {}

void ModelPredictiveController::update(LipRobot& robot) {
    QP<Scalar> qp = solve_qp(robot);

    // Extract the solution
    isize Fprime = footsteps.num_controlled_footsteps;
    auto start = std::chrono::high_resolution_clock::now();
    Xdz = qp.results.x.segment(0, numC);
    Ydz = qp.results.x.segment(numC, numC);
    Xf = qp.results.x.segment(2 * numC, Fprime);
    Yf = qp.results.x.segment(2 * numC + Fprime, Fprime);

    // Update the lip state
    robot.state.update(Xdz(0), Ydz(0));

    auto end = std::chrono::high_resolution_clock::now();
    robot.total_mpc_postprocessing_duration +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

QP<Scalar> ModelPredictiveController::solve_qp(LipRobot& robot) {
    isize Fprime = footsteps.num_controlled_footsteps;
    isize d = 2 * numC + 2 * Fprime;  // number of primal variables (xdz, ydz, xf and yf)

    Vector3 lipx = robot.state.getLipx();
    Vector3 lipy = robot.state.getLipy();

    auto start = std::chrono::high_resolution_clock::now();
    // Cost Function
    Cost cost = cost_lib.getMpcCost();

    // Kinematic Constraint (2 * Fprime inequalities)
    isize dimk = 2 * Fprime;
    InequalityConstraint kinematic_constraint = constraint_lib.getKinematicConstraint(Fprime);

    // ZMP Constraint (2 * numC inequalities)
    isize dimz = 2 * numC;
    InequalityConstraint zmp_constraint = constraint_lib.getZmpConstraint(lipx, lipy);
    InequalityConstraint zmp_velocity_constraint = constraint_lib.getZmpVelocityConstraint();

    // Combine inequality constraints
    isize n_in = 2 * dimz + dimk;
    Matrix C = Matrix::Zero(n_in, d);
    C.block(0, 0, dimz, dimz) = zmp_constraint.C;
    C.block(dimz, dimz, dimk, dimk) = kinematic_constraint.C;
    C.block(dimz + dimk, 0, dimz, dimz) = zmp_velocity_constraint.C;
    VectorX l = VectorX::Zero(n_in);
    l.segment(0, dimz) << zmp_constraint.l;
    l.segment(dimz, dimk) << kinematic_constraint.l;
    l.segment(dimz + dimk, dimz) << zmp_velocity_constraint.l;
    VectorX u = VectorX::Zero(n_in);
    u.segment(0, dimz) << zmp_constraint.u;
    u.segment(dimz, dimk) << kinematic_constraint.u;
    u.segment(dimz + dimk, dimz) << zmp_velocity_constraint.u;

    // Stability Constraint
    isize n_eq = 2;  // number of equality constraints
    EqualityConstraint stability_constraint = constraint_lib.getStabilityConstraint(lipx, lipy);

    auto end = std::chrono::high_resolution_clock::now();
    robot.total_mpc_preprocessing_duration +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    QP<Scalar> qp = QP<Scalar>(d, n_eq, n_in);
    qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;

    start = std::chrono::high_resolution_clock::now();
    qp.init(cost.H, cost.g, stability_constraint.A, stability_constraint.b, C, l, u);
    qp.solve();
    end = std::chrono::high_resolution_clock::now();
    robot.total_mpc_qp_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    return qp;
}

}  // namespace ismpc
