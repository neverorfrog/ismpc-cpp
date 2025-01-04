#include "ismpc_cpp/modules/model_predictive_controller.h"

namespace ismpc {

ModelPredictiveController::ModelPredictiveController(const FrameInfo& frame_info, const State& state,
                                                     const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

void ModelPredictiveController::update(State& state) {
    //     // ================== PREPROCESSING ==================
    //     start = std::chrono::high_resolution_clock::now();

    //     // State related stuff
    //     isize d = 2 * numC;  // number of primal variables (xdz, ydz)
    //     Vector3 lipx = state.getLipx();
    //     Vector3 lipy = state.getLipy();

    //     // Cost Function
    //     Cost mpc_cost = cost.getMpcCost();

    //     // ZMP Constraint (2 * numC inequalities)
    //     zmp_constraint = constraint.getZmpConstraint(lipx, lipy);
    //     zmp_velocity_constraint = constraint.getZmpVelocityConstraint();

    //     // Combine inequality constraints
    //     isize n_in = 2 * dimz;
    //     C = Matrix::Zero(n_in, d);
    //     C.block(0, 0, dimz, dimz) = zmp_constraint.C;
    //     C.block(dimz, dimz, dimk, dimk) = kinematic_constraint.C;
    //     C.block(dimz + dimk, 0, dimz, dimz) = zmp_velocity_constraint.C;
    //     l = VectorX::Zero(n_in);
    //     l.segment(0, dimz) << zmp_constraint.l;
    //     l.segment(dimz, dimk) << kinematic_constraint.l;
    //     l.segment(dimz + dimk, dimz) << zmp_velocity_constraint.l;
    //     u = VectorX::Zero(n_in);
    //     u.segment(0, dimz) << zmp_constraint.u;
    //     u.segment(dimz, dimk) << kinematic_constraint.u;
    //     u.segment(dimz + dimk, dimz) << zmp_velocity_constraint.u;

    //     // Stability Constraint
    //     EqualityConstraint stability_constraint = constraint.getStabilityConstraint(lipx, lipy);
    //     end = std::chrono::high_resolution_clock::now();
    //     total_mpc_preprocessing_duration += std::chrono::duration_cast<std::chrono::microseconds>(end -
    //     start).count();
    //     // ==================================================

    //     // ================== SOLVE QP ==================
    //     QP<Scalar> qp = QP<Scalar>(d, n_eq, n_in);
    //     start = std::chrono::high_resolution_clock::now();
    //     qp = QP<Scalar>(d, n_eq, n_in);
    //     qp.settings.max_iter = 50;
    //     qp.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
    //     qp.settings.verbose = false;
    //     qp.settings.compute_timings = true;
    //     qp.init(mpc_cost.H, mpc_cost.g, stability_constraint.A, stability_constraint.b, C, l, u);
    //     qp.solve();

    //     if (qp.results.info.status != PROXQP_SOLVED) {
    //         throw std::runtime_error("QP solver failed to find a solution.");
    //     }
    //     end = std::chrono::high_resolution_clock::now();
    //     total_mpc_qp_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    //     // =============================================

    //     // ================== POSTPROCESSING ==================
    //     // Extract the solution
    //     Xdz = qp.results.x.segment(0, numC);
    //     Ydz = qp.results.x.segment(numC, numC);
    //     Xf = qp.results.x.segment(2 * numC, Fprime);
    //     Yf = qp.results.x.segment(2 * numC + Fprime, Fprime);

    //     // Integrate the lip velocities
    //     Vector3 predicted_x = state.getNextLipx(Xdz(0));
    //     Vector3 predicted_y = state.getNextLipy(Ydz(0));

    //     // Set desired state
    //     desired_state.com_pos << predicted_x(0), predicted_y(0), RobotConfig::h;
    //     desired_state.com_vel << predicted_x(1), predicted_y(1), 0.0;
    //     desired_state.zmp_pos << predicted_x(2), predicted_y(2), 0.0;
    //     desired_state.zmp_vel << Xdz(0), Ydz(0), 0.0;
    //     desired_state.com_acc = (RobotConfig::eta * RobotConfig::eta) * (state.com_pos - state.zmp_pos);
    //     desired_state.com_acc(2) = 0.0;
    //     // ==================================================
}

// Cost ModelPredictiveController::getCost() const {
//     // Cost Matrix
//     int d = 2 * numC + 2 * Fprime;
//     Matrix H = Matrix::Identity(d, d);
//     Matrix H1 = Matrix::Identity(2 * numC, 2 * numC) * 2;             // Square Sum of xdz and ydz
//     Matrix H2 = Matrix::Identity(2 * Fprime, 2 * Fprime) * 2 * beta;  // Square Sum of xf and yf
//     H.block(0, 0, 2 * numC, 2 * numC) = H1;
//     H.block(2 * numC, 2 * numC, 2 * Fprime, 2 * Fprime) = H2;

//     // Cost vector
//     VectorX g = VectorX::Zero(d);
//     for (int i = 2 * numC, j = 0; j < Fprime; ++i, ++j) {
//         g(i) = -2 * footsteps.x(j) * beta;
//         g(i + Fprime) = -2 * footsteps.y(j) * beta;
//     }

//     return Cost(H, g);
// }

// InequalityConstraint ConstraintLib::getZmpConstraint(const Vector3& lipx, const Vector3& lipy) const {
//     int d = 2 * numC;
//     Matrix C = Matrix::Zero(d, d);
//     VectorX lb = VectorX::Zero(d);
//     VectorX ub = VectorX::Zero(d);

//     VectorX theta = footsteps.get_zmp_midpoints_theta();
//     VectorX cos_theta = theta.array().cos();
//     VectorX sin_theta = theta.array().sin();
//     VectorX xf = footsteps.get_zmp_midpoints_x();
//     VectorX yf = footsteps.get_zmp_midpoints_y();

//     Scalar xz = lipx(2);
//     Scalar yz = lipy(2);

//     for (int i = 0; i < numC; ++i) {
//         C.block(i, 0, 1, i + 1).setConstant(delta * cos_theta(i));
//         C.block(i, numC, 1, i + 1).setConstant(delta * sin_theta(i));
//         C.block(i + numC, 0, 1, i + 1).setConstant(-delta * sin_theta(i));
//         C.block(i + numC, numC, 1, i + 1).setConstant(delta * cos_theta(i));

//         Scalar x_displacement = (xf(i) - xz);
//         Scalar y_displacement = (yf(i) - yz);
//         Scalar rotated_x_displacement = cos_theta(i) * x_displacement + sin_theta(i) * y_displacement;
//         Scalar rotated_y_displacement = -sin_theta(i) * x_displacement + cos_theta(i) * y_displacement;

//         ub(i) = rotated_x_displacement + 0.5 * dxz;
//         ub(i + numC) = rotated_y_displacement + 0.5 * dyz;
//         lb(i) = rotated_x_displacement - 0.5 * dxz;
//         lb(i + numC) = rotated_y_displacement - 0.5 * dyz;
//     }

//     return InequalityConstraint(C, lb, ub);
// }

// InequalityConstraint ConstraintLib::getZmpVelocityConstraint() const {
//     int n_in = 2 * numC;
//     Matrix C = Matrix::Identity(n_in, n_in);
//     VectorX lb = VectorX::Ones(n_in);
//     VectorX ub = VectorX::Ones(n_in);
//     lb.segment(0, numC) *= -zmp_vx_max;
//     lb.segment(numC, numC) *= -zmp_vy_max;
//     ub.segment(0, numC) *= zmp_vx_max;
//     ub.segment(numC, numC) *= zmp_vy_max;

//     return InequalityConstraint(C, lb, ub);
// }

// EqualityConstraint ConstraintLib::getStabilityConstraint(const Vector3& lipx, const Vector3& lipy) const {
//     int d = 2 * numC + 2 * Fprime;
//     Matrix A = Matrix::Zero(2, d);
//     VectorX b = VectorX::Zero(2);

//     // Summation of zmp velocities
//     for (int i = 0; i < numC; ++i) {
//         A(0, i) = std::exp(-i * delta * eta);
//         A(1, i + numC) = std::exp(-i * delta * eta);
//     }

//     Scalar xu = lipx(0) + lipx(1) / eta;
//     Scalar yu = lipy(0) + lipy(1) / eta;

//     Scalar temp = eta / (1 - std::exp(-delta * eta));
//     b(0) = temp * (xu - lipx(2));
//     b(1) = temp * (yu - lipy(2));

//     if (tail_type == TailType::PERIODIC) {
//         b(0) *= (1 - std::exp(-delta * eta * numC));
//         b(1) *= (1 - std::exp(-delta * eta * numC));
//     } else if (tail_type == TailType::ANTICIPATIVE) {
//         VectorX xf = footsteps.get_zmp_midpoints_x();
//         VectorX yf = footsteps.get_zmp_midpoints_y();

//         Scalar xdz, ydz, exp_term;
//         for (int i = numC; i < numP; ++i) {
//             exp_term = std::exp(-i * delta * eta);
//             xdz = (xf(i) - xf(i - 1)) / delta;
//             ydz = (yf(i) - yf(i - 1)) / delta;

//             b(0) -= exp_term * xdz;
//             b(1) -= exp_term * ydz;
//         }
//     }

//     return EqualityConstraint(A, b);
// }

}  // namespace ismpc
