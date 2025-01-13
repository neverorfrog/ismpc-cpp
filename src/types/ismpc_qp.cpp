#include <ismpc_cpp/types/ismpc_qp.h>

namespace ismpc {

IsmpcQp::IsmpcQp() {
    // Options
    qp.settings.max_iter = 100;
    qp.settings.initial_guess = InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;
    qp.settings.eps_abs = 1e-3;
    qp.settings.eps_rel = 1e-3;

    // Cost Function
    cost = Cost(d);
    cost.H = Matrix::Identity(d, d) * 2;  // Square Sum of xz, xdz
    cost.H(0, 0) = 0;                     // xz_0
    for (int i = 1; i < numC; ++i) {
        cost.H(2 * i, 2 * i) = 2 * beta;  // xz_i
    }
    cost.H(numC, numC) = 2 * beta;  // xz_n

    // Initial Constraint
    initial_constraint = EqualityConstraint(1, d);
    initial_constraint.A(0, 0) = 1;
    A.block(0, 0, 1, d) = initial_constraint.A;

    // Model Constraint (Ax = b) -> b are all zeros
    model_constraint = EqualityConstraint(numC, d);
    for (int i = 0; i < numC; ++i) {
        model_constraint.A(i, 2 * i) = 1;          // current zmp
        model_constraint.A(i, 2 * i + 1) = delta;  // current zmp_dot
        model_constraint.A(i, 2 * i + 2) = -1;     // next zmp
    }
    A.block(1, 0, numC, d) = model_constraint.A;

    // Stability Constraint
    stability_constraint = EqualityConstraint(1, d);
    for (int i = 1; i < numC; ++i) {  // Summation of zmp velocities
        stability_constraint.A(0, 2 * i + 1) = std::exp(-(i - 1) * delta * eta);
    }
    A.block(1 + numC, 0, 1, d) = stability_constraint.A;

    // Zmp Constraint
    zmp_constraint = InequalityConstraint(d, d);
    zmp_constraint.C = Matrix::Identity(d, d);
}

void IsmpcQp::update(const Vector3 &lip, const VectorX &mc) {
    // Cost Function
    for (int i = 1; i < numC; ++i) {
        cost.g(2 * i) = -2 * beta * mc(i - 1);
    }
    cost.g(numC) = -2 * beta * mc(numC - 1);

    // Initial Constraint
    Scalar zmp = lip(2);
    initial_constraint.b(0) = zmp;

    // Model Constraint
    model_constraint.b = VectorX::Zero(numC);

    // Stability Constraint
    Scalar unstable = lip(0) + lip(1) / eta;
    Scalar temp = eta / (1 - std::exp(-delta * eta));
    if (tail_type == TailType::PERIODIC) {
        stability_constraint.b(0) = temp * (unstable - zmp) * (1 - std::exp(-delta * eta * numC));
    } else if (tail_type == TailType::TRUNCATED) {
        stability_constraint.b(0) = temp * (unstable - zmp);
    }
    std::cout << "STABILITY CONSTRAINTS B: " << stability_constraint.b.transpose().format(Config::CleanFmt)
              << std::endl;

    // Fuse equality constraints
    b.segment(0, 1) = initial_constraint.b;
    b.segment(1, 1 + numC) = model_constraint.b;
    b.segment(1 + numC, 1) = stability_constraint.b;

    // ZMP Constraint
    for (int i = 1; i < numC; ++i) {
        zmp_constraint.u(2 * i) = mc(i - 1) - zmp + 0.5 * dxz;
        zmp_constraint.l(2 * i) = mc(i - 1) - zmp - 0.5 * dxz;
        zmp_constraint.u(2 * i + 1) = zmp_vx_max;
        zmp_constraint.l(2 * i + 1) = -zmp_vx_max;
    }
    zmp_constraint.u(numC) = mc(numC - 1) + 0.5 * dxz;
    zmp_constraint.l(numC) = mc(numC - 1) - 0.5 * dxz;
}

VectorX IsmpcQp::solve() {
    std::cout << "SOLVING QP" << std::endl;
    qp.update(cost.H, cost.g, A, b, zmp_constraint.C, zmp_constraint.l, zmp_constraint.u);
    qp.solve();
    sol = qp.results.x;

    if (qp.results.info.status != PROXQP_SOLVED) {
        throw std::runtime_error("QP solver failed to find a solution.");
    }

    return sol;
}


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

}  // namespace ismpc