#include <ismpc_cpp/types/ismpc_qp.h>

#include "ismpc_cpp/tools/config/config.h"

namespace ismpc {

IsmpcQp::IsmpcQp() {
    // Options
    qp.settings.max_iter = 1000;
    qp.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;

    // Cost Function
    cost = Cost(d);
    for (int i = 0; i < numC; ++i) {
        cost.H(nv * i + nl, nv * i + nl) = 2;           // dzmp
        cost.H(nv * (i + 1), nv * (i + 1)) = 2 * beta;  // zmp
    }

    // Initial Constraint
    initial_constraint = EqualityConstraint(nl, d);
    initial_constraint.A.block(0, 0, nl, nl) = Matrix::Identity(nl, nl);
    A.block(0, 0, nl, d) = initial_constraint.A;

    // Model Constraint (Ax = b) -> b are all zeros
    model_constraint = EqualityConstraint(nl * numC, d);
    Matrix Ak = Matrix::Zero(nl, nv + nl);
    Ak.block(0, 0, nl, nl) = Matrix::Identity(nl, nl);        // current lip state
    Ak.block(0, nv, nl, nl) = -1 * Matrix::Identity(nl, nl);  // next lip state
    Ak(0, 1) = delta;                                         // current zmp vel
    for (int i = 0; i < numC; ++i) {
        model_constraint.A.block(nl * i, nv * i, nl, nv + nl) = Ak;
    }
    A.block(nl, 0, nl * numC, d) = model_constraint.A;

    // Stability Constraint
    stability_constraint = EqualityConstraint(1, d);
    for (int i = 0; i < numC - 1; ++i) {  // Summation of zmp velocities
        stability_constraint.A(0, 2 * i + 1) = std::exp(-i * delta * eta);
    }
    A.block(nl + nl * numC, 0, 1, d) = stability_constraint.A;

    // Zmp Constraint
    zmp_constraint = InequalityConstraint(n_in, d);
    for (int i = 0; i < numC; ++i) {
        zmp_constraint.C(2 * i, nv * (i + 1)) = 1;
        zmp_constraint.C(2 * i + 1, 2 * i + 1) = 1;
    }
}

void IsmpcQp::update(const Vector3 &lip, const VectorX &mc) {
    // Cost Function
    for (int i = 0; i < numC + 1; ++i) {
        cost.g(nv * (i + 1)) = -2 * beta * mc(i);
    }

    // Initial Constraint
    Scalar zmp = lip(2);
    initial_constraint.b(0) = zmp;
    b.segment(0, nl) = initial_constraint.b;

    // Stability Constraint
    Scalar unstable = lip(0) + lip(1) / eta;
    Scalar temp = eta / (1 - std::exp(-delta * eta));
    if (tail_type == TailType::PERIODIC) {
        stability_constraint.b << temp * (unstable - zmp) * (1 - std::exp(-delta * eta * numC));
    } else if (tail_type == TailType::TRUNCATED) {
        stability_constraint.b << temp * (unstable - zmp);
    }
    b.segment(nl + nl * numC, 1) = stability_constraint.b;

    // ZMP Constraint
    for (int i = 0; i < numC; ++i) {
        zmp_constraint.l(2 * i) = mc(i) - 0.5 * dxz;
        zmp_constraint.u(2 * i) = mc(i) + 0.5 * dxz;
        zmp_constraint.l(2 * i + 1) = -zmp_vx_max;
        zmp_constraint.u(2 * i + 1) = zmp_vx_max;
    }
}

bool IsmpcQp::solve() {
    std::cout << "SOLVING QP" << std::endl;
    qp.update(cost.H, cost.g, A, b, zmp_constraint.C, zmp_constraint.l, zmp_constraint.u);
    qp.solve();

    if (qp.results.info.status != PROXQP_SOLVED) {
        std::cout << "QP solver failed to find a solution." << std::endl;
        return false;
    }

    sol = qp.results.x;
    return true;
}

}  // namespace ismpc