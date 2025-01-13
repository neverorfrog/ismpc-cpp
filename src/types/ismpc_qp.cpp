#include <ismpc_cpp/types/ismpc_qp.h>

#include "ismpc_cpp/tools/config/config.h"

namespace ismpc {

IsmpcQp::IsmpcQp() {
    // Options
    qp.settings.max_iter = 100;
    qp.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;
    qp.settings.eps_abs = 1e-3;
    qp.settings.eps_rel = 1e-3;

    // Cost Function
    cost = Cost(d);
    for (int i = 0; i < numC; ++i) {
        cost.H(nv * i + 3, nv * i + 3) = 2;                     // dzmp
        cost.H(nv * (i + 1) + 2, nv * (i + 1) + 2) = 2 * beta;  // zmp
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
    Ak(0, 1) = delta;                                         // current com vel
    Ak(2, 3) = delta;                                         // current zmp vel
    Ak(1, 0) = std::pow(eta, 2) * delta;                      // current com pos
    Ak(1, 2) = -std::pow(eta, 2) * delta;                     // current zmp pos
    for (int i = 0; i < numC; ++i) {
        model_constraint.A.block(nl * i, nv * i, nl, nv + nl) = Ak;
    }
    A.block(nl, 0, nl * numC, d) = model_constraint.A;

    // Stability Constraint
    stability_constraint = EqualityConstraint(1, d);
    stability_constraint.A(0, 0) = eta;              // initial com pos
    stability_constraint.A(0, 2) = -eta;             // initial zmp pos
    stability_constraint.A(0, 1) = 1;                // initial com vel
    stability_constraint.A(0, nv * numC) = -eta;     // final com pos
    stability_constraint.A(0, nv * numC + 2) = eta;  // final zmp pos
    stability_constraint.A(0, nv * numC + 1) = -1;   // final com vel
    A.block(nl + nl * numC, 0, 1, d) = stability_constraint.A;

    // Zmp Constraint
    zmp_constraint = InequalityConstraint(n_in, d);
    for (int i = 1; i < numC + 1; ++i) {
        zmp_constraint.C(i, nv * i + 2) = 1;
    }
}

void IsmpcQp::update(const Vector3 &lip, const VectorX &mc) {
    // Cost Function
    for (int i = 0; i < numC; ++i) {
        cost.g(nv * (i+1) + 2) = -2 * beta * mc(i);
    }

    // Initial Constraint
    initial_constraint.b = lip;
    b.segment(0, nl) = initial_constraint.b;

    // ZMP Constraint
    // std::cout << "MOVING CONSTRAINT: \n" << mc.transpose().format(Config::CleanFmt) << std::endl;
    for (int i = 0; i < numC; ++i) {
        zmp_constraint.u(i+1) = mc(i) + 0.5 * dxz;
        zmp_constraint.l(i+1) = mc(i) - 0.5 * dxz;
    }
    // std::cout << "ZMP CONSTRAINT CONSTRAINT: \n" << zmp_constraint.C.format(Config::CleanFmt) << std::endl;
    // std::cout << "ZMP CONSTRAINT UPPER: \n" << zmp_constraint.u.transpose().format(Config::CleanFmt) << std::endl;
    // std::cout << "ZMP CONSTRAINT LOWER: \n" << zmp_constraint.l.transpose().format(Config::CleanFmt) << std::endl;
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

}  // namespace ismpc