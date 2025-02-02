#include <ismpc_cpp/types/ismpc_qp.h>
#include <iostream>
#include <string>

namespace ismpc {

IsmpcQp::IsmpcQp() {
    // Options
    qp.settings.max_iter = 500;
    qp.settings.initial_guess = InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
    qp.settings.verbose = false;
    qp.settings.compute_timings = true;

    // Cost Function
    cost = Cost(d);
    for (int i = 0; i < numC; ++i) {
        cost.H(nv * i + nl, nv * i + nl) = 2;                                 // dzmp
        cost.H(nv * (i + 1) + (nl - 1), nv * (i + 1) + (nl - 1)) = 2 * beta;  // zmp
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
    Ak(0, 1) = delta;  // current com (or zmp) vel to next com pos (or zmp pos)
    if (nl == 3) {
        Ak(1, 0) = std::pow(eta, 2) * delta;   // current com pos to next com vel
        Ak(1, 2) = -std::pow(eta, 2) * delta;  // current zmp pos to next com vel
        Ak(2, 3) = delta;                      // current zmp vel to next zmp pos
    }
    for (int i = 0; i < numC; ++i) {
        model_constraint.A.block(nl * i, nv * i, nl, nv + nl) = Ak;
    }
    A.block(nl, 0, nl * numC, d) = model_constraint.A;
    b.segment(nl, nl + nl * numC) = model_constraint.b;

    // Stability Constraint
    stability_constraint = EqualityConstraint(1, d);
    if (nl == 1) {
        for (int i = 0; i < numC - 1; ++i) {  // Summation of zmp velocities
            stability_constraint.A(0, 2 * i + 1) = std::exp(-i * delta * eta);
        }
    } else if (nl == 3) {
        stability_constraint.A(0, 0) = std::pow(eta, 3);
        stability_constraint.A(0, 1) = 1;
        stability_constraint.A(0, 2) = -std::pow(eta, 3);
        stability_constraint.A(0, d - 3) = -std::pow(eta, 3);
        stability_constraint.A(0, d - 2) = -1;
        stability_constraint.A(0, d - 1) = std::pow(eta, 3);
    }
    A.block(nl + nl * numC, 0, 1, d) = stability_constraint.A;

    // Zmp Constraint
    zmp_constraint = InequalityConstraint(n_in, d);
    for (int i = 0; i < numC; ++i) {
        zmp_constraint.C(2 * i, nv * (i + 1) + (nl - 1)) = 1;  // zmp pos
        zmp_constraint.C(2 * i + 1, nv * i + nl) = 1;          // zmp vel
    }

    qp.init(cost.H.sparseView(), cost.g, A.sparseView(), b, zmp_constraint.C.sparseView(), zmp_constraint.l,
            zmp_constraint.u);
}

void IsmpcQp::update(const Vector3 &lip, const VectorX &mc) {
    // Cost Function
    for (int i = 0; i < numC + 1; ++i) {
        cost.g(nv * (i + 1) + (nl - 1)) = -2 * beta * mc(i);
    }

    // Initial Constraint
    initial_constraint.b(nl - 1) = lip(2);
    if (nl == 3) {
        initial_constraint.b(0) = lip(0);
        initial_constraint.b(1) = lip(1);
    }
    b.segment(0, nl) = initial_constraint.b;

    // Stability Constraint
    if (nl == 1) {
        Scalar unstable = lip(0) + lip(1) / eta;
        Scalar temp = eta / (1 - std::exp(-delta * eta));
        if (tail_type == TailType::PERIODIC) {
            stability_constraint.b << temp * (unstable - lip(2)) * (1 - std::exp(-delta * eta * numC));
        } else if (tail_type == TailType::TRUNCATED) {
            stability_constraint.b << temp * (unstable - lip(2));
        }
        b.segment(nl + nl * numC, 1) = stability_constraint.b;
    } else if (nl == 3) {
        b.segment(nl + nl * numC, 1) << 0;
    }

    // ZMP Constraint
    for (int i = 0; i < numC; ++i) {
        zmp_constraint.l(2 * i) = mc(i) - 0.5 * dxz;
        zmp_constraint.u(2 * i) = mc(i) + 0.5 * dxz;
        zmp_constraint.l(2 * i + 1) = -zmp_vx_max;
        zmp_constraint.u(2 * i + 1) = zmp_vx_max;
    }
}

bool IsmpcQp::solve() {
    qp.update(cost.H.sparseView(), cost.g, A.sparseView(), b, zmp_constraint.C.sparseView(), zmp_constraint.l,
              zmp_constraint.u);
    qp.solve();

    if (qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
        std::cout << "QP solver failed to find a solution." << std::endl;
        return false;
    }

    sol = qp.results.x;
    return true;
}

}  // namespace ismpc
