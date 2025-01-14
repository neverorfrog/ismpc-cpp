#include "ismpc_cpp/dart/qp_solver.h"

namespace ismpc {

QPSolver::QPSolver(int d) {
    this->d = d;
    opti = casadi::Opti("conic");
    casadi::Dict options;
    options["expand"] = true;
    options["print_time"] = false;
    opti.solver("proxqp", options);

    // Define decision variables
    x = opti.variable(d);

    // Cost function
    H = opti.parameter(d, d);
    g = opti.parameter(d);
    MX cost = 0.5 * MX::mtimes(std::vector<MX>{x.T(), H, x}) + MX::mtimes(std::vector<MX>{g.T(), x});
    opti.minimize(cost);
}

void QPSolver::init(const DM& H, const DM& g) {
    opti.set_value(this->H, H);
    opti.set_value(this->g, g);
};

VectorX QPSolver::solve() {
    casadi::OptiSol sol = opti.solve();

    DM x_sol = sol.value(x);

    VectorX x_sol_vec = VectorX::Zero(d);
    for (int i = 0; i < d; i++) {
        x_sol_vec(i) = x_sol->at(i);
    }

    return x_sol_vec;
}

}  // namespace ismpc
