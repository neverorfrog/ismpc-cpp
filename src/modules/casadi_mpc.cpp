#include "ismpc_cpp/modules/casadi_mpc.h"

#include <core/generic_matrix.hpp>
#include <core/mx.hpp>

namespace ismpc {

CasadiMPC::CasadiMPC(const FrameInfo& frame_info, const State& state, const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {
    opti = casadi::Opti("conic");
    casadi::Dict options;
    casadi::Dict solver_options;
    options["expand"] = true;
    options["print_time"] = false;
    solver_options["max_iter"] = 1000;
    opti.solver("proxqp", options, solver_options);

    X = opti.variable(6, numC + 1);
    U = opti.variable(2, numC);

    x0_param = opti.parameter(6);
    zmp_x_param = opti.parameter(numC);
    zmp_y_param = opti.parameter(numC);

    opti.subject_to(X(all, 0) == x0_param);
    for (int i = 0; i < numC; i++) {
        opti.subject_to(X(all, i + 1) == X(all, i) + delta * f(X(all, i), U(all, i)));
    }

    cost = casadi::MX::sumsqr(U(0, all)) + casadi::MX::sumsqr(U(1, all)) +
           100 * casadi::MX::sumsqr(X(2, all_but_first).T() - zmp_x_param) +
           100 * casadi::MX::sumsqr(X(5, all_but_first).T() - zmp_y_param);

    opti.subject_to(X(2, all_but_first).T() <= zmp_x_param + dxz / 2);
    opti.subject_to(X(2, all_but_first).T() >= zmp_x_param - dxz / 2);
    opti.subject_to(X(5, all_but_first).T() <= zmp_y_param + dyz / 2);
    opti.subject_to(X(5, all_but_first).T() >= zmp_y_param - dyz / 2);

    opti.subject_to(X(1, 0) + std::pow(eta, 3) + (X(0, 0) - X(2, 0)) ==
                    X(1, numC) + std::pow(eta, 3) * (X(0, numC) - X(2, numC)));
    opti.subject_to(X(4, 0) + std::pow(eta, 3) + (X(3, 0) - X(5, 0)) ==
                    X(4, numC) + std::pow(eta, 3) * (X(3, numC) - X(5, numC)));

    opti.minimize(cost);

    std::cout << "Casadi MPC Initialized" << std::endl;
}

void CasadiMPC::update(State& state) {
    VectorX x0 = state.lip.getState();
    for (int i = 0; i < 6; i++) {
        opti.set_value(x0_param(i), x0(i));
    }

    VectorX zmp_x = plan.zmp_midpoints_x;
    VectorX zmp_y = plan.zmp_midpoints_y;

    std::cout << "ZMP_X: " << zmp_x.transpose().format(Config::CleanFmt) << std::endl;
    std::cout << "ZMP_Y: " << zmp_y.transpose().format(Config::CleanFmt) << std::endl;

    for (int i = 0; i < numC; i++) {
        opti.set_value(zmp_x_param(i), zmp_x(i));
        opti.set_value(zmp_y_param(i), zmp_y(i));
    }

    casadi::OptiSol sol = opti.solve();
    opti.set_initial(X, sol.value(X));
    opti.set_initial(U, sol.value(U));

    std::vector<Scalar> x_sol = sol.value(X(all, 1)).get_elements();
    std::vector<Scalar> u_sol = sol.value(U(all, 0)).get_elements();

    state.desired_lip.com_pos << x_sol[0], x_sol[3], 0.75;
    state.desired_lip.com_vel << x_sol[1], x_sol[4], 0.0;
    state.desired_lip.zmp_pos << x_sol[2], x_sol[5], 0.0;
    state.desired_lip.zmp_vel << u_sol[0], u_sol[1], 0.0;
    VectorX com_acc =
        (RobotConfig::eta * RobotConfig::eta) * (state.desired_lip.com_pos - state.desired_lip.zmp_pos);
    state.desired_lip.com_acc << com_acc(0), com_acc(1), 0.0;

    state.lip_history.push_back(state.lip);
    state.left_foot_history.push_back(state.left_foot);
    state.right_foot_history.push_back(state.right_foot);

    // =============== FOOTSTEP UPDATE ==================

    // Switch support foot when the double support phase ends
    if (frame_info.tk >= state.footstep.end && state.support_phase == SupportPhase::DOUBLE) {
        state.fs_history.push_back(state.footstep);
        state.footstep = plan.footsteps[fs_index];
        fs_index++;
    }

    // std::cout << "TIME: " << frame_info.tk << std::endl;

    // Update the support phase info
    if (frame_info.tk >= state.footstep.ds_start)
        state.support_phase = SupportPhase::DOUBLE;
    else
        state.support_phase = SupportPhase::SINGLE;
    // ====================================================
}

casadi::MX CasadiMPC::f(const casadi::MX& x, const casadi::MX& u) const {
    casadi::MX x_next = casadi::MX::mtimes(A, x(first_half)) + casadi::MX::mtimes(B, u(0));
    casadi::MX y_next = casadi::MX::mtimes(A, x(second_half)) + casadi::MX::mtimes(B, u(1));
    return casadi::MX::vertcat({x_next, y_next});
}

}  // namespace ismpc
