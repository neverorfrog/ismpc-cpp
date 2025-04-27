#include "ismpc_cpp/modules/model_predictive_controller.h"

namespace ismpc {

void ModelPredictiveController::update(State& state) {
    // ================== PREPROCESSING ===================
    start = std::chrono::high_resolution_clock::now();
    qpx.update(state.lip.getX(), plan.zmp_midpoints_x);
    qpy.update(state.lip.getY(), plan.zmp_midpoints_y);
    end = std::chrono::high_resolution_clock::now();
    total_mpc_preprocessing_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // ===================================================

    // ================== SOLVE QP =======================
    start = std::chrono::high_resolution_clock::now();
    bool x_ok = qpx.solve();
    bool y_ok = qpy.solve();
    if (!x_ok || !y_ok) {
        throw std::runtime_error("QP Solver failed");
    }
    x_sol = qpx.getSol();
    y_sol = qpy.getSol();
    end = std::chrono::high_resolution_clock::now();
    total_mpc_qp_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // ====================================================

    // ================== POSTPROCESSING ==================
    if (nl == 1) {
        // Integrate the lip velocities
        xdz = x_sol(1);
        ydz = y_sol(1);
        xz = x_sol(2);
        yz = y_sol(2);
        Vector3 predicted_x = state.lip.integrateX(xdz);
        Vector3 predicted_y = state.lip.integrateY(ydz);
        xc = predicted_x(0);
        yc = predicted_y(0);
        xdc = predicted_x(1);
        ydc = predicted_y(1);
    } else if (nl == 3) {
        xdz = x_sol(3);
        ydz = y_sol(3);
        xc = x_sol(4);
        yc = y_sol(4);
        xdc = x_sol(5);
        ydc = y_sol(5);
        xz = x_sol(6);
        yz = y_sol(6);
    }

    // Set desired state
    state.desired_lip.com_pos << xc, yc, h;
    state.desired_lip.com_vel << xdc, ydc, 0.0;
    state.desired_lip.zmp_pos << xz, yz, 0.0;
    state.desired_lip.zmp_vel << xdz, ydz, 0.0;
    state.desired_lip.com_acc(0) = std::pow(eta, 2) * (xc - xz);
    state.desired_lip.com_acc(1) = std::pow(eta, 2) * (yc - yz);
    state.desired_lip.com_acc(2) = 0.00;

    // The desired zmp velocity is actually a command, so it must also go directly into the state
    state.lip.zmp_vel = state.desired_lip.zmp_vel;

    state.lip_history.push_back(state.lip);
    state.left_foot_history.push_back(state.left_foot);
    state.right_foot_history.push_back(state.right_foot);
}

}  // namespace ismpc
