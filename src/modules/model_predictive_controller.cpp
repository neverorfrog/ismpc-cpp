#include "ismpc_cpp/modules/model_predictive_controller.h"

namespace ismpc {

ModelPredictiveController::ModelPredictiveController(const FrameInfo& frame_info, const State& state,
                                                     const FootstepPlan& plan)
    : frame_info(frame_info), state(state), plan(plan) {}

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
    // Integrate the lip velocities
    Scalar xdz = x_sol(1);
    Scalar ydz = y_sol(1);
    Vector3 predicted_x = state.lip.integrateX(xdz);
    Vector3 predicted_y = state.lip.integrateY(ydz);

    // Set desired state
    state.desired_lip.com_pos << predicted_x(0), predicted_y(0), RobotConfig::h;
    state.desired_lip.com_vel << predicted_x(1), predicted_y(1), 0.0;
    state.desired_lip.zmp_pos << predicted_x(2), predicted_y(2), 0.0;
    state.desired_lip.zmp_vel << xdz, ydz, 0.0;
    Vector3 com_acc = (eta * eta) * (state.desired_lip.com_pos - state.desired_lip.zmp_pos);
    state.desired_lip.com_acc << com_acc(0), com_acc(1), 0.0;

    state.lip_history.push_back(state.lip);
    state.left_foot_history.push_back(state.left_foot);
    state.right_foot_history.push_back(state.right_foot);
    // ==================================================

    // =============== FOOTSTEP UPDATE ==================

    // Switch support foot when the double support phase ends
    if (frame_info.tk >= state.footstep.end && state.support_phase == SupportPhase::DOUBLE) {
        state.fs_history.push_back(state.footstep);
        state.footstep = plan.footsteps[fs_index];
        fs_index++;
    }

    // Update the support phase info
    if (frame_info.tk >= state.footstep.ds_start)
        state.support_phase = SupportPhase::DOUBLE;
    else
        state.support_phase = SupportPhase::SINGLE;
    // ====================================================
}

}  // namespace ismpc
