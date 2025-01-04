#include "ismpc_cpp/modules/walk_engine.h"

namespace ismpc {

WalkEngine::WalkEngine(const State& state)
    : state(state),
      frame_info(),
      reference(),
      plan(),
      planner(frame_info, reference, state, plan),
      generator(frame_info, state, plan),
      mpc(frame_info, state, plan) {}

void WalkEngine::update_time() {
    frame_info.k += 1;
    frame_info.tk += delta;
}

void WalkEngine::update(State& state) {
    auto start = std::chrono::high_resolution_clock::now();
    planner.update(plan);
    auto end = std::chrono::high_resolution_clock::now();
    total_planner_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // start = std::chrono::high_resolution_clock::now();
    // mpc.update(desired_state);
    // swing_foot_provider.update(desired_state);
    // end = std::chrono::high_resolution_clock::now();
    // total_mpc_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // history.push_back(state);
    update_time();
}

const Reference& WalkEngine::get_reference() const {
    return reference;
}

const FootstepPlan& WalkEngine::get_footsteps() const {
    return plan;
}

const State& WalkEngine::get_state() const {
    return state;
}

const FrameInfo& WalkEngine::get_frame_info() const {
    return frame_info;
}

const std::vector<State>& WalkEngine::get_history() const {
    return history;
}

}  // namespace ismpc
