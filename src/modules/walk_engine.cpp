#include "ismpc_cpp/modules/walk_engine.h"

namespace ismpc {

WalkEngine::WalkEngine()
    : frame_info(),
      reference(),
      footsteps(),
      state(),
      walk(),
      feet(state, walk),
      cost(reference, footsteps, feet, walk),
      constraint(footsteps, feet),
      reference_provider(frame_info, state),
      planner(frame_info, state, walk, reference, feet, cost, constraint),
      walk_state_provider(frame_info, state, walk, feet, footsteps),
      swing_foot_provider(frame_info, walk, feet, footsteps),
      mpc(frame_info, state, walk, footsteps, feet, cost, constraint) {}

void WalkEngine::update_time() {
    frame_info.k += 1;
    frame_info.tk += delta;
}

void WalkEngine::update() {
    auto start = std::chrono::high_resolution_clock::now();
    reference_provider.update(reference);
    planner.update(footsteps);
    walk_state_provider.update(walk);
    swing_foot_provider.update(state);
    auto end = std::chrono::high_resolution_clock::now();
    total_planner_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    start = std::chrono::high_resolution_clock::now();
    mpc.update(state);
    end = std::chrono::high_resolution_clock::now();
    total_mpc_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    history.push_back(state);
    walk_history.push_back(walk);
    update_time();
}

void WalkEngine::set_reference_velocity(Scalar vx, Scalar vy, Scalar omega) {
    reference.set_velocity(vx, vy, omega);
}

void WalkEngine::print() const {
    PRINT("State: \n" << robot.state);
    PRINT("Walk state: \n" << robot.walk);
    PRINT("Next planned footstep: " << footsteps.theta(0) << " " << footsteps.x(0) << " " << footsteps.y(0));
    PRINT("");
}

const Reference& WalkEngine::get_reference() const {
    return reference;
}

const FootstepsPlan& WalkEngine::get_footsteps() const {
    return footsteps;
}

const State& WalkEngine::get_state() const {
    return state;
}

const WalkState& WalkEngine::get_walk_state() const {
    return walk;
}

const FrameInfo& WalkEngine::get_frame_info() const {
    return frame_info;
}

const std::vector<State>& WalkEngine::get_history() const {
    return history;
}

const std::vector<WalkState>& WalkEngine::get_walk_history() const {
    return walk_history;
}

const std::vector<Pose2>& WalkEngine::get_footstep_history() const {
    return walk.footstep_history;
}

const std::vector<Scalar>& WalkEngine::get_timestamp_history() const {
    return walk.timestamp_history;
}

}  // namespace ismpc
