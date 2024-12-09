#include "modules/walk_engine.h"

namespace ismpc {

WalkEngine::WalkEngine()
    : frame_info(),
      robot(),
      reference(),
      footsteps(),
      cost_lib(reference, footsteps, robot),
      constraint_lib(footsteps, robot),
      reference_provider(frame_info, robot),
      planner(frame_info, robot, reference, cost_lib, constraint_lib),
      walk_state_provider(frame_info, robot, footsteps),
      swing_foot_provider(frame_info, robot, footsteps),
      mpc(frame_info, robot, footsteps, cost_lib, constraint_lib) {}

void WalkEngine::update_time() {
    frame_info.k += 1;
    frame_info.tk += delta;
}

void WalkEngine::update() {
    auto start = std::chrono::high_resolution_clock::now();
    reference_provider.update(reference);
    planner.update(footsteps);
    walk_state_provider.update(robot);
    swing_foot_provider.update(robot);
    auto end = std::chrono::high_resolution_clock::now();
    total_planner_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    start = std::chrono::high_resolution_clock::now();
    mpc.update(robot);
    end = std::chrono::high_resolution_clock::now();
    total_mpc_duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    history.push_back(robot.state);
    walk_history.push_back(robot.walk);
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

LipRobot& WalkEngine::get_robot() {
    return robot;
}

FrameInfo& WalkEngine::get_frame_info() {
    return frame_info;
}

const std::vector<State>& WalkEngine::get_history() const {
    return history;
}

const std::vector<WalkState>& WalkEngine::get_walk_history() const {
    return walk_history;
}

const std::vector<Pose2>& WalkEngine::get_footstep_history() const {
    return robot.walk.footstep_history;
}

const std::vector<Scalar>& WalkEngine::get_timestamp_history() const {
    return robot.walk.timestamp_history;
}

}  // namespace ismpc
