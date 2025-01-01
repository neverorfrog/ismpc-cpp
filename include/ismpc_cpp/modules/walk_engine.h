#pragma once

#include <chrono>

#include "ismpc_cpp/ismpc.h"
#include "ismpc_cpp/tools/debug.h"
#include "ismpc_cpp/tools/math/pose2.h"
#include "ismpc_cpp/types/math_types.h"

namespace ismpc {

class WalkEngine {
   private:
    const Scalar delta = Config::delta;

   protected:
    // Simulation stuff
    FrameInfo frame_info;
    std::vector<State> history;
    std::vector<WalkState> walk_history;

    // Shared Representations (owned by me but passed to the modules in my constructor)
    Reference reference;
    FootstepsPlan footsteps;
    State state;
    State desired_state;
    WalkState walk;

    // Libraries (Act as representations for modules, are in fact modules without an update)
    FeetLib feet;
    CostLib cost;
    ConstraintLib constraint;

    // Modules
    FootstepsPlanProvider planner;
    WalkStateProvider walk_state_provider;
    SwingFootProvider swing_foot_provider;
    ModelPredictiveController mpc;

   public:
    WalkEngine();

    /**
     * @brief Updates the whole walk engine every delta seconds. Plans the footsteps and computes the
     * desired zmp velocity to be tracked by the com through the intrinsically stable mpc algorithm
     *
     */
    void update(State& desired_state);

    /**
     * @brief Updates time information (needed for the python binding)
     *
     */
    void update_time();

    /**
     * @brief Set the reference velocity object
     *
     * @param vx
     * @param vy
     * @param omega
     */
    void set_reference_velocity(Scalar vx, Scalar vy, Scalar omega);

    // Getters
    const FootstepsPlan& get_footsteps() const;
    const Reference& get_reference() const;
    const FrameInfo& get_frame_info() const;
    const State& get_state() const;
    void set_state(const State& state);
    State& get_desired_state();
    const WalkState& get_walk_state() const;
    const std::vector<State>& get_history() const;
    const std::vector<WalkState>& get_walk_history() const;
    const std::vector<Pose2>& get_footstep_history() const;
    const std::vector<Scalar>& get_timestamp_history() const;

    void print() const;

    // Timing
    Scalar total_planner_duration = 0.0;
    Scalar total_mpc_duration = 0.0;
};

}  // namespace ismpc
