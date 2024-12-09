#pragma once

#include <chrono>

#include "ismpc.h"
#include "tools/debug.h"
#include "tools/math/pose2.h"
#include "types/math_types.h"

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
    LipRobot robot;
    Reference reference;
    FootstepsPlan footsteps;

    // Libraries (Act as representations for modules, but treat other representations as if it were a
    // module)
    CostLib cost_lib;
    ConstraintLib constraint_lib;

    // Modules
    ReferenceProvider reference_provider;
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
    void update();

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
    LipRobot& get_robot();
    FrameInfo& get_frame_info();
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
