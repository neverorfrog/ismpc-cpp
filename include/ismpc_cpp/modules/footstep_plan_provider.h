/**
 * @file FootstepPlanProvider.hpp
 * @brief Header file for the FootstepPlanProvider class.
 *
 * This file contains the declaration of the FootstepPlanProvider module. This module uses
 * trajectory optimization to generate the footstep candidates. These in turn
 * are sent to the Footsteps representation. This representation is then
 * accessed by the ISMPC module.
 */

#pragma once

#include <cmath>
#include <ctime>
#include <iostream>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/body_parts.h"
#include "ismpc_cpp/types/configs.h"
#include "ismpc_cpp/types/end_effector.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/optimization.h"
#include "ismpc_cpp/types/support_phase.h"

using namespace ismpc::Arithmetic;
namespace ismpc {

/**
 * @class FootstepPlanProvider
 * @brief Class responsible for generating candidate footstep poses for the IS-MPC controller.
 *
 * The FootstepsPlanProvider class uses the last footstep timestamp and the current time to generate
 * potential footstep locations that the IS-MPC controller can use to maintain balance and achieve
 * desired motion.
 */
class FootstepPlanProvider {
   private:
    const FrameInfo& frame_info;
    const Reference& reference;
    const State& state;
    const FootstepPlan& plan;

    // Utility stuff
    std::vector<Scalar> timestamps;
    std::vector<Scalar> theta_sequence;
    std::vector<Scalar> x_sequence;
    std::vector<Scalar> y_sequence;
    int F;
    Footstep current_footstep;
    bool in_double_support;

    // Parameters
    int numP;
    Scalar T_p, T_c, delta, T_bar, v_bar, alpha, ds_percentage, theta_max, dax, day, l, fs_duration;

    // TODO: Test
    Scalar last_plan_timestamp = 0.0;

    /**
     * @brief Compute the sign corresponding to the footstep index
     * @param j
     * @return int
     */
    int getFootstepSign(int j) const;

   public:
    FootstepPlanProvider(const FrameInfo& frame_info, const Reference& reference, const State& state,
                         const FootstepPlan& plan, const Params& params)
        : frame_info(frame_info),
          reference(reference),
          state(state),
          plan(plan),
          numP(params.mpc.P),
          T_p(params.mpc.T_p),
          T_c(params.mpc.T_c),
          delta(params.mpc.delta),
          T_bar(params.gait.T_bar),
          v_bar(params.gait.v_bar),
          alpha(params.gait.alpha),
          ds_percentage(params.gait.ds_percentage),
          theta_max(params.gait.theta_max),
          dax(params.gait.dax),
          day(params.gait.day),
          l(params.gait.l),
          fs_duration(params.gait.fs_duration){};

    /**
     * @brief Update the footstep plan. This function computes the timing,
     * theta and position sequence by tracking a virtual unicycle model.
     */
    void update(FootstepPlan& plan);

    void computePlan(FootstepPlan& plan);

    /**
     * @brief Get the duration of each footstep. Sets the timestamps in
     * class footsteps. Each timestamp indicates the instant in which one
     * foot lifts off the ground. The duration of each footstep is
     * calculated based on the reference velocity and some parameters
     * depending on the specific robot.
     */
    void computeTiming();

    /**
     * @brief Obtain the optimal theta trajectory by solving a QP problem
     */
    void computeThetaSequence();

    /**
     * @brief Obtain the optimal position trajectory (x and y coordinates) by solving a QP problem
     */
    void computePositionSequence();

    /**
     * @brief Get the Kinematic Constraint object packed as a struct
     *
     * @param F number of footsteps, whether that we are considering the
     * control horizon or the prediction horizon
     * @return Inequality
     */
    InequalityConstraint getKinematicConstraint(int F) const;

    /**
     * @brief Get the Theta Constraint object to maintain the theta displacement
     * between two adjacent j within a certain limit
     *
     * @return InequalityConstraint
     */
    InequalityConstraint getThetaConstraint() const;

    /**
     * @brief Get the Theta Cost object such as to minimize the squared error between
     * the difference of two adjacent decision variables theta_j, theta_(j+1) and the
     * theta displacement made by a unicycle template model in a certain
     * time interval decided by the reference velocity
     *
     * @return Cost
     */
    Cost getThetaCost() const;

    /**
     * @brief Get the Position Cost object such as to minimize the squared error
     * between the difference of two adjacent variables x_j, x_(j+1) and the position
     * displacement made by a unicycle template model in a certain time interval
     * decived by the reference velocity
     *
     * @return Cost
     */
    Cost getPositionCost() const;

    // Timing Stuff
    Scalar total_planner_qp_duration = 0.0;
    Scalar total_planner_duration = 0.0;
};

}  // namespace ismpc
