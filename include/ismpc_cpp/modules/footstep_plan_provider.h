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
#include <iostream>

#include "ismpc_cpp/representations/footstep_plan.h"
#include "ismpc_cpp/representations/frame_info.h"
#include "ismpc_cpp/representations/reference.h"
#include "ismpc_cpp/representations/state.h"
#include "ismpc_cpp/tools/config/config.h"
#include "ismpc_cpp/tools/config/robot_config.h"
#include "ismpc_cpp/tools/proxsuite.h"
#include "ismpc_cpp/types/footstep.h"
#include "ismpc_cpp/types/math_types.h"
#include "ismpc_cpp/types/optimization.h"

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
    int num_predicted_footsteps;

    // Parameters
    const int numP = Config::P;
    const Scalar T_p = Config::T_p;
    const Scalar T_c = Config::T_c;
    const Scalar delta = Config::delta;
    const Scalar T_bar = RobotConfig::T_bar;
    const Scalar v_bar = RobotConfig::v_bar;
    const Scalar alpha = RobotConfig::alpha;
    const Scalar ds_percentage = RobotConfig::ds_percentage;
    const Scalar theta_max = RobotConfig::theta_max;
    const Scalar eta = RobotConfig::eta;
    const Scalar dax = RobotConfig::dax;
    const Scalar day = RobotConfig::day;
    const Scalar l = RobotConfig::l;

   public:
    FootstepPlanProvider(const FrameInfo& frame_info, const Reference& reference, const State& state,
                         const FootstepPlan& plan);

    /**
     * @brief Update the footstep plan. This function computes the timing,
     * theta and position sequence by tracking a virtual unicycle model.
     */
    void update(FootstepPlan& plan);

    // /**
    //  * @brief Compute the ZMP midpoints for the moving constraint. The goal
    //  * is to have the ZMP midpoints passing from one foot to another in the
    //  * double support phase and to be on the support foot in the single
    //  * support phase. Thus,
    //  */
    // void computeZmpMidpoints(FootstepPlan& plan);

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
