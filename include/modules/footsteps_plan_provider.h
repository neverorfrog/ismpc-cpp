/**
 * @file FootstepsPlanProvider.hpp
 * @brief Header file for the FootstepsPlanProvider class.
 *
 * This file contains the declaration of the FootstepsPlanProvider module. This module uses
 * trajectory optimization to generate the footstep candidates. These in turn
 * are sent to the Footsteps representation. This representation is then
 * accessed by the ISMPC module.
 */

#pragma once

#include <cmath>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <vector>

#include "libraries/constraint_lib.h"
#include "libraries/cost_lib.h"
#include "representations/footsteps.h"
#include "representations/frame_info.h"
#include "representations/lip_robot.h"
#include "representations/reference.h"
#include "tools/config/config.h"
#include "tools/config/robot_config.h"
#include "tools/math/arithmetic.h"
#include "types/math_types.h"

using proxsuite::proxqp::InitialGuessStatus;
using proxsuite::proxqp::dense::isize;
using proxsuite::proxqp::dense::QP;
using std::nullopt;

using namespace ismpc::Arithmetic;
namespace ismpc {

/**
 * @class FootstepsPlanProvider
 * @brief Class responsible for generating candidate footstep poses for the IS-MPC controller.
 *
 * The FootstepsPlanProvider class uses the last footstep timestamp and the current time to generate
 * potential footstep locations that the IS-MPC controller can use to maintain balance and achieve
 * desired motion.
 */
class FootstepsPlanProvider {
   private:
    const FrameInfo& frame_info;
    const LipRobot& robot;
    const Reference& reference;
    const CostLib& cost_lib;
    const ConstraintLib& constraint_lib;

    const int numP = Config::P;
    const Scalar T_p = Config::T_p;
    const Scalar T_c = Config::T_c;
    const Scalar delta = Config::delta;

    const Scalar T_bar = RobotConfig::T_bar;
    const Scalar v_bar = RobotConfig::v_bar;
    const Scalar alpha = RobotConfig::alpha;
    const Scalar ds_percentage = RobotConfig::ds_percentage;

   public:
    FootstepsPlanProvider(const FrameInfo& frame_info, const LipRobot& robot, const Reference& reference,
                          const CostLib& cost_lib, const ConstraintLib& constraint_lib);

    void update(FootstepsPlan& footsteps);

    /**
     * @brief Get the duration of each footstep. Sets the
     * timestamps in class footsteps
     *
     * @param tk The current time at step k
     */
    void computeTiming(FootstepsPlan& footsteps);

    /**
     * @brief Obtain the optimal theta trajectory
     */
    void computeThetaSequence(FootstepsPlan& footsteps);

    /**
     * @brief Obtain the optimal position trajectory (x and y coordinates)
     */
    void computePositionSequence(FootstepsPlan& footsteps);

    void computeZmpMidpoints(FootstepsPlan& footsteps);
};

}  // namespace ismpc